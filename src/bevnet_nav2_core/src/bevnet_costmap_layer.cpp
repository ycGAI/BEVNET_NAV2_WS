// src/bevnet_costmap_layer.cpp
#include "bevnet_nav2_core/bevnet_costmap_layer.hpp"
#include "nav2_costmap_2d/costmap_math.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "rclcpp/parameter_events_filter.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/convert.h"
#include "tf2_ros/transform_listener.h"

using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;
using nav2_costmap_2d::FREE_SPACE;

namespace bevnet_nav2_core
{

BEVNetCostmapLayer::BEVNetCostmapLayer()
  : has_bevnet_data_(false),
    use_maximum_(false),
    use_bevnet_semantic_(false),  // 默认使用costmap
    semantic_weight_(0.7)
{
}

BEVNetCostmapLayer::~BEVNetCostmapLayer()
{
}

void BEVNetCostmapLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("Unable to lock node");
  }

  // 声明参数
  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("bevnet_topic", rclcpp::ParameterValue("/bevnet/costmap"));
  declareParameter("use_bevnet_semantic", rclcpp::ParameterValue(false));
  declareParameter("semantic_weight", rclcpp::ParameterValue(0.7));
  declareParameter("combination_method", rclcpp::ParameterValue("weighted_average"));
  declareParameter("use_maximum", rclcpp::ParameterValue(false));

  // 获取参数
  node->get_parameter(name_ + ".enabled", enabled_);
  node->get_parameter(name_ + ".bevnet_topic", bevnet_topic_);
  node->get_parameter(name_ + ".use_bevnet_semantic", use_bevnet_semantic_);
  node->get_parameter(name_ + ".semantic_weight", semantic_weight_);
  node->get_parameter(name_ + ".combination_method", combination_method_);
  node->get_parameter(name_ + ".use_maximum", use_maximum_);

  global_frame_ = layered_costmap_->getGlobalFrameID();
  rolling_window_ = layered_costmap_->isRolling();

  // 初始化语义到代价的映射（与Python端一致）
  semantic_to_cost_[0] = FREE_SPACE;                    // free_space
  semantic_to_cost_[1] = 10;                           // road (低代价，优先通行)
  semantic_to_cost_[2] = 30;                           // sidewalk
  semantic_to_cost_[3] = LETHAL_OBSTACLE;              // obstacle
  semantic_to_cost_[4] = NO_INFORMATION;               // unknown

  // 创建TF缓冲区
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // QoS配置
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
  
  // 根据配置选择订阅哪个话题
  if (use_bevnet_semantic_) {
    // 订阅语义地图（BEVMap类型）
    RCLCPP_INFO(logger_, "Subscribing to semantic map at /bevnet/semantic_map");
    semantic_sub_ = node->create_subscription<bevnet_nav2_msgs::msg::BEVMap>(
      "/bevnet/semantic_map",
      qos,
      std::bind(&BEVNetCostmapLayer::semanticMapCallback, this, std::placeholders::_1));
  } else {
    // 订阅代价地图（OccupancyGrid类型）
    RCLCPP_INFO(logger_, "Subscribing to costmap at %s", bevnet_topic_.c_str());
    bevnet_sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
      bevnet_topic_,
      qos,
      std::bind(&BEVNetCostmapLayer::bevnetCallback, this, std::placeholders::_1));
  }

  current_ = true;
  default_value_ = NO_INFORMATION;
  matchSize();

  RCLCPP_INFO(logger_, "BEVNet costmap layer initialized");
  RCLCPP_INFO(logger_, "  - Use semantic: %s", use_bevnet_semantic_ ? "true" : "false");
  RCLCPP_INFO(logger_, "  - Combination method: %s", combination_method_.c_str());
  RCLCPP_INFO(logger_, "  - Semantic weight: %.2f", semantic_weight_);
}

void BEVNetCostmapLayer::bevnetCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  
  RCLCPP_DEBUG(logger_, "Received BEVNet costmap: %dx%d, resolution: %.2f",
               msg->info.width, msg->info.height, msg->info.resolution);
  
  // 更新BEVNet数据缓存
  bevnet_origin_x_ = msg->info.origin.position.x;
  bevnet_origin_y_ = msg->info.origin.position.y;
  bevnet_size_x_ = msg->info.width;
  bevnet_size_y_ = msg->info.height;
  bevnet_resolution_ = msg->info.resolution;
  
  // 缓存数据
  bevnet_data_.resize(bevnet_size_x_ * bevnet_size_y_);
  for (unsigned int i = 0; i < bevnet_data_.size(); ++i) {
    bevnet_data_[i] = interpretCostValue(msg->data[i]);
  }
  
  has_bevnet_data_ = true;
  bevnet_costmap_ = msg;  // 保存原始消息
}

void BEVNetCostmapLayer::semanticMapCallback(
  const bevnet_nav2_msgs::msg::BEVMap::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  
  RCLCPP_DEBUG(logger_, "Received BEVNet semantic map: %dx%d, %d classes",
               msg->info.width, msg->info.height, msg->num_classes);
  
  semantic_map_ = msg;
  
  // 更新地图参数
  bevnet_origin_x_ = msg->info.origin.position.x;
  bevnet_origin_y_ = msg->info.origin.position.y;
  bevnet_size_x_ = msg->info.width;
  bevnet_size_y_ = msg->info.height;
  bevnet_resolution_ = msg->info.resolution;
  
  // 处理语义地图
  processSemanticMap(*msg);
}

void BEVNetCostmapLayer::processSemanticMap(
  const bevnet_nav2_msgs::msg::BEVMap & semantic_map)
{
  // 确保数据大小正确
  if (semantic_map.semantic_map.size() != bevnet_size_x_ * bevnet_size_y_) {
    RCLCPP_ERROR(logger_, "Semantic map size mismatch: expected %u, got %zu",
                 bevnet_size_x_ * bevnet_size_y_, semantic_map.semantic_map.size());
    return;
  }
  
  bevnet_data_.resize(bevnet_size_x_ * bevnet_size_y_);
  
  // 将语义类别转换为代价值
  for (unsigned int i = 0; i < bevnet_data_.size(); ++i) {
    int semantic_class = semantic_map.semantic_map[i];
    
    // 根据映射表转换
    auto it = semantic_to_cost_.find(semantic_class);
    if (it != semantic_to_cost_.end()) {
      bevnet_data_[i] = it->second;
    } else {
      bevnet_data_[i] = NO_INFORMATION;
      RCLCPP_WARN_ONCE(logger_, "Unknown semantic class: %d", semantic_class);
    }
  }
  
  has_bevnet_data_ = true;
  
  // 打印统计信息（调试用）
  if (semantic_map.class_names.size() > 0) {
    RCLCPP_DEBUG(logger_, "Semantic classes:");
    for (size_t i = 0; i < semantic_map.class_names.size(); ++i) {
      RCLCPP_DEBUG(logger_, "  %zu: %s -> cost %d", 
                   i, semantic_map.class_names[i].c_str(), semantic_to_cost_[i]);
    }
  }
}

unsigned char BEVNetCostmapLayer::interpretCostValue(signed char value)
{
  // 处理OccupancyGrid格式的值
  // -1 = 未知, 0-100 = 占用概率
  if (value == -1) {
    return NO_INFORMATION;  // 255
  } else if (value == 0) {
    return FREE_SPACE;      // 0
  } else if (value >= 99) {
    return LETHAL_OBSTACLE; // 254
  } else {
    // 线性映射 1-98 -> 1-252
    return static_cast<unsigned char>(1 + (252 * value) / 98);
  }
}

unsigned char BEVNetCostmapLayer::interpretValue(unsigned char value)
{
  // 保留原有函数以兼容
  if (value == 255) {
    return NO_INFORMATION;
  } else if (value == 0) {
    return FREE_SPACE;
  } else if (value >= 100) {
    return LETHAL_OBSTACLE;
  } else {
    return static_cast<unsigned char>(1 + (251 * value) / 99);
  }
}

void BEVNetCostmapLayer::updateBounds(
  double robot_x, double robot_y, double robot_yaw,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  if (!enabled_ || !has_bevnet_data_) {
    return;
  }

  std::lock_guard<std::mutex> lock(data_mutex_);
  
  if (rolling_window_) {
    // 滚动窗口模式：以机器人为中心
    double half_width = getSizeInMetersX() / 2;
    double half_height = getSizeInMetersY() / 2;
    *min_x = robot_x - half_width;
    *min_y = robot_y - half_height;
    *max_x = robot_x + half_width;
    *max_y = robot_y + half_height;
  } else {
    // 静态地图模式：使用BEVNet覆盖的区域
    double bevnet_max_x = bevnet_origin_x_ + bevnet_size_x_ * bevnet_resolution_;
    double bevnet_max_y = bevnet_origin_y_ + bevnet_size_y_ * bevnet_resolution_;
    
    *min_x = std::min(*min_x, bevnet_origin_x_);
    *min_y = std::min(*min_y, bevnet_origin_y_);
    *max_x = std::max(*max_x, bevnet_max_x);
    *max_y = std::max(*max_y, bevnet_max_y);
  }
  
  RCLCPP_DEBUG(logger_, "Update bounds: [%.2f, %.2f] to [%.2f, %.2f]",
               *min_x, *min_y, *max_x, *max_y);
}

void BEVNetCostmapLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_ || !has_bevnet_data_) {
    return;
  }

  std::lock_guard<std::mutex> lock(data_mutex_);

  // 获取主costmap的参数
  unsigned int master_size_x = master_grid.getSizeInCellsX();
  double master_origin_x = master_grid.getOriginX();
  double master_origin_y = master_grid.getOriginY();
  double master_resolution = master_grid.getResolution();
  
  // 统计更新的单元格数量（调试用）
  int updated_cells = 0;
  int skipped_cells = 0;

  // 遍历需要更新的区域
  for (int j = min_j; j <= max_j; j++) {
    for (int i = min_i; i <= max_i; i++) {
      // 边界检查
      if (i < 0 || i >= static_cast<int>(master_size_x) || 
          j < 0 || j >= static_cast<int>(master_grid.getSizeInCellsY())) {
        continue;
      }
      
      // 计算世界坐标
      double wx = master_origin_x + (i + 0.5) * master_resolution;
      double wy = master_origin_y + (j + 0.5) * master_resolution;

      // 转换到BEVNet坐标系（像素坐标）
      int bevnet_mx = static_cast<int>((wx - bevnet_origin_x_) / bevnet_resolution_);
      int bevnet_my = static_cast<int>((wy - bevnet_origin_y_) / bevnet_resolution_);

      // 检查是否在BEVNet范围内
      if (bevnet_mx >= 0 && bevnet_mx < static_cast<int>(bevnet_size_x_) &&
          bevnet_my >= 0 && bevnet_my < static_cast<int>(bevnet_size_y_))
      {
        // 获取BEVNet的代价值
        unsigned int bevnet_index = bevnet_my * bevnet_size_x_ + bevnet_mx;
        unsigned char bevnet_cost = bevnet_data_[bevnet_index];
        
        // 获取当前代价
        unsigned char old_cost = master_grid.getCost(i, j);
        
        // 处理未知区域
        if (bevnet_cost == NO_INFORMATION) {
          // BEVNet未知，保持原值
          skipped_cells++;
          continue;
        }
        
        // 计算新代价
        unsigned char new_cost;
        
        if (combination_method_ == "maximum" || use_maximum_) {
          // 取最大值（最保守）
          if (old_cost == NO_INFORMATION) {
            new_cost = bevnet_cost;
          } else {
            new_cost = std::max(old_cost, bevnet_cost);
          }
        } else if (combination_method_ == "weighted_average") {
          // 加权平均
          if (old_cost == NO_INFORMATION) {
            new_cost = bevnet_cost;
          } else {
            float weighted = semantic_weight_ * bevnet_cost + 
                           (1.0 - semantic_weight_) * old_cost;
            new_cost = static_cast<unsigned char>(std::min(254.0f, weighted));
          }
        } else if (combination_method_ == "override") {
          // 直接覆盖
          new_cost = bevnet_cost;
        } else {
          // 默认：BEVNet优先
          new_cost = bevnet_cost;
        }
        
        // 更新代价
        master_grid.setCost(i, j, new_cost);
        updated_cells++;
      }
    }
  }
  
  RCLCPP_DEBUG_THROTTLE(logger_, *clock_, 5000,
                        "Updated %d cells, skipped %d cells", 
                        updated_cells, skipped_cells);
}

void BEVNetCostmapLayer::activate()
{
  // 重新初始化
  onInitialize();
  RCLCPP_INFO(logger_, "BEVNet costmap layer activated");
}

void BEVNetCostmapLayer::deactivate()
{
  // 清理订阅
  bevnet_sub_.reset();
  semantic_sub_.reset();
  has_bevnet_data_ = false;
  bevnet_data_.clear();
  RCLCPP_INFO(logger_, "BEVNet costmap layer deactivated");
}

}  // namespace bevnet_nav2_core

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(bevnet_nav2_core::BEVNetCostmapLayer, nav2_costmap_2d::Layer)