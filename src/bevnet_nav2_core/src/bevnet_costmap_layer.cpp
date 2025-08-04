#include "bevnet_nav2_core/bevnet_costmap_layer.hpp"
#include "nav2_costmap_2d/costmap_math.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "rclcpp/parameter_events_filter.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/convert.h"

using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;
using nav2_costmap_2d::FREE_SPACE;

namespace bevnet_nav2_core
{

BEVNetCostmapLayer::BEVNetCostmapLayer()
  : has_bevnet_data_(false),
    use_maximum_(true),
    use_bevnet_semantic_(true),
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

  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("bevnet_topic", rclcpp::ParameterValue("/bevnet/costmap"));
  declareParameter("use_bevnet_semantic", rclcpp::ParameterValue(true));
  declareParameter("semantic_weight", rclcpp::ParameterValue(0.7));
  declareParameter("combination_method", rclcpp::ParameterValue("weighted_average"));
  declareParameter("use_maximum", rclcpp::ParameterValue(true));

  node->get_parameter(name_ + ".enabled", enabled_);
  node->get_parameter(name_ + ".bevnet_topic", bevnet_topic_);
  node->get_parameter(name_ + ".use_bevnet_semantic", use_bevnet_semantic_);
  node->get_parameter(name_ + ".semantic_weight", semantic_weight_);
  node->get_parameter(name_ + ".combination_method", combination_method_);
  node->get_parameter(name_ + ".use_maximum", use_maximum_);

  global_frame_ = layered_costmap_->getGlobalFrameID();
  rolling_window_ = layered_costmap_->isRolling();

  // 初始化语义到代价的映射
  semantic_to_cost_[0] = FREE_SPACE;           // 可通行区域
  semantic_to_cost_[1] = 20;                   // 低代价区域
  semantic_to_cost_[2] = 50;                   // 中等代价区域
  semantic_to_cost_[3] = INSCRIBED_INFLATED_OBSTACLE;  // 障碍物边缘
  semantic_to_cost_[4] = LETHAL_OBSTACLE;      // 障碍物
  semantic_to_cost_[5] = NO_INFORMATION;       // 未知区域

  // 创建TF缓冲区
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // 订阅BEVNet输出
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
  
  bevnet_sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
    bevnet_topic_,
    qos,
    std::bind(&BEVNetCostmapLayer::bevnetCallback, this, std::placeholders::_1));

  if (use_bevnet_semantic_) {
    semantic_sub_ = node->create_subscription<bevnet_nav2_msgs::msg::BEVMap>(
      "/bevnet/semantic_map",
      qos,
      std::bind(&BEVNetCostmapLayer::semanticMapCallback, this, std::placeholders::_1));
  }

  current_ = true;
  default_value_ = NO_INFORMATION;
  matchSize();

  RCLCPP_INFO(logger_, "BEVNet costmap layer initialized");
}

void BEVNetCostmapLayer::bevnetCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  bevnet_costmap_ = msg;
  has_bevnet_data_ = true;
  
  // 更新BEVNet数据缓存
  bevnet_origin_x_ = msg->info.origin.position.x;
  bevnet_origin_y_ = msg->info.origin.position.y;
  bevnet_size_x_ = msg->info.width;
  bevnet_size_y_ = msg->info.height;
  bevnet_resolution_ = msg->info.resolution;
  
  bevnet_data_.resize(bevnet_size_x_ * bevnet_size_y_);
  for (unsigned int i = 0; i < bevnet_data_.size(); ++i) {
    bevnet_data_[i] = interpretValue(msg->data[i]);
  }
}

void BEVNetCostmapLayer::semanticMapCallback(
  const bevnet_nav2_msgs::msg::BEVMap::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  semantic_map_ = msg;
  
  if (use_bevnet_semantic_) {
    processSemanticMap(*msg);
  }
}

void BEVNetCostmapLayer::processSemanticMap(
  const bevnet_nav2_msgs::msg::BEVMap & semantic_map)
{
  // 更新BEVNet数据缓存，使用语义信息
  bevnet_origin_x_ = semantic_map.info.origin.position.x;
  bevnet_origin_y_ = semantic_map.info.origin.position.y;
  bevnet_size_x_ = semantic_map.info.width;
  bevnet_size_y_ = semantic_map.info.height;
  bevnet_resolution_ = semantic_map.info.resolution;
  
  bevnet_data_.resize(bevnet_size_x_ * bevnet_size_y_);
  
  for (unsigned int i = 0; i < bevnet_data_.size(); ++i) {
    int semantic_class = semantic_map.semantic_map[i];
    
    // 将语义类别转换为代价值
    auto it = semantic_to_cost_.find(semantic_class);
    if (it != semantic_to_cost_.end()) {
      bevnet_data_[i] = it->second;
    } else {
      bevnet_data_[i] = NO_INFORMATION;
    }
  }
  
  has_bevnet_data_ = true;
}

unsigned char BEVNetCostmapLayer::interpretValue(unsigned char value)
{
  if (value == -1) {
    return NO_INFORMATION;
  } else if (value == 0) {
    return FREE_SPACE;
  } else if (value >= 100) {
    return LETHAL_OBSTACLE;
  } else {
    // 线性映射到[1, 252]
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
    // 对于滚动窗口，更新整个窗口
    *min_x = robot_x - getSizeInMetersX() / 2;
    *min_y = robot_y - getSizeInMetersY() / 2;
    *max_x = robot_x + getSizeInMetersX() / 2;
    *max_y = robot_y + getSizeInMetersY() / 2;
  } else {
    // 对于静态地图，更新BEVNet覆盖的区域
    *min_x = std::min(*min_x, bevnet_origin_x_);
    *min_y = std::min(*min_y, bevnet_origin_y_);
    *max_x = std::max(*max_x, bevnet_origin_x_ + bevnet_size_x_ * bevnet_resolution_);
    *max_y = std::max(*max_y, bevnet_origin_y_ + bevnet_size_y_ * bevnet_resolution_);
  }
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

  // 遍历需要更新的区域
  for (int j = min_j; j < max_j; j++) {
    for (int i = min_i; i < max_i; i++) {
      // 计算世界坐标
      double wx = master_origin_x + i * master_resolution;
      double wy = master_origin_y + j * master_resolution;

      // 转换到BEVNet坐标系
      int bevnet_mx = static_cast<int>((wx - bevnet_origin_x_) / bevnet_resolution_);
      int bevnet_my = static_cast<int>((wy - bevnet_origin_y_) / bevnet_resolution_);

      // 检查是否在BEVNet范围内
      if (bevnet_mx >= 0 && bevnet_mx < static_cast<int>(bevnet_size_x_) &&
          bevnet_my >= 0 && bevnet_my < static_cast<int>(bevnet_size_y_))
      {
        unsigned int bevnet_index = bevnet_my * bevnet_size_x_ + bevnet_mx;
        unsigned char bevnet_cost = bevnet_data_[bevnet_index];
        
        unsigned int master_index = j * master_size_x + i;
        unsigned char old_cost = master_grid.getCost(master_index);
        
        if (old_cost == NO_INFORMATION || bevnet_cost == NO_INFORMATION) {
          continue;
        }

        // 根据组合方法更新代价
        unsigned char new_cost;
        if (combination_method_ == "maximum" || use_maximum_) {
          new_cost = std::max(old_cost, bevnet_cost);
        } else if (combination_method_ == "weighted_average") {
          new_cost = static_cast<unsigned char>(
            semantic_weight_ * bevnet_cost + (1.0 - semantic_weight_) * old_cost);
        } else {
          new_cost = bevnet_cost;  // 覆盖
        }
        
        master_grid.setCost(i, j, new_cost);
      }
    }
  }
}

void BEVNetCostmapLayer::activate()
{
  onInitialize();
}

void BEVNetCostmapLayer::deactivate()
{
  bevnet_sub_.reset();
  semantic_sub_.reset();
  has_bevnet_data_ = false;
}

}  // namespace bevnet_nav2_core

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(bevnet_nav2_core::BEVNetCostmapLayer, nav2_costmap_2d::Layer)