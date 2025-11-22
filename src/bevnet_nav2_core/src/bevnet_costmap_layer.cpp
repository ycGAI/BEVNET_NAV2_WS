// bevnet_costmap_layer.cpp - 使用Nav2共享TF buffer的版本
#include "bevnet_nav2_core/bevnet_costmap_layer.hpp"
#include "nav2_costmap_2d/costmap_math.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "rclcpp/parameter_events_filter.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/convert.h"

using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;
using nav2_costmap_2d::FREE_SPACE;

namespace bevnet_nav2_core
{

BEVNetCostmapLayer::BEVNetCostmapLayer()
  : has_bevnet_data_(false),
    use_bevnet_semantic_(false),
    semantic_weight_(0.7),
    use_maximum_(false),
    rolling_window_(false),
    default_value_(nav2_costmap_2d::NO_INFORMATION),
    has_semantic_data_(false)
{
}

BEVNetCostmapLayer::~BEVNetCostmapLayer()
{
}

void BEVNetCostmapLayer::onInitialize()
{
  auto node = node_;
  if (!node) {
    throw std::runtime_error("Unable to get node");
  }

  auto logger = node->get_logger();
  
  // 声明参数
  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("bevnet_topic", rclcpp::ParameterValue("/bevnet/costmap"));
  declareParameter("use_bevnet_semantic", rclcpp::ParameterValue(false));
  declareParameter("semantic_weight", rclcpp::ParameterValue(0.7));
  declareParameter("combination_method", rclcpp::ParameterValue("override"));
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

  // 初始化语义到代价的映射
  semantic_to_cost_[0] = FREE_SPACE;                    
  semantic_to_cost_[1] = 10;                           
  semantic_to_cost_[2] = 30;                           
  semantic_to_cost_[3] = LETHAL_OBSTACLE;              
  semantic_to_cost_[4] = NO_INFORMATION;               

  // 重要！使用父类的TF buffer，而不是创建新的
  // tf_ 已经在父类 nav2_costmap_2d::Layer 中定义
  // 不需要创建新的 tf_buffer_

  // QoS配置
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
  
  // 订阅BEVNet costmap
  RCLCPP_INFO(logger, "Subscribing to BEVNet costmap at %s", bevnet_topic_.c_str());
  bevnet_sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
    bevnet_topic_,
    qos,
    std::bind(&BEVNetCostmapLayer::bevnetCallback, this, std::placeholders::_1));

  current_ = true;
  default_value_ = NO_INFORMATION;
  matchSize();

  RCLCPP_INFO(logger, "BEVNet costmap layer initialized");
  RCLCPP_INFO(logger, "  - Combination method: %s", combination_method_.c_str());
  RCLCPP_INFO(logger, "  - Global frame: %s", global_frame_.c_str());
  RCLCPP_INFO(logger, "  - Rolling window: %s", rolling_window_ ? "true" : "false");
}

void BEVNetCostmapLayer::bevnetCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  
  // 保存原始消息
  bevnet_costmap_ = msg;
  
  // 存储BEVNet地图参数
  bevnet_origin_x_ = msg->info.origin.position.x;
  bevnet_origin_y_ = msg->info.origin.position.y;
  bevnet_size_x_ = msg->info.width;
  bevnet_size_y_ = msg->info.height;
  bevnet_resolution_ = msg->info.resolution;
  bevnet_frame_id_ = msg->header.frame_id;
  
  // 调整缓存大小
  bevnet_data_.resize(bevnet_size_x_ * bevnet_size_y_);
  
  // 转换数据格式
  for (size_t i = 0; i < msg->data.size(); ++i) {
    signed char value = msg->data[i];
    
    if (value == -1) {
      bevnet_data_[i] = NO_INFORMATION;
    } else if (value == 0) {
      bevnet_data_[i] = FREE_SPACE;
    } else if (value >= 99) {
      bevnet_data_[i] = LETHAL_OBSTACLE;
    } else if (value >= 90) {
      bevnet_data_[i] = INSCRIBED_INFLATED_OBSTACLE;
    } else {
      bevnet_data_[i] = static_cast<unsigned char>(1 + (value * 252.0 / 89.0));
    }
  }
  
  has_bevnet_data_ = true;
  
  if (node_) {
    auto logger = node_->get_logger();
    auto clock = node_->get_clock();
    RCLCPP_DEBUG_THROTTLE(logger, *clock, 5000,
      "Received BEVNet map: %dx%d cells, resolution %.2f m/cell, frame: %s",
      bevnet_size_x_, bevnet_size_y_, bevnet_resolution_, bevnet_frame_id_.c_str());
  }
}

void BEVNetCostmapLayer::semanticCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  
  // 存储语义数据
  semantic_data_.clear();
  semantic_data_.reserve(msg->data.size());
  for (auto val : msg->data) {
    semantic_data_.push_back(val);
  }
  has_semantic_data_ = true;
  
  if (node_) {
    auto logger = node_->get_logger();
    auto clock = node_->get_clock();
    RCLCPP_DEBUG_THROTTLE(logger, *clock, 5000,
      "Received semantic map with %zu elements", semantic_data_.size());
  }
}

void BEVNetCostmapLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  if (!enabled_ || !has_bevnet_data_) {
    return;
  }

  std::lock_guard<std::mutex> lock(data_mutex_);
  
  // 使用父类提供的tf_
  if (!tf_) {
    if (node_) {
      auto logger = node_->get_logger();
      RCLCPP_DEBUG(logger, "TF buffer not available yet");
    }
    return;
  }
  
  // 尝试获取变换
  if (bevnet_frame_id_ != global_frame_) {
    try {
      // 使用更长的超时时间
      auto timeout = tf2::durationFromSec(1.0);
      geometry_msgs::msg::TransformStamped transform = 
        tf_->lookupTransform(global_frame_, bevnet_frame_id_, 
                            tf2::TimePointZero, timeout);
      
      // 使用实际的传感器位置
      double sensor_x = transform.transform.translation.x;
      double sensor_y = transform.transform.translation.y;
      
      // 提取yaw角
      tf2::Quaternion q(
        transform.transform.rotation.x,
        transform.transform.rotation.y,
        transform.transform.rotation.z,
        transform.transform.rotation.w);
      tf2::Matrix3x3 m(q);
      double roll, pitch, sensor_yaw;
      m.getRPY(roll, pitch, sensor_yaw);
      
      double cos_yaw = cos(sensor_yaw);
      double sin_yaw = sin(sensor_yaw);
      
      // BEVNet地图的四个角
      double corners_x[4] = {
        bevnet_origin_x_,
        bevnet_origin_x_ + bevnet_size_x_ * bevnet_resolution_,
        bevnet_origin_x_ + bevnet_size_x_ * bevnet_resolution_,
        bevnet_origin_x_
      };
      double corners_y[4] = {
        bevnet_origin_y_,
        bevnet_origin_y_,
        bevnet_origin_y_ + bevnet_size_y_ * bevnet_resolution_,
        bevnet_origin_y_ + bevnet_size_y_ * bevnet_resolution_
      };
      
      // 转换到全局坐标并找到边界
      for (int i = 0; i < 4; i++) {
        double global_x = sensor_x + cos_yaw * corners_x[i] - sin_yaw * corners_y[i];
        double global_y = sensor_y + sin_yaw * corners_x[i] + cos_yaw * corners_y[i];
        
        *min_x = std::min(*min_x, global_x);
        *max_x = std::max(*max_x, global_x);
        *min_y = std::min(*min_y, global_y);
        *max_y = std::max(*max_y, global_y);
      }
    } catch (tf2::TransformException & ex) {
      if (node_) {
        auto logger = node_->get_logger();
        auto clock = node_->get_clock();
        RCLCPP_DEBUG_THROTTLE(logger, *clock, 1000, 
          "Could not get transform from %s to %s: %s", 
          bevnet_frame_id_.c_str(), global_frame_.c_str(), ex.what());
      }
    }
  } else {
    // Frame相同，直接使用
    double bevnet_max_x = bevnet_origin_x_ + bevnet_size_x_ * bevnet_resolution_;
    double bevnet_max_y = bevnet_origin_y_ + bevnet_size_y_ * bevnet_resolution_;
    
    *min_x = std::min(*min_x, bevnet_origin_x_);
    *min_y = std::min(*min_y, bevnet_origin_y_);
    *max_x = std::max(*max_x, bevnet_max_x);
    *max_y = std::max(*max_y, bevnet_max_y);
  }
}

void BEVNetCostmapLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_ || !has_bevnet_data_) {
    return;
  }

  // 使用父类提供的tf_
  if (!tf_) {
    if (node_) {
      auto logger = node_->get_logger();
      auto clock = node_->get_clock();
      RCLCPP_DEBUG_THROTTLE(logger, *clock, 1000, "TF buffer not available yet");
    }
    return;
  }

  std::lock_guard<std::mutex> lock(data_mutex_);

  // 获取主costmap的参数
  unsigned int master_size_x = master_grid.getSizeInCellsX();
  unsigned int master_size_y = master_grid.getSizeInCellsY();
  double master_origin_x = master_grid.getOriginX();
  double master_origin_y = master_grid.getOriginY();
  double master_resolution = master_grid.getResolution();
  
  std::string master_frame = layered_costmap_->getGlobalFrameID();
  
  // 获取变换
  geometry_msgs::msg::TransformStamped transform;
  bool have_transform = false;
  
  if (bevnet_frame_id_ != master_frame) {
    try {
      auto timeout = tf2::durationFromSec(1.0);
      transform = tf_->lookupTransform(master_frame, bevnet_frame_id_, 
                                      tf2::TimePointZero, timeout);
      have_transform = true;
    } catch (tf2::TransformException & ex) {
      if (node_) {
        auto logger = node_->get_logger();
        auto clock = node_->get_clock();
        RCLCPP_DEBUG_THROTTLE(logger, *clock, 1000, 
          "Waiting for transform from %s to %s: %s", 
          bevnet_frame_id_.c_str(), master_frame.c_str(), ex.what());
      }
      return;
    }
  } else {
    // Frame相同，使用单位变换
    transform.transform.translation.x = 0.0;
    transform.transform.translation.y = 0.0;
    transform.transform.translation.z = 0.0;
    transform.transform.rotation.x = 0.0;
    transform.transform.rotation.y = 0.0;
    transform.transform.rotation.z = 0.0;
    transform.transform.rotation.w = 1.0;
    have_transform = true;
  }
  
  if (!have_transform) {
    return;
  }
  
  // 获取传感器位置
  double sensor_x = transform.transform.translation.x;
  double sensor_y = transform.transform.translation.y;
  
  // 提取yaw角
  tf2::Quaternion q(
    transform.transform.rotation.x,
    transform.transform.rotation.y,
    transform.transform.rotation.z,
    transform.transform.rotation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, sensor_yaw;
  m.getRPY(roll, pitch, sensor_yaw);
  
  double cos_yaw = cos(sensor_yaw);
  double sin_yaw = sin(sensor_yaw);
  
  // 遍历需要更新的区域
  for (int j = min_j; j <= max_j; j++) {
    for (int i = min_i; i <= max_i; i++) {
      // 边界检查
      if (i < 0 || i >= static_cast<int>(master_size_x) || 
          j < 0 || j >= static_cast<int>(master_size_y)) {
        continue;
      }
      
      // 计算世界坐标（在master frame中）
      double wx = master_origin_x + (i + 0.5) * master_resolution;
      double wy = master_origin_y + (j + 0.5) * master_resolution;
      
      // 转换到BEVNet坐标系（传感器坐标系）
      double dx = wx - sensor_x;
      double dy = wy - sensor_y;
      double bevnet_x = cos_yaw * dx + sin_yaw * dy;
      double bevnet_y = -sin_yaw * dx + cos_yaw * dy;
      
      // 转换到BEVNet像素坐标
      int bevnet_mx = static_cast<int>((bevnet_x - bevnet_origin_x_) / bevnet_resolution_);
      int bevnet_my = static_cast<int>((bevnet_y - bevnet_origin_y_) / bevnet_resolution_);

      // 检查是否在BEVNet范围内
      if (bevnet_mx >= 0 && bevnet_mx < static_cast<int>(bevnet_size_x_) &&
          bevnet_my >= 0 && bevnet_my < static_cast<int>(bevnet_size_y_))
      {
        unsigned int bevnet_index = bevnet_my * bevnet_size_x_ + bevnet_mx;
        if (bevnet_index < bevnet_data_.size()) {
          unsigned char bevnet_cost = bevnet_data_[bevnet_index];
          
          // 根据combination_method处理
          if (combination_method_ == "override") {
            master_grid.setCost(i, j, bevnet_cost);
          } else {
            unsigned char old_cost = master_grid.getCost(i, j);
            
            if (bevnet_cost == NO_INFORMATION && old_cost != NO_INFORMATION) {
              continue;
            }
            
            unsigned char new_cost;
            if (combination_method_ == "maximum" || use_maximum_) {
              if (old_cost == NO_INFORMATION) {
                new_cost = bevnet_cost;
              } else {
                new_cost = std::max(old_cost, bevnet_cost);
              }
            } else {
              new_cost = bevnet_cost;
            }
            
            master_grid.setCost(i, j, new_cost);
          }
        }
      }
    }
  }
}

void BEVNetCostmapLayer::activate()
{
  // 重新初始化
  onInitialize();
  if (node_) {
    auto logger = node_->get_logger();
    RCLCPP_INFO(logger, "BEVNet costmap layer activated");
  }
}

void BEVNetCostmapLayer::deactivate()
{
  // 清理订阅
  bevnet_sub_.reset();
  semantic_sub_.reset();
  has_bevnet_data_ = false;
  bevnet_data_.clear();
  if (node_) {
    auto logger = node_->get_logger();
    RCLCPP_INFO(logger, "BEVNet costmap layer deactivated");
  }
}

}  // namespace bevnet_nav2_core

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(bevnet_nav2_core::BEVNetCostmapLayer, nav2_costmap_2d::Layer)