// src/bevnet_costmap_layer.cpp
#include "bevnet_nav2_core/bevnet_costmap_layer.hpp"
#include "nav2_costmap_2d/costmap_math.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "rclcpp/parameter_events_filter.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/convert.h"
#include "tf2_ros/transform_listener.h"
#include <pluginlib/class_list_macros.hpp>

using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;
using nav2_costmap_2d::FREE_SPACE;

namespace bevnet_nav2_core
{

BEVNetCostmapLayer::BEVNetCostmapLayer()
  : has_bevnet_data_(false),
    use_maximum_(false),
    use_bevnet_semantic_(false),
    semantic_weight_(0.7)
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

  // 初始化语义到代价的映射
  semantic_to_cost_[0] = FREE_SPACE;
  semantic_to_cost_[1] = 10;
  semantic_to_cost_[2] = 30;
  semantic_to_cost_[3] = LETHAL_OBSTACLE;
  semantic_to_cost_[4] = NO_INFORMATION;

  // 创建TF缓冲区
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // QoS配置
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
  
  // 根据配置选择订阅哪个话题
  if (use_bevnet_semantic_) {
    RCLCPP_INFO(node->get_logger(), "Subscribing to semantic map at /bevnet/semantic_map");
    semantic_sub_ = node->create_subscription<bevnet_nav2_msgs::msg::BEVMap>(
      "/bevnet/semantic_map",
      qos,
      std::bind(&BEVNetCostmapLayer::semanticMapCallback, this, std::placeholders::_1));
  } else {
    RCLCPP_INFO(node->get_logger(), "Subscribing to costmap at %s", bevnet_topic_.c_str());
    bevnet_sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
      bevnet_topic_,
      qos,
      std::bind(&BEVNetCostmapLayer::bevnetCallback, this, std::placeholders::_1));
  }

  current_ = true;
  default_value_ = NO_INFORMATION;
  matchSize();

  RCLCPP_INFO(node->get_logger(), "BEVNet costmap layer initialized");
}

void BEVNetCostmapLayer::bevnetCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  
  RCLCPP_DEBUG(rclcpp::get_logger("bevnet_costmap_layer"), 
               "Received BEVNet costmap: %dx%d", msg->info.width, msg->info.height);
  
  bevnet_origin_x_ = msg->info.origin.position.x;
  bevnet_origin_y_ = msg->info.origin.position.y;
  bevnet_size_x_ = msg->info.width;
  bevnet_size_y_ = msg->info.height;
  bevnet_resolution_ = msg->info.resolution;
  
  bevnet_data_.resize(bevnet_size_x_ * bevnet_size_y_);
  for (unsigned int i = 0; i < bevnet_data_.size(); ++i) {
    bevnet_data_[i] = interpretValue(msg->data[i]);
  }
  
  has_bevnet_data_ = true;
  bevnet_costmap_ = msg;
}

void BEVNetCostmapLayer::semanticMapCallback(const bevnet_nav2_msgs::msg::BEVMap::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  
  RCLCPP_DEBUG(rclcpp::get_logger("bevnet_costmap_layer"),
               "Received BEVNet semantic map: %dx%d", msg->info.width, msg->info.height);
  
  semantic_map_ = msg;
  
  bevnet_origin_x_ = msg->info.origin.position.x;
  bevnet_origin_y_ = msg->info.origin.position.y;
  bevnet_size_x_ = msg->info.width;
  bevnet_size_y_ = msg->info.height;
  bevnet_resolution_ = msg->info.resolution;
  
  processSemanticMap(*msg);
}

void BEVNetCostmapLayer::processSemanticMap(const bevnet_nav2_msgs::msg::BEVMap & semantic_map)
{
  if (semantic_map.semantic_map.size() != bevnet_size_x_ * bevnet_size_y_) {
    RCLCPP_ERROR(rclcpp::get_logger("bevnet_costmap_layer"),
                 "Semantic map size mismatch");
    return;
  }
  
  bevnet_data_.resize(bevnet_size_x_ * bevnet_size_y_);
  
  for (unsigned int i = 0; i < bevnet_data_.size(); ++i) {
    int semantic_class = semantic_map.semantic_map[i];
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
  if (value == 255) {
    return NO_INFORMATION;
  } else if (value >= 100) {
    return LETHAL_OBSTACLE;
  } else if (value >= 50) {
    return INSCRIBED_INFLATED_OBSTACLE;
  } else {
    return FREE_SPACE;
  }
}

void BEVNetCostmapLayer::updateBounds(
  double robot_x, double robot_y, double robot_yaw,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  if (!has_bevnet_data_) {
    return;
  }

  *min_x = std::min(*min_x, bevnet_origin_x_);
  *min_y = std::min(*min_y, bevnet_origin_y_);
  *max_x = std::max(*max_x, bevnet_origin_x_ + bevnet_size_x_ * bevnet_resolution_);
  *max_y = std::max(*max_y, bevnet_origin_y_ + bevnet_size_y_ * bevnet_resolution_);
}

void BEVNetCostmapLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  if (!has_bevnet_data_) {
    return;
  }

  std::lock_guard<std::mutex> lock(data_mutex_);

  double master_origin_x = master_grid.getOriginX();
  double master_origin_y = master_grid.getOriginY();
  double master_resolution = master_grid.getResolution();
  unsigned int master_size_x = master_grid.getSizeInCellsX();
  unsigned int master_size_y = master_grid.getSizeInCellsY();

  int bevnet_min_i = static_cast<int>((bevnet_origin_x_ - master_origin_x) / master_resolution);
  int bevnet_min_j = static_cast<int>((bevnet_origin_y_ - master_origin_y) / master_resolution);
  int bevnet_max_i = bevnet_min_i + static_cast<int>(bevnet_size_x_ * bevnet_resolution_ / master_resolution);
  int bevnet_max_j = bevnet_min_j + static_cast<int>(bevnet_size_y_ * bevnet_resolution_ / master_resolution);

  bevnet_min_i = std::max(0, std::max(bevnet_min_i, min_i));
  bevnet_min_j = std::max(0, std::max(bevnet_min_j, min_j));
  bevnet_max_i = std::min(static_cast<int>(master_size_x), std::min(bevnet_max_i, max_i));
  bevnet_max_j = std::min(static_cast<int>(master_size_y), std::min(bevnet_max_j, max_j));

  for (int j = bevnet_min_j; j < bevnet_max_j; j++) {
    for (int i = bevnet_min_i; i < bevnet_max_i; i++) {
      double world_x = master_origin_x + i * master_resolution;
      double world_y = master_origin_y + j * master_resolution;
      
      int bevnet_i = static_cast<int>((world_x - bevnet_origin_x_) / bevnet_resolution_);
      int bevnet_j = static_cast<int>((world_y - bevnet_origin_y_) / bevnet_resolution_);
      
      if (bevnet_i >= 0 && bevnet_i < static_cast<int>(bevnet_size_x_) &&
          bevnet_j >= 0 && bevnet_j < static_cast<int>(bevnet_size_y_)) {
        
        unsigned int bevnet_index = bevnet_j * bevnet_size_x_ + bevnet_i;
        unsigned char bevnet_cost = bevnet_data_[bevnet_index];
        unsigned char current_cost = master_grid.getCost(i, j);
        
        if (combination_method_ == "override") {
          master_grid.setCost(i, j, bevnet_cost);
        } else if (combination_method_ == "maximum" || use_maximum_) {
          master_grid.setCost(i, j, std::max(current_cost, bevnet_cost));
        } else if (combination_method_ == "weighted_average") {
          unsigned char new_cost = static_cast<unsigned char>(
            semantic_weight_ * bevnet_cost + (1.0 - semantic_weight_) * current_cost);
          master_grid.setCost(i, j, new_cost);
        }
      }
    }
  }
}

void BEVNetCostmapLayer::activate()
{
  RCLCPP_INFO(rclcpp::get_logger("bevnet_costmap_layer"), "BEVNet costmap layer activated");
}

void BEVNetCostmapLayer::deactivate()
{
  RCLCPP_INFO(rclcpp::get_logger("bevnet_costmap_layer"), "BEVNet costmap layer deactivated");
  has_bevnet_data_ = false;
}

}  // namespace bevnet_nav2_core

PLUGINLIB_EXPORT_CLASS(bevnet_nav2_core::BEVNetCostmapLayer, nav2_costmap_2d::Layer)
