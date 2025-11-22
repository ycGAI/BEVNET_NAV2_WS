// bevnet_costmap_layer.hpp
#ifndef BEVNET_NAV2_CORE__BEVNET_COSTMAP_LAYER_HPP_
#define BEVNET_NAV2_CORE__BEVNET_COSTMAP_LAYER_HPP_

#include <memory>
#include <string>
#include <vector>
#include <mutex>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

namespace bevnet_nav2_core
{

class BEVNetCostmapLayer : public nav2_costmap_2d::Layer
{
public:
  BEVNetCostmapLayer();
  virtual ~BEVNetCostmapLayer();

  virtual void onInitialize();
  virtual void updateBounds(
    double robot_x, double robot_y, double robot_yaw, double * min_x,
    double * min_y, double * max_x, double * max_y);
  virtual void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j);
  
  virtual void activate();
  virtual void deactivate();
  virtual void reset() {}

private:
  void bevnetCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void semanticCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg);
  
  // 订阅器
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr bevnet_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr semantic_sub_;
  
  // 参数
  std::string bevnet_topic_;
  std::string semantic_topic_;
  std::string combination_method_;
  bool use_bevnet_semantic_;
  double semantic_weight_;
  bool use_maximum_;
  
  // Layer基础变量（Nav2需要的）
  std::string global_frame_;
  bool rolling_window_;
  unsigned char default_value_;
  
  // BEVNet数据
  nav_msgs::msg::OccupancyGrid::SharedPtr bevnet_costmap_;
  std::vector<unsigned char> bevnet_data_;
  bool has_bevnet_data_;
  
  // BEVNet地图参数
  double bevnet_origin_x_;
  double bevnet_origin_y_;
  unsigned int bevnet_size_x_;
  unsigned int bevnet_size_y_;
  double bevnet_resolution_;
  std::string bevnet_frame_id_;
  
  // 语义数据
  std::vector<int> semantic_data_;
  bool has_semantic_data_;
  
  // 语义到代价值的映射
  std::unordered_map<int, unsigned char> semantic_to_cost_;
  
  // 互斥锁
  std::mutex data_mutex_;
  
  // 注意：不再需要tf_buffer_成员变量
  // 使用父类提供的 tf_ 成员变量
};

}  // namespace bevnet_nav2_core

#endif  // BEVNET_NAV2_CORE__BEVNET_COSTMAP_LAYER_HPP_