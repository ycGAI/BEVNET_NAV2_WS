#ifndef BEVNET_NAV2_CORE__BEVNET_COSTMAP_LAYER_HPP_
#define BEVNET_NAV2_CORE__BEVNET_COSTMAP_LAYER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "bevnet_nav2_msgs/msg/bev_map.hpp"
#include "bevnet_nav2_msgs/msg/semantic_costmap.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"

namespace bevnet_nav2_core
{

class BEVNetCostmapLayer : public nav2_costmap_2d::CostmapLayer
{
public:
  BEVNetCostmapLayer();
  virtual ~BEVNetCostmapLayer();

  virtual void onInitialize();
  virtual void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y);
  virtual void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j);

  virtual void reset()
  {
    deactivate();
    activate();
  }

  virtual void activate();
  virtual void deactivate();

  virtual bool isClearable() {return false;}

private:
  void bevnetCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void semanticMapCallback(const bevnet_nav2_msgs::msg::BEVMap::SharedPtr msg);
  
  unsigned char interpretValue(unsigned char value);
  void processSemanticMap(const bevnet_nav2_msgs::msg::BEVMap & semantic_map);
  void updateCostmapFromBEVNet();

  std::string global_frame_;
  std::string bevnet_topic_;
  bool use_bevnet_semantic_;
  double semantic_weight_;
  std::string combination_method_;
  
  nav_msgs::msg::OccupancyGrid::SharedPtr bevnet_costmap_;
  bevnet_nav2_msgs::msg::BEVMap::SharedPtr semantic_map_;
  
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr bevnet_sub_;
  rclcpp::Subscription<bevnet_nav2_msgs::msg::BEVMap>::SharedPtr semantic_sub_;
  
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  
  bool has_bevnet_data_;
  bool rolling_window_;
  bool use_maximum_;
  double update_frequency_;
  
  // 语义类别到代价的映射
  std::map<int, unsigned char> semantic_to_cost_;
  
  std::mutex data_mutex_;
  
  // BEVNet数据缓存
  std::vector<unsigned char> bevnet_data_;
  double bevnet_origin_x_;
  double bevnet_origin_y_;
  unsigned int bevnet_size_x_;
  unsigned int bevnet_size_y_;
  double bevnet_resolution_;
};

}  // namespace bevnet_nav2_core

#endif  // BEVNET_NAV2_CORE__BEVNET_COSTMAP_LAYER_HPP_