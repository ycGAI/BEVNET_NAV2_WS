// src/bevnet_controller.cpp
#include <memory>
#include <string>
#include <vector>
#include <algorithm>

#include "nav2_core/controller.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.h"
#include "angles/angles.h"

namespace bevnet_nav2_core
{

class BEVNetController : public nav2_core::Controller
{
public:
  BEVNetController() = default;
  ~BEVNetController() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override
  {
    node_ = parent;
    auto node = node_.lock();
    costmap_ros_ = costmap_ros;
    tf_buffer_ = tf_buffer;
    name_ = name;
    logger_ = node->get_logger();
    clock_ = node->get_clock();

    // 声明参数
    nav2_util::declare_parameter_if_not_declared(
      node, name_ + ".max_vel_x", rclcpp::ParameterValue(0.5));
    nav2_util::declare_parameter_if_not_declared(
      node, name_ + ".max_vel_theta", rclcpp::ParameterValue(1.0));
    nav2_util::declare_parameter_if_not_declared(
      node, name_ + ".min_vel_x", rclcpp::ParameterValue(0.0));
    nav2_util::declare_parameter_if_not_declared(
      node, name_ + ".min_vel_theta", rclcpp::ParameterValue(-1.0));
    nav2_util::declare_parameter_if_not_declared(
      node, name_ + ".lookahead_dist", rclcpp::ParameterValue(0.6));
    nav2_util::declare_parameter_if_not_declared(
      node, name_ + ".use_velocity_scaled_lookahead_dist", rclcpp::ParameterValue(false));
    nav2_util::declare_parameter_if_not_declared(
      node, name_ + ".min_lookahead_dist", rclcpp::ParameterValue(0.3));
    nav2_util::declare_parameter_if_not_declared(
      node, name_ + ".max_lookahead_dist", rclcpp::ParameterValue(0.9));
    nav2_util::declare_parameter_if_not_declared(
      node, name_ + ".lookahead_time", rclcpp::ParameterValue(1.5));
    nav2_util::declare_parameter_if_not_declared(
      node, name_ + ".bevnet_weight", rclcpp::ParameterValue(0.7));

    // 获取参数
    node->get_parameter(name_ + ".max_vel_x", max_vel_x_);
    node->get_parameter(name_ + ".max_vel_theta", max_vel_theta_);
    node->get_parameter(name_ + ".min_vel_x", min_vel_x_);
    node->get_parameter(name_ + ".min_vel_theta", min_vel_theta_);
    node->get_parameter(name_ + ".lookahead_dist", lookahead_dist_);
    node->get_parameter(name_ + ".use_velocity_scaled_lookahead_dist", use_velocity_scaled_lookahead_dist_);
    node->get_parameter(name_ + ".min_lookahead_dist", min_lookahead_dist_);
    node->get_parameter(name_ + ".max_lookahead_dist", max_lookahead_dist_);
    node->get_parameter(name_ + ".lookahead_time", lookahead_time_);
    node->get_parameter(name_ + ".bevnet_weight", bevnet_weight_);

    // 发布局部路径用于可视化
    local_path_pub_ = node->create_publisher<nav_msgs::msg::Path>("/local_plan", 1);

    RCLCPP_INFO(logger_, "BEVNet Controller configured");
  }

  void activate() override
  {
    RCLCPP_INFO(logger_, "BEVNet Controller activated");
  }

  void deactivate() override
  {
    RCLCPP_INFO(logger_, "BEVNet Controller deactivated");
  }

  void cleanup() override
  {
    RCLCPP_INFO(logger_, "BEVNet Controller cleaning up");
    local_path_pub_.reset();
  }

  void setPlan(const nav_msgs::msg::Path & path) override
  {
    global_plan_ = path;
  }

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override
  {
    // 获取机器人当前位置
    double robot_x = pose.pose.position.x;
    double robot_y = pose.pose.position.y;
    double robot_theta = tf2::getYaw(pose.pose.orientation);

    // 计算前瞻点
    auto lookahead_point = getLookAheadPoint(robot_x, robot_y, velocity);
    
    // 获取代价地图
    nav2_costmap_2d::Costmap2D * costmap = costmap_ros_->getCostmap();
    
    // 基于BEVNet信息调整路径
    adjustPathWithBEVNet(lookahead_point, robot_x, robot_y, costmap);
    
    // 计算到前瞻点的角度
    double angle_to_goal = atan2(
      lookahead_point.pose.position.y - robot_y,
      lookahead_point.pose.position.x - robot_x);
    
    // 计算角度差
    double angle_diff = angles::shortest_angular_distance(robot_theta, angle_to_goal);
    
    // 计算速度命令
    geometry_msgs::msg::TwistStamped cmd_vel;
    cmd_vel.header.frame_id = pose.header.frame_id;
    cmd_vel.header.stamp = clock_->now();
    
    // 线速度控制
    double dist_to_goal = hypot(
      lookahead_point.pose.position.x - robot_x,
      lookahead_point.pose.position.y - robot_y);
    
    if (fabs(angle_diff) > 0.2) {
      // 需要转弯时减速
      cmd_vel.twist.linear.x = min_vel_x_ + (max_vel_x_ - min_vel_x_) * 
                               (1.0 - fabs(angle_diff) / M_PI);
    } else {
      // 直行时加速
      cmd_vel.twist.linear.x = std::min(max_vel_x_, dist_to_goal);
    }
    
    // 角速度控制
    cmd_vel.twist.angular.z = std::max(min_vel_theta_, 
                                      std::min(max_vel_theta_, 2.0 * angle_diff));
    
    // 发布局部路径用于可视化
    publishLocalPath(pose, lookahead_point);
    
    return cmd_vel;
  }

  void setSpeedLimit(const double & speed_limit, const bool & percentage) override
  {
    if (percentage) {
      max_vel_x_ = max_vel_x_ * speed_limit / 100.0;
    } else {
      max_vel_x_ = speed_limit;
    }
  }

protected:
  geometry_msgs::msg::PoseStamped getLookAheadPoint(
    double robot_x, double robot_y,
    const geometry_msgs::msg::Twist & velocity)
  {
    // 计算前瞻距离
    double lookahead_dist = lookahead_dist_;
    if (use_velocity_scaled_lookahead_dist_) {
      lookahead_dist = velocity.linear.x * lookahead_time_;
      lookahead_dist = std::max(min_lookahead_dist_,
                                std::min(max_lookahead_dist_, lookahead_dist));
    }
    
    // 在全局路径上查找前瞻点
    geometry_msgs::msg::PoseStamped lookahead_point;
    double min_dist = std::numeric_limits<double>::max();
    size_t closest_idx = 0;
    
    // 找到最近的路径点
    for (size_t i = 0; i < global_plan_.poses.size(); ++i) {
      double dx = global_plan_.poses[i].pose.position.x - robot_x;
      double dy = global_plan_.poses[i].pose.position.y - robot_y;
      double dist = hypot(dx, dy);
      
      if (dist < min_dist) {
        min_dist = dist;
        closest_idx = i;
      }
    }
    
    // 从最近点开始查找前瞻点
    for (size_t i = closest_idx; i < global_plan_.poses.size(); ++i) {
      double dx = global_plan_.poses[i].pose.position.x - robot_x;
      double dy = global_plan_.poses[i].pose.position.y - robot_y;
      double dist = hypot(dx, dy);
      
      if (dist >= lookahead_dist) {
        lookahead_point = global_plan_.poses[i];
        break;
      }
    }
    
    // 如果没找到，使用最后一个点
    if (lookahead_point.header.frame_id.empty()) {
      lookahead_point = global_plan_.poses.back();
    }
    
    return lookahead_point;
  }

  void adjustPathWithBEVNet(
    geometry_msgs::msg::PoseStamped & lookahead_point,
    double robot_x, double robot_y,
    nav2_costmap_2d::Costmap2D * costmap)
  {
    // 基于BEVNet语义信息调整路径
    // 这里可以实现更复杂的路径调整逻辑
    
    unsigned int mx, my;
    if (!costmap->worldToMap(lookahead_point.pose.position.x, 
                             lookahead_point.pose.position.y, mx, my)) {
      return;
    }
    
    unsigned char cost = costmap->getCost(mx, my);
    
    // 如果前瞻点代价太高，尝试找到附近的低代价点
    if (cost > 200) {
      // 在周围搜索低代价点
      int search_radius = 5;
      double min_cost = cost;
      unsigned int best_mx = mx, best_my = my;
      
      for (int dx = -search_radius; dx <= search_radius; ++dx) {
        for (int dy = -search_radius; dy <= search_radius; ++dy) {
          unsigned int test_mx = mx + dx;
          unsigned int test_my = my + dy;
          
          if (test_mx < costmap->getSizeInCellsX() && 
              test_my < costmap->getSizeInCellsY()) {
            unsigned char test_cost = costmap->getCost(test_mx, test_my);
            
            if (test_cost < min_cost) {
              min_cost = test_cost;
              best_mx = test_mx;
              best_my = test_my;
            }
          }
        }
      }
      
      // 更新前瞻点位置
      if (best_mx != mx || best_my != my) {
        double wx, wy;
        costmap->mapToWorld(best_mx, best_my, wx, wy);
        lookahead_point.pose.position.x = wx;
        lookahead_point.pose.position.y = wy;
      }
    }
  }

  void publishLocalPath(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & end)
  {
    nav_msgs::msg::Path local_path;
    local_path.header = start.header;
    local_path.poses.push_back(start);
    local_path.poses.push_back(end);
    
    local_path_pub_->publish(local_path);
  }

private:
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  rclcpp::Logger logger_{rclcpp::get_logger("BEVNetController")};
  rclcpp::Clock::SharedPtr clock_;
  
  std::string name_;
  nav_msgs::msg::Path global_plan_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_path_pub_;
  
  // 参数
  double max_vel_x_;
  double max_vel_theta_;
  double min_vel_x_;
  double min_vel_theta_;
  double lookahead_dist_;
  bool use_velocity_scaled_lookahead_dist_;
  double min_lookahead_dist_;
  double max_lookahead_dist_;
  double lookahead_time_;
  double bevnet_weight_;
};

}  // namespace bevnet_nav2_core

PLUGINLIB_EXPORT_CLASS(bevnet_nav2_core::BEVNetController, nav2_core::Controller)