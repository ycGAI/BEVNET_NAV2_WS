// bevnet_controller.cpp - 简化版本（兼容ROS2 Foxy）
// 注意：由于Foxy API限制，这是一个简化实现
// 建议在实际使用中直接使用DWB controller配合BEVNet costmap layer

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

  // Foxy版本的configure函数签名
  void configure(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr & parent,  // 注意：SharedPtr而不是WeakPtr
    std::string name,
    const std::shared_ptr<tf2_ros::Buffer> & tf_buffer,          // 注意：const引用
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> & costmap_ros) override
  {
    node_ = parent;
    costmap_ros_ = costmap_ros;
    tf_buffer_ = tf_buffer;
    name_ = name;
    logger_ = node_->get_logger();
    clock_ = node_->get_clock();

    // 声明参数
    nav2_util::declare_parameter_if_not_declared(
      node_, name_ + ".max_vel_x", rclcpp::ParameterValue(0.5));
    nav2_util::declare_parameter_if_not_declared(
      node_, name_ + ".max_vel_theta", rclcpp::ParameterValue(1.0));
    nav2_util::declare_parameter_if_not_declared(
      node_, name_ + ".min_vel_x", rclcpp::ParameterValue(0.0));
    nav2_util::declare_parameter_if_not_declared(
      node_, name_ + ".min_vel_theta", rclcpp::ParameterValue(-1.0));
    nav2_util::declare_parameter_if_not_declared(
      node_, name_ + ".lookahead_dist", rclcpp::ParameterValue(0.6));
    nav2_util::declare_parameter_if_not_declared(
      node_, name_ + ".goal_tolerance", rclcpp::ParameterValue(0.25));

    // 获取参数
    node_->get_parameter(name_ + ".max_vel_x", max_vel_x_);
    node_->get_parameter(name_ + ".max_vel_theta", max_vel_theta_);
    node_->get_parameter(name_ + ".min_vel_x", min_vel_x_);
    node_->get_parameter(name_ + ".min_vel_theta", min_vel_theta_);
    node_->get_parameter(name_ + ".lookahead_dist", lookahead_dist_);
    node_->get_parameter(name_ + ".goal_tolerance", goal_tolerance_);

    // 发布局部路径用于可视化
    local_path_pub_ = node_->create_publisher<nav_msgs::msg::Path>("/local_plan", 1);

    RCLCPP_INFO(logger_, "BEVNet Controller configured (Foxy compatible)");
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
    global_plan_.poses.clear();
  }

  void setPlan(const nav_msgs::msg::Path & path) override
  {
    global_plan_ = path;
  }

  // Foxy版本的computeVelocityCommands（没有GoalChecker参数）
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity) override
  {
    // 简单的纯追踪控制器实现
    geometry_msgs::msg::TwistStamped cmd_vel;
    cmd_vel.header.frame_id = pose.header.frame_id;
    cmd_vel.header.stamp = clock_->now();

    // 如果没有全局路径，停止
    if (global_plan_.poses.empty()) {
      cmd_vel.twist.linear.x = 0.0;
      cmd_vel.twist.angular.z = 0.0;
      return cmd_vel;
    }

    // 获取机器人当前位置
    double robot_x = pose.pose.position.x;
    double robot_y = pose.pose.position.y;
    double robot_theta = tf2::getYaw(pose.pose.orientation);

    // 找到前瞻点
    geometry_msgs::msg::PoseStamped lookahead_point;
    if (!findLookAheadPoint(robot_x, robot_y, lookahead_point)) {
      // 如果找不到前瞻点，检查是否到达目标
      auto & goal = global_plan_.poses.back();
      double dist_to_goal = hypot(
        goal.pose.position.x - robot_x,
        goal.pose.position.y - robot_y);
      
      if (dist_to_goal < goal_tolerance_) {
        // 到达目标，停止
        cmd_vel.twist.linear.x = 0.0;
        cmd_vel.twist.angular.z = 0.0;
      } else {
        // 向目标点移动
        lookahead_point = goal;
      }
    }

    // 计算到前瞻点的角度
    double angle_to_goal = atan2(
      lookahead_point.pose.position.y - robot_y,
      lookahead_point.pose.position.x - robot_x);
    
    // 计算角度差
    double angle_diff = angles::shortest_angular_distance(robot_theta, angle_to_goal);
    
    // 计算速度命令
    double dist_to_lookahead = hypot(
      lookahead_point.pose.position.x - robot_x,
      lookahead_point.pose.position.y - robot_y);

    // 线速度控制：根据角度差减速
    if (fabs(angle_diff) > 0.5) {
      // 需要转弯时减速
      cmd_vel.twist.linear.x = min_vel_x_ + (max_vel_x_ - min_vel_x_) * 
                               (1.0 - fabs(angle_diff) / M_PI) * 0.5;
    } else {
      // 直行时加速，但根据距离限制速度
      cmd_vel.twist.linear.x = std::min(max_vel_x_, 
                                        std::min(dist_to_lookahead, max_vel_x_));
    }

    // 角速度控制
    cmd_vel.twist.angular.z = std::max(min_vel_theta_, 
                                       std::min(max_vel_theta_, 2.0 * angle_diff));

    // 发布局部路径用于可视化
    publishLocalPath(pose, lookahead_point);

    return cmd_vel;
  }

protected:
  bool findLookAheadPoint(
    double robot_x, double robot_y,
    geometry_msgs::msg::PoseStamped & lookahead_point)
  {
    // 在全局路径中找到前瞻点
    double closest_dist = std::numeric_limits<double>::max();
    size_t closest_idx = 0;
    
    // 找到最近的路径点
    for (size_t i = 0; i < global_plan_.poses.size(); ++i) {
      double dx = global_plan_.poses[i].pose.position.x - robot_x;
      double dy = global_plan_.poses[i].pose.position.y - robot_y;
      double dist = hypot(dx, dy);
      
      if (dist < closest_dist) {
        closest_dist = dist;
        closest_idx = i;
      }
    }
    
    // 从最近点开始，找到前瞻距离的点
    for (size_t i = closest_idx; i < global_plan_.poses.size(); ++i) {
      double dx = global_plan_.poses[i].pose.position.x - robot_x;
      double dy = global_plan_.poses[i].pose.position.y - robot_y;
      double dist = hypot(dx, dy);
      
      if (dist >= lookahead_dist_) {
        lookahead_point = global_plan_.poses[i];
        return true;
      }
    }
    
    // 如果没找到，使用最后一个点
    if (!global_plan_.poses.empty()) {
      lookahead_point = global_plan_.poses.back();
      return true;
    }
    
    return false;
  }

  void publishLocalPath(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & end)
  {
    nav_msgs::msg::Path local_path;
    local_path.header = start.header;
    local_path.header.stamp = clock_->now();
    local_path.poses.push_back(start);
    local_path.poses.push_back(end);
    
    if (local_path_pub_) {
      local_path_pub_->publish(local_path);
    }
  }

private:
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  rclcpp::Logger logger_{rclcpp::get_logger("BEVNetController")};
  rclcpp::Clock::SharedPtr clock_;
  
  std::string name_;
  nav_msgs::msg::Path global_plan_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_path_pub_;
  
  // 参数
  double max_vel_x_ = 0.5;
  double max_vel_theta_ = 1.0;
  double min_vel_x_ = 0.0;
  double min_vel_theta_ = -1.0;
  double lookahead_dist_ = 0.6;
  double goal_tolerance_ = 0.25;
};

}  // namespace bevnet_nav2_core

PLUGINLIB_EXPORT_CLASS(bevnet_nav2_core::BEVNetController, nav2_core::Controller)