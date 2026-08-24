#ifndef SHARMECH_CORE__TRAJECTORY_GENERATOR_NODE_HPP_
#define SHARMECH_CORE__TRAJECTORY_GENERATOR_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include "sharmech_core/utility/trajectory_utils.hpp"

namespace sharmech_core
{

// カルテシアン空間での軌道生成ノード
// 位置(x,y,z)は線形補間、姿勢(orientation)はSlerpで補間する
//
// Sub: /goal_pose           (state_manager_node から)
// Sub: /robot/current_pose  (hardware_bridge_node から、始点として使用)
// Pub: /cartesian_trajectory (hardware_bridge_node へ)
//      nav_msgs/Path の各ポーズの header.stamp に到達時刻を格納
class TrajectoryGeneratorNode : public rclcpp::Node
{
public:
  explicit TrajectoryGeneratorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void onGoalPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  nav_msgs::msg::Path generateTrajectory(
    const geometry_msgs::msg::Pose & start,
    const geometry_msgs::msg::Pose & goal) const;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr                trajectory_pub_;

  // 現在のEE姿勢 (hardware_bridge_node から /robot/current_pose を購読)
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_sub_;
  geometry_msgs::msg::Pose current_pose_;

  int    num_waypoints_;  // 補間点数
  double v_max_;          // 最大速度 [m/s]
  double a_max_;          // 最大加速度 [m/s^2]
};

}  // namespace sharmech_core

#endif  // SHARMECH_CORE__TRAJECTORY_GENERATOR_NODE_HPP_
