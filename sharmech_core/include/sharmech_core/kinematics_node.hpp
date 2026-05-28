#ifndef SHARMECH_CORE__KINEMATICS_NODE_HPP_
#define SHARMECH_CORE__KINEMATICS_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include "sharmech_core/utility/five_bar_kinematics.hpp"

namespace sharmech_core
{

// 5節リンク逆運動学ノード
// カルテシアン軌道の各ウェイポイントに対してIKを解き
// 関節角度の時系列 (JointTrajectory) を生成する
//
// Sub: /cartesian_trajectory (trajectory_generator_node から)
// Pub: /joint_trajectory     (hardware_bridge_node, state_manager_node へ)
class KinematicsNode : public rclcpp::Node
{
public:
  explicit KinematicsNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void onCartesianTrajectory(const nav_msgs::msg::Path::SharedPtr msg);

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr              cartesian_traj_sub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_traj_pub_;

  FiveBarParams params_;
};

}  // namespace sharmech_core

#endif  // SHARMECH_CORE__KINEMATICS_NODE_HPP_
