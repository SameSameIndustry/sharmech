#ifndef SHARMECH_CORE__KINEMATICS_NODE_HPP_
#define SHARMECH_CORE__KINEMATICS_NODE_HPP_

#include <optional>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sharmech_msgs/msg/cartesian_command.hpp>

#include "sharmech_core/utility/parallel_arm_kinematics.hpp"

namespace sharmech_core
{

// パターンB 用。/catchrobo/command/cartesian (motion_generator_node が出す
// 位置+速度ストリーム) を購読し、5モータ個別の関節角+角速度に変換して
// /catchrobo/command/joint に publish する。
//
// このロボット (5軸パラレルリンク) は座標 (x,y,z) のみを実現でき、姿勢
// (pitch/yaw) の自由度を持たない。Cartesian指令の姿勢成分は無視する
// (要求された場合は初回のみ警告)。詳細は CLAUDE.md 「ロボット構成」節。
//
// IKには2解ありうる (SymmetricLinkage::computeMotorAngle 参照)ため、
// 直前の解を reference として渡し、関節角が滑らかに繋がるようにする。
//
// Sub: /catchrobo/command/cartesian
// Pub: /catchrobo/command/joint (sensor_msgs/JointState.
//      name = [shoulder_left, shoulder_right, turntable, knee_left, knee_right])
class KinematicsNode : public rclcpp::Node
{
public:
  explicit KinematicsNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void onCartesianCommand(const sharmech_msgs::msg::CartesianCommand::SharedPtr msg);

  rclcpp::Subscription<sharmech_msgs::msg::CartesianCommand>::SharedPtr cartesian_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_command_pub_;

  ParallelArmGeometry geometry_;
  std::optional<ParallelArmJointAngles> last_joints_;  // IKの2解を連続性で選ぶための直前解

  bool warned_orientation_ignored_{false};
};

}  // namespace sharmech_core

#endif  // SHARMECH_CORE__KINEMATICS_NODE_HPP_
