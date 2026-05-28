#ifndef SHARMECH_CORE__HARDWARE_BRIDGE_NODE_HPP_
#define SHARMECH_CORE__HARDWARE_BRIDGE_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

namespace sharmech_core
{

// MCU通信ノード (低レイヤーブリッジ)
//
// Sub: /joint_trajectory  (kinematics_node から)
// Sub: /gripper/command   (state_manager_node から)
// Pub: /joint_states      (state_manager_node へ)
//
// 制御タイマー (control_rate Hz) で軌道を順番に送信する
// エンコーダ値を読み取り /joint_states として配信する
class HardwareBridgeNode : public rclcpp::Node
{
public:
  explicit HardwareBridgeNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~HardwareBridgeNode();

private:
  void onJointTrajectory(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg);
  void onGripperCommand(const std_msgs::msg::Bool::SharedPtr msg);
  void controlLoop();

  void sendJointCommand(double theta1, double theta2);
  void sendGripperCommand(bool grasp);
  void readEncoders(double & theta1, double & theta2);

  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr                   gripper_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr             joint_states_pub_;
  rclcpp::TimerBase::SharedPtr                                           control_timer_;

  // 実行中の軌道
  trajectory_msgs::msg::JointTrajectory current_traj_;
  size_t traj_index_{0};
  bool   is_executing_{false};

  // 現在のフィードバック
  double current_theta1_{0.0};
  double current_theta2_{0.0};

  double control_rate_;  // [Hz]
};

}  // namespace sharmech_core

#endif  // SHARMECH_CORE__HARDWARE_BRIDGE_NODE_HPP_
