#ifndef SHARMECH_CORE__ARM_CONTROLLER_NODE_HPP_
#define SHARMECH_CORE__ARM_CONTROLLER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace sharmech_core
{

// モータドライバとのインターフェースノード
// 関節角度指令を受け取りハードウェアへ送信し、現在の関節角度を配信する
//
// Subscribe: /joint_command (sensor_msgs/JointState) - 関節角度指令
// Publish  : /joint_states  (sensor_msgs/JointState) - 現在の関節角度
class ArmControllerNode : public rclcpp::Node
{
public:
  explicit ArmControllerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~ArmControllerNode();

private:
  void onJointCommand(const sensor_msgs::msg::JointState::SharedPtr msg);
  void publishJointStates();

  // ハードウェアへ角度指令を送信 (シリアル/CAN等に応じて実装)
  void sendCommandToHardware(double theta1, double theta2);

  // ハードウェアから現在角度を読み込み
  void readFromHardware(double & theta1, double & theta2);

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_command_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  double current_theta1_{0.0};
  double current_theta2_{0.0};
  double command_theta1_{0.0};
  double command_theta2_{0.0};
};

}  // namespace sharmech_core

#endif  // SHARMECH_CORE__ARM_CONTROLLER_NODE_HPP_
