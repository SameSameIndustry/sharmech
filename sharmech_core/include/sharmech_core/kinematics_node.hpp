#ifndef SHARMECH_CORE__KINEMATICS_NODE_HPP_
#define SHARMECH_CORE__KINEMATICS_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <cmath>

namespace sharmech_core
{

// 5節リンク(5-bar linkage)の逆運動学・順運動学ノード
// 構成: 2つのモータ(joint1, joint2)がリンクを介してエンドエフェクタを駆動
//
//   motor1(-d/2, 0) ---l1--- elbow_L ---l2---+
//                                            | EE (x, y)
//   motor2(+d/2, 0) ---l1--- elbow_R ---l2---+
//
// Subscribe: /target_pose  (geometry_msgs/Point)  - 目標位置 (x, y)
// Publish  : /joint_command (sensor_msgs/JointState) - 関節角度指令 (θ1, θ2)
class KinematicsNode : public rclcpp::Node
{
public:
  explicit KinematicsNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void onTargetPose(const geometry_msgs::msg::Point::SharedPtr msg);

  // 逆運動学: 目標位置 (x, y) → 関節角度 (θ1, θ2)
  // 戻り値: 解が存在すれば true
  bool inverseKinematics(double x, double y, double & theta1, double & theta2);

  // 順運動学: 関節角度 (θ1, θ2) → エンドエフェクタ位置 (x, y)
  void forwardKinematics(double theta1, double theta2, double & x, double & y);

  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr target_pose_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_command_pub_;

  // 機構パラメータ (config.yaml から読み込み)
  double l1_;          // 第1リンク長 [m]
  double l2_;          // 第2リンク長 [m]
  double base_width_;  // 2モータ間の距離 [m]
};

}  // namespace sharmech_core

#endif  // SHARMECH_CORE__KINEMATICS_NODE_HPP_
