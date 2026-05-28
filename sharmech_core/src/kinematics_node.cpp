#include "sharmech_core/kinematics_node.hpp"
#include <rclcpp_components/register_node_macro.hpp>

namespace sharmech_core
{

KinematicsNode::KinematicsNode(const rclcpp::NodeOptions & options)
: Node("kinematics_node", options)
{
  l1_         = declare_parameter("l1", 0.15);
  l2_         = declare_parameter("l2", 0.15);
  base_width_ = declare_parameter("base_width", 0.10);

  target_pose_sub_ = create_subscription<geometry_msgs::msg::Point>(
    "/target_pose", 10,
    std::bind(&KinematicsNode::onTargetPose, this, std::placeholders::_1));

  joint_command_pub_ = create_publisher<sensor_msgs::msg::JointState>("/joint_command", 10);

  RCLCPP_INFO(get_logger(), "kinematics_node started. l1=%.3f l2=%.3f base=%.3f",
    l1_, l2_, base_width_);
}

void KinematicsNode::onTargetPose(const geometry_msgs::msg::Point::SharedPtr msg)
{
  double theta1{0.0}, theta2{0.0};
  if (!inverseKinematics(msg->x, msg->y, theta1, theta2)) {
    RCLCPP_WARN(get_logger(), "IK failed for (x=%.3f, y=%.3f)", msg->x, msg->y);
    return;
  }

  sensor_msgs::msg::JointState cmd;
  cmd.header.stamp = now();
  cmd.name     = {"joint1", "joint2"};
  cmd.position = {theta1, theta2};
  joint_command_pub_->publish(cmd);
}

bool KinematicsNode::inverseKinematics(
  double x, double y, double & theta1, double & theta2)
{
  const double d = base_width_ / 2.0;

  // --- モータ1 (-d, 0) からエンドエフェクタへの2リンクIK ---
  const double dx1 = x + d;
  const double dy1 = y;
  const double r1  = std::hypot(dx1, dy1);

  if (r1 > l1_ + l2_ || r1 < std::abs(l1_ - l2_)) {
    return false;  // 到達不能
  }

  // 余弦定理で肘角度を計算
  const double cos_elbow1 = (l1_ * l1_ + r1 * r1 - l2_ * l2_) / (2.0 * l1_ * r1);
  const double elbow1     = std::acos(std::clamp(cos_elbow1, -1.0, 1.0));
  theta1 = std::atan2(dy1, dx1) - elbow1;  // elbow-up 解

  // --- モータ2 (+d, 0) からエンドエフェクタへの2リンクIK ---
  const double dx2 = x - d;
  const double dy2 = y;
  const double r2  = std::hypot(dx2, dy2);

  if (r2 > l1_ + l2_ || r2 < std::abs(l1_ - l2_)) {
    return false;
  }

  const double cos_elbow2 = (l1_ * l1_ + r2 * r2 - l2_ * l2_) / (2.0 * l1_ * r2);
  const double elbow2     = std::acos(std::clamp(cos_elbow2, -1.0, 1.0));
  theta2 = std::atan2(dy2, dx2) + elbow2;  // elbow-up 解 (右腕は符号逆)

  return true;
}

void KinematicsNode::forwardKinematics(
  double theta1, double theta2, double & x, double & y)
{
  const double d = base_width_ / 2.0;

  // モータ1側の肘位置
  const double ex1 = -d + l1_ * std::cos(theta1);
  const double ey1 =      l1_ * std::sin(theta1);

  // モータ2側の肘位置
  const double ex2 = d + l1_ * std::cos(theta2);
  const double ey2 =     l1_ * std::sin(theta2);

  // 2円の交点がEE位置 (簡易: 中点で近似 / TODO: 厳密な交点計算)
  x = (ex1 + ex2) / 2.0;
  y = (ey1 + ey2) / 2.0;
  (void)ex1; (void)ey1; (void)ex2; (void)ey2;
}

}  // namespace sharmech_core

RCLCPP_COMPONENTS_REGISTER_NODE(sharmech_core::KinematicsNode)
