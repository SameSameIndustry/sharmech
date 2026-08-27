#include "sharmech_core/kinematics_node.hpp"

#include <cmath>
#include <string>

#include <rclcpp_components/register_node_macro.hpp>

namespace sharmech_core
{

namespace
{
// クォータニオンが単位姿勢からどれだけ離れているかを見るための許容誤差。
// このロボットは姿勢を実現できないため、これを超える指令が来たら警告する
constexpr double kOrientationIgnoreTolerance = 1e-3;
}  // namespace

KinematicsNode::KinematicsNode(const rclcpp::NodeOptions & options)
: Node("kinematics_node", options)
{
  geometry_.shoulder.pivot_half_separation_m =
    declare_parameter("shoulder_pivot_half_separation_m", 0.0);
  geometry_.shoulder.proximal_link_length_m =
    declare_parameter("shoulder_proximal_link_length_m", 0.0);
  geometry_.shoulder.distal_link_length_m =
    declare_parameter("shoulder_distal_link_length_m", 0.0);
  geometry_.knee.pivot_half_separation_m =
    declare_parameter("knee_pivot_half_separation_m", 0.0);
  geometry_.knee.proximal_link_length_m =
    declare_parameter("knee_proximal_link_length_m", 0.0);
  geometry_.knee.distal_link_length_m =
    declare_parameter("knee_distal_link_length_m", 0.0);
  geometry_.turntable_axis_x_m = declare_parameter("turntable_axis_x_m", 0.0);
  geometry_.turntable_axis_y_m = declare_parameter("turntable_axis_y_m", 0.0);
  geometry_.knee_base_height_m = declare_parameter("knee_base_height_m", 0.0);

  joint_command_pub_ = create_publisher<sensor_msgs::msg::JointState>(
    "/catchrobo/command/joint", 10);

  cartesian_sub_ = create_subscription<sharmech_msgs::msg::CartesianCommand>(
    "/catchrobo/command/cartesian", 10,
    std::bind(&KinematicsNode::onCartesianCommand, this, std::placeholders::_1));
}

void KinematicsNode::onCartesianCommand(const sharmech_msgs::msg::CartesianCommand::SharedPtr msg)
{
  const auto & o = msg->pose.orientation;
  const bool orientation_is_identity =
    std::abs(o.x) < kOrientationIgnoreTolerance &&
    std::abs(o.y) < kOrientationIgnoreTolerance &&
    std::abs(o.z) < kOrientationIgnoreTolerance &&
    std::abs(o.w - 1.0) < kOrientationIgnoreTolerance;
  if (!orientation_is_identity && !warned_orientation_ignored_) {
    RCLCPP_WARN(
      get_logger(),
      "この5軸パラレルリンク機構は姿勢(pitch/yaw)を実現できません。"
      "Cartesian指令の姿勢成分は無視します (このログは初回のみ)");
    warned_orientation_ignored_ = true;
  }

  const EndEffectorPosition target{
    msg->pose.position.x, msg->pose.position.y, msg->pose.position.z};
  const auto joints = ParallelArmKinematics::inverseKinematics(target, geometry_, last_joints_);
  if (!joints) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "IK unreachable for target (%.3f, %.3f, %.3f)",
      target.x, target.y, target.z);
    return;
  }

  const auto & tw = msg->twist.linear;
  const auto rates = ParallelArmKinematics::inverseVelocity(*joints, tw.x, tw.y, tw.z, geometry_);
  if (!rates) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "IK velocity singular at current joint angles; dropping this sample");
    return;
  }

  last_joints_ = *joints;

  sensor_msgs::msg::JointState js;
  js.header = msg->header;
  js.name = {"shoulder_left", "shoulder_right", "turntable", "knee_left", "knee_right"};
  js.position = {
    joints->shoulder_motor_angle_rad, joints->shoulder_motor_angle_rad,
    joints->turntable_angle_rad,
    joints->knee_motor_angle_rad, joints->knee_motor_angle_rad};
  js.velocity = {
    rates->shoulder_motor_angle_rate_rad_s, rates->shoulder_motor_angle_rate_rad_s,
    rates->turntable_angle_rate_rad_s,
    rates->knee_motor_angle_rate_rad_s, rates->knee_motor_angle_rate_rad_s};

  joint_command_pub_->publish(js);
}

}  // namespace sharmech_core

RCLCPP_COMPONENTS_REGISTER_NODE(sharmech_core::KinematicsNode)
