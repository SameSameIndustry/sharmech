#include "sharmech_core/kinematics_node.hpp"
#include <rclcpp_components/register_node_macro.hpp>

namespace sharmech_core
{

KinematicsNode::KinematicsNode(const rclcpp::NodeOptions & options)
: Node("kinematics_node", options)
{
  params_.l1         = declare_parameter("l1", 0.15);
  params_.l2         = declare_parameter("l2", 0.15);
  params_.base_width = declare_parameter("base_width", 0.10);

  cartesian_traj_sub_ = create_subscription<nav_msgs::msg::Path>(
    "/cartesian_trajectory", 10,
    std::bind(&KinematicsNode::onCartesianTrajectory, this, std::placeholders::_1));

  joint_traj_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
    "/joint_trajectory", 10);

  RCLCPP_INFO(get_logger(), "kinematics_node started. l1=%.3f l2=%.3f base=%.3f",
    params_.l1, params_.l2, params_.base_width);
}

void KinematicsNode::onCartesianTrajectory(const nav_msgs::msg::Path::SharedPtr msg)
{
  trajectory_msgs::msg::JointTrajectory joint_traj;
  joint_traj.header   = msg->header;
  joint_traj.joint_names = {"joint1", "joint2"};

  for (const auto & pose_stamped : msg->poses) {
    const CartesianPoint target{
      pose_stamped.pose.position.x,
      pose_stamped.pose.position.y
    };

    const auto result = FiveBarKinematics::inverseKinematics(target, params_);
    if (!result) {
      RCLCPP_WARN(get_logger(), "IK failed at (%.3f, %.3f). Skipping waypoint.",
        target.x, target.y);
      continue;
    }

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions  = {result->theta1, result->theta2};
    point.velocities = {0.0, 0.0};

    // 到達時刻を time_from_start に変換
    const double t_sec = static_cast<double>(pose_stamped.header.stamp.nanosec) * 1e-9
                       + static_cast<double>(pose_stamped.header.stamp.sec);
    point.time_from_start = rclcpp::Duration::from_seconds(t_sec);

    joint_traj.points.push_back(point);
  }

  if (joint_traj.points.empty()) {
    RCLCPP_ERROR(get_logger(), "All IK solutions failed. Trajectory not published.");
    return;
  }

  joint_traj_pub_->publish(joint_traj);
  RCLCPP_INFO(get_logger(), "joint_trajectory published: %zu points",
    joint_traj.points.size());
}

}  // namespace sharmech_core

RCLCPP_COMPONENTS_REGISTER_NODE(sharmech_core::KinematicsNode)
