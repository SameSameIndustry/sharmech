#include "sharmech_core/trajectory_generator_node.hpp"
#include <rclcpp_components/register_node_macro.hpp>

namespace sharmech_core
{

TrajectoryGeneratorNode::TrajectoryGeneratorNode(const rclcpp::NodeOptions & options)
: Node("trajectory_generator_node", options)
{
  num_waypoints_ = declare_parameter("num_waypoints", 50);
  v_max_         = declare_parameter("v_max", 0.1);
  a_max_         = declare_parameter("a_max", 0.2);

  goal_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "/goal_pose", 10,
    std::bind(&TrajectoryGeneratorNode::onGoalPose, this, std::placeholders::_1));

  current_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "/robot/current_pose", 10,
    [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
      current_pos_ = msg->pose.position;
    });

  trajectory_pub_ = create_publisher<nav_msgs::msg::Path>("/cartesian_trajectory", 10);

  // 初期位置を原点として設定
  current_pos_.x = 0.0;
  current_pos_.y = 0.0;
  current_pos_.z = 0.0;

  RCLCPP_INFO(get_logger(), "trajectory_generator_node started");
}

void TrajectoryGeneratorNode::onGoalPose(
  const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  const auto traj = generateTrajectory(current_pos_, msg->pose.position);
  trajectory_pub_->publish(traj);

  RCLCPP_INFO(get_logger(), "trajectory published: %zu waypoints → (%.3f, %.3f)",
    traj.poses.size(), msg->pose.position.x, msg->pose.position.y);
}

nav_msgs::msg::Path TrajectoryGeneratorNode::generateTrajectory(
  const geometry_msgs::msg::Point & start,
  const geometry_msgs::msg::Point & goal) const
{
  const auto waypoints = TrajectoryUtils::interpolateLinear(start, goal, num_waypoints_);
  const auto timestamps = TrajectoryUtils::trapezoidalTimeStamps(waypoints, v_max_, a_max_);

  nav_msgs::msg::Path path;
  path.header.stamp    = now();
  path.header.frame_id = "world";

  for (size_t i = 0; i < waypoints.size(); ++i) {
    geometry_msgs::msg::PoseStamped ps;
    // header.stamp に到達時刻を格納
    const auto t_ns = static_cast<int64_t>(timestamps[i] * 1e9);
    ps.header.stamp = rclcpp::Time(t_ns);
    ps.header.frame_id = "world";
    ps.pose.position   = waypoints[i];
    path.poses.push_back(ps);
  }

  return path;
}

}  // namespace sharmech_core

RCLCPP_COMPONENTS_REGISTER_NODE(sharmech_core::TrajectoryGeneratorNode)
