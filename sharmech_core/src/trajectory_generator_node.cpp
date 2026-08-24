#include "sharmech_core/trajectory_generator_node.hpp"
#include <rclcpp_components/register_node_macro.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

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
      current_pose_ = msg->pose;
    });

  trajectory_pub_ = create_publisher<nav_msgs::msg::Path>("/cartesian_trajectory", 10);

  // 初期姿勢を原点・無回転として設定
  current_pose_.position.x = 0.0;
  current_pose_.position.y = 0.0;
  current_pose_.position.z = 0.0;
  current_pose_.orientation.w = 1.0;

  RCLCPP_INFO(get_logger(), "trajectory_generator_node started");
}

void TrajectoryGeneratorNode::onGoalPose(
  const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  const auto traj = generateTrajectory(current_pose_, msg->pose);
  trajectory_pub_->publish(traj);

  RCLCPP_INFO(get_logger(), "trajectory published: %zu waypoints → (%.3f, %.3f, %.3f)",
    traj.poses.size(), msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
}

nav_msgs::msg::Path TrajectoryGeneratorNode::generateTrajectory(
  const geometry_msgs::msg::Pose & start,
  const geometry_msgs::msg::Pose & goal) const
{
  const auto waypoints = TrajectoryUtils::interpolateLinear(
    start.position, goal.position, num_waypoints_);
  const auto timestamps = TrajectoryUtils::trapezoidalTimeStamps(waypoints, v_max_, a_max_);

  tf2::Quaternion q_start, q_goal;
  tf2::fromMsg(start.orientation, q_start);
  tf2::fromMsg(goal.orientation, q_goal);

  nav_msgs::msg::Path path;
  path.header.stamp    = now();
  path.header.frame_id = "world";

  for (size_t i = 0; i < waypoints.size(); ++i) {
    const double t = static_cast<double>(i) / (waypoints.size() - 1);

    geometry_msgs::msg::PoseStamped ps;
    // header.stamp に到達時刻を格納
    const auto t_ns = static_cast<int64_t>(timestamps[i] * 1e9);
    ps.header.stamp    = rclcpp::Time(t_ns);
    ps.header.frame_id = "world";
    ps.pose.position    = waypoints[i];
    ps.pose.orientation = tf2::toMsg(q_start.slerp(q_goal, t));
    path.poses.push_back(ps);
  }

  return path;
}

}  // namespace sharmech_core

RCLCPP_COMPONENTS_REGISTER_NODE(sharmech_core::TrajectoryGeneratorNode)
