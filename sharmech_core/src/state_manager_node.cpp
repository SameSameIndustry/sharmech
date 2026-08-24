#include "sharmech_core/state_manager_node.hpp"
#include "sharmech_core/utility/orientation_utils.hpp"
#include <rclcpp_components/register_node_macro.hpp>

#include <cmath>

namespace sharmech_core
{

StateManagerNode::StateManagerNode(const rclcpp::NodeOptions & options)
: Node("state_manager_node", options)
{
  pos_tolerance_ = declare_parameter("pos_tolerance", 0.005);
  rot_tolerance_ = declare_parameter("rot_tolerance", 0.05);

  target_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "/target_pose", 10,
    std::bind(&StateManagerNode::onTargetPose, this, std::placeholders::_1));

  current_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "/robot/current_pose", 10,
    std::bind(&StateManagerNode::onCurrentPose, this, std::placeholders::_1));

  cartesian_traj_sub_ = create_subscription<nav_msgs::msg::Path>(
    "/cartesian_trajectory", 10,
    std::bind(&StateManagerNode::onCartesianTrajectory, this, std::placeholders::_1));

  goal_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);

  cancel_srv_ = create_service<std_srvs::srv::Trigger>(
    "/task/cancel",
    std::bind(&StateManagerNode::onTaskCancel, this,
      std::placeholders::_1, std::placeholders::_2));

  // 20Hz で状態更新
  state_timer_ = create_wall_timer(
    std::chrono::milliseconds(50),
    std::bind(&StateManagerNode::updateState, this));

  current_pose_.orientation.w = 1.0;

  RCLCPP_INFO(get_logger(), "state_manager_node started");
}

void StateManagerNode::onTargetPose(
  const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  if (state_ != TaskState::IDLE) {
    RCLCPP_WARN(get_logger(), "Ignored target_pose: not IDLE (state=%d)",
      static_cast<int>(state_));
    return;
  }

  RCLCPP_INFO(get_logger(), "IDLE → PLANNING (goal: %.3f, %.3f, %.3f)",
    msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);

  state_           = TaskState::PLANNING;
  has_goal_pose_   = false;
  goal_pose_pub_->publish(*msg);
}

void StateManagerNode::onCurrentPose(
  const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  current_pose_ = msg->pose;
}

void StateManagerNode::onCartesianTrajectory(
  const nav_msgs::msg::Path::SharedPtr msg)
{
  if (msg->poses.empty()) {return;}

  goal_pose_     = msg->poses.back().pose;
  has_goal_pose_ = true;

  if (state_ == TaskState::PLANNING) {
    RCLCPP_INFO(get_logger(), "PLANNING → EXECUTING");
    state_ = TaskState::EXECUTING;
  }
}

void StateManagerNode::onTaskCancel(
  const std_srvs::srv::Trigger::Request::SharedPtr,
  std_srvs::srv::Trigger::Response::SharedPtr response)
{
  RCLCPP_WARN(get_logger(), "Task cancelled (state=%d)", static_cast<int>(state_));
  state_          = TaskState::IDLE;
  has_goal_pose_  = false;
  response->success = true;
  response->message  = "cancelled";
}

void StateManagerNode::updateState()
{
  switch (state_) {
    case TaskState::EXECUTING:
      if (isExecutionComplete()) {
        RCLCPP_INFO(get_logger(), "EXECUTING → DONE");
        state_ = TaskState::DONE;
      }
      break;

    case TaskState::DONE:
      RCLCPP_INFO(get_logger(), "DONE → IDLE");
      state_         = TaskState::IDLE;
      has_goal_pose_ = false;
      break;

    default:
      break;
  }
}

bool StateManagerNode::isExecutionComplete() const
{
  if (!has_goal_pose_) {return false;}

  const auto & cp = current_pose_.position;
  const auto & gp = goal_pose_.position;
  const double pos_err = std::hypot(cp.x - gp.x, cp.y - gp.y, cp.z - gp.z);
  if (pos_err > pos_tolerance_) {return false;}

  const auto current_orientation = OrientationUtils::toPitchYaw(current_pose_.orientation);
  const auto goal_orientation    = OrientationUtils::toPitchYaw(goal_pose_.orientation);

  return std::abs(current_orientation.pitch - goal_orientation.pitch) < rot_tolerance_ &&
         std::abs(current_orientation.yaw - goal_orientation.yaw) < rot_tolerance_;
}

}  // namespace sharmech_core

RCLCPP_COMPONENTS_REGISTER_NODE(sharmech_core::StateManagerNode)
