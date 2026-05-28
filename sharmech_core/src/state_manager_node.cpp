#include "sharmech_core/state_manager_node.hpp"
#include <rclcpp_components/register_node_macro.hpp>

namespace sharmech_core
{

StateManagerNode::StateManagerNode(const rclcpp::NodeOptions & options)
: Node("state_manager_node", options)
{
  kinematics_params_.l1         = declare_parameter("l1", 0.15);
  kinematics_params_.l2         = declare_parameter("l2", 0.15);
  kinematics_params_.base_width = declare_parameter("base_width", 0.10);
  joint_tolerance_              = declare_parameter("joint_tolerance", 0.05);

  target_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "/target_pose", 10,
    std::bind(&StateManagerNode::onTargetPose, this, std::placeholders::_1));

  joint_states_sub_ = create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", 10,
    std::bind(&StateManagerNode::onJointStates, this, std::placeholders::_1));

  joint_traj_sub_ = create_subscription<trajectory_msgs::msg::JointTrajectory>(
    "/joint_trajectory", 10,
    std::bind(&StateManagerNode::onJointTrajectory, this, std::placeholders::_1));

  goal_pose_pub_    = create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);
  current_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/robot/current_pose", 10);

  cancel_srv_ = create_service<std_srvs::srv::Trigger>(
    "/task/cancel",
    std::bind(&StateManagerNode::onTaskCancel, this,
      std::placeholders::_1, std::placeholders::_2));

  // 20Hz で状態更新・現在位置配信
  state_timer_ = create_wall_timer(
    std::chrono::milliseconds(50),
    std::bind(&StateManagerNode::updateState, this));

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

  RCLCPP_INFO(get_logger(), "IDLE → PLANNING (goal: %.3f, %.3f)",
    msg->pose.position.x, msg->pose.position.y);

  state_ = TaskState::PLANNING;
  has_goal_joints_ = false;
  goal_pose_pub_->publish(*msg);
}

void StateManagerNode::onJointStates(
  const sensor_msgs::msg::JointState::SharedPtr msg)
{
  if (msg->position.size() < 2) {return;}

  current_theta1_ = msg->position[0];
  current_theta2_ = msg->position[1];

  current_pos_ = FiveBarKinematics::forwardKinematics(
    {current_theta1_, current_theta2_}, kinematics_params_);

  publishCurrentPose();
}

void StateManagerNode::onJointTrajectory(
  const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
{
  if (msg->points.empty()) {return;}

  // 最終ウェイポイントを完了判定に使用
  const auto & last = msg->points.back();
  if (last.positions.size() >= 2) {
    goal_theta1_     = last.positions[0];
    goal_theta2_     = last.positions[1];
    has_goal_joints_ = true;
  }

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
  state_           = TaskState::IDLE;
  has_goal_joints_ = false;
  response->success = true;
  response->message = "cancelled";
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
      state_           = TaskState::IDLE;
      has_goal_joints_ = false;
      break;

    default:
      break;
  }
}

void StateManagerNode::publishCurrentPose()
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.stamp    = now();
  pose.header.frame_id = "world";
  pose.pose.position.x = current_pos_.x;
  pose.pose.position.y = current_pos_.y;
  pose.pose.position.z = 0.0;
  current_pose_pub_->publish(pose);
}

bool StateManagerNode::isExecutionComplete() const
{
  if (!has_goal_joints_) {return false;}

  return std::abs(current_theta1_ - goal_theta1_) < joint_tolerance_ &&
         std::abs(current_theta2_ - goal_theta2_) < joint_tolerance_;
}

}  // namespace sharmech_core

RCLCPP_COMPONENTS_REGISTER_NODE(sharmech_core::StateManagerNode)
