#include "sharmech_core/task_manager_node.hpp"

namespace sharmech_core
{

TaskManagerNode::TaskManagerNode(const rclcpp::NodeOptions & options)
: Node("task_manager_node", options)
{
  pick_x_  = declare_parameter("pick_x",  0.0);
  pick_y_  = declare_parameter("pick_y",  0.15);
  place_x_ = declare_parameter("place_x", 0.10);
  place_y_ = declare_parameter("place_y", 0.15);
  home_x_  = declare_parameter("home_x",  0.0);
  home_y_  = declare_parameter("home_y",  0.20);
  position_tolerance_ = declare_parameter("position_tolerance", 0.005);

  task_start_srv_ = create_service<std_srvs::srv::Trigger>(
    "/task/start",
    std::bind(&TaskManagerNode::onTaskStart, this,
      std::placeholders::_1, std::placeholders::_2));

  task_abort_srv_ = create_service<std_srvs::srv::Trigger>(
    "/task/abort",
    std::bind(&TaskManagerNode::onTaskAbort, this,
      std::placeholders::_1, std::placeholders::_2));

  joint_states_sub_ = create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", 10,
    std::bind(&TaskManagerNode::onJointStates, this, std::placeholders::_1));

  target_pose_pub_ = create_publisher<geometry_msgs::msg::Point>("/target_pose", 10);

  gripper_client_ = create_client<std_srvs::srv::SetBool>("/gripper/command");

  // 20Hz で状態機械を更新
  state_machine_timer_ = create_wall_timer(
    std::chrono::milliseconds(50),
    std::bind(&TaskManagerNode::updateStateMachine, this));

  RCLCPP_INFO(get_logger(), "task_manager_node started");
}

void TaskManagerNode::onTaskStart(
  const std_srvs::srv::Trigger::Request::SharedPtr,
  std_srvs::srv::Trigger::Response::SharedPtr response)
{
  if (state_ != TaskState::IDLE) {
    response->success = false;
    response->message = "task already running";
    return;
  }
  state_ = TaskState::MOVE_TO_PICK;
  publishTargetPose(pick_x_, pick_y_);
  response->success = true;
  response->message = "task started";
  RCLCPP_INFO(get_logger(), "task started → MOVE_TO_PICK");
}

void TaskManagerNode::onTaskAbort(
  const std_srvs::srv::Trigger::Request::SharedPtr,
  std_srvs::srv::Trigger::Response::SharedPtr response)
{
  state_ = TaskState::ABORTED;
  commandGripper(false);
  response->success = true;
  response->message = "task aborted";
  RCLCPP_WARN(get_logger(), "task aborted");
}

void TaskManagerNode::onJointStates(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  // TODO: kinematics_node の順運動学でEE位置を計算して current_x_, current_y_ を更新
  // 暫定: joint_states は受け取るが位置計算は未実装
  (void)msg;
}

void TaskManagerNode::updateStateMachine()
{
  switch (state_) {
    case TaskState::IDLE:
    case TaskState::ABORTED:
      break;

    case TaskState::MOVE_TO_PICK:
      if (isArmAtTarget()) {
        state_ = TaskState::PICKING;
        commandGripper(true);
        RCLCPP_INFO(get_logger(), "arrived at pick → PICKING");
      }
      break;

    case TaskState::PICKING:
      // グリッパの把持完了を待つ (TODO: フィードバックがあれば使う)
      state_ = TaskState::MOVE_TO_PLACE;
      publishTargetPose(place_x_, place_y_);
      RCLCPP_INFO(get_logger(), "picked → MOVE_TO_PLACE");
      break;

    case TaskState::MOVE_TO_PLACE:
      if (isArmAtTarget()) {
        state_ = TaskState::PLACING;
        commandGripper(false);
        RCLCPP_INFO(get_logger(), "arrived at place → PLACING");
      }
      break;

    case TaskState::PLACING:
      state_ = TaskState::HOMING;
      publishTargetPose(home_x_, home_y_);
      RCLCPP_INFO(get_logger(), "placed → HOMING");
      break;

    case TaskState::HOMING:
      if (isArmAtTarget()) {
        state_ = TaskState::IDLE;
        RCLCPP_INFO(get_logger(), "homed → IDLE (task complete)");
      }
      break;
  }
}

void TaskManagerNode::publishTargetPose(double x, double y)
{
  geometry_msgs::msg::Point msg;
  msg.x = x;
  msg.y = y;
  msg.z = 0.0;
  target_pose_pub_->publish(msg);
}

void TaskManagerNode::commandGripper(bool grasp)
{
  if (!gripper_client_->service_is_ready()) {
    RCLCPP_WARN(get_logger(), "gripper service not ready");
    return;
  }
  auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
  req->data = grasp;
  gripper_client_->async_send_request(req);
}

bool TaskManagerNode::isArmAtTarget()
{
  // TODO: current_x_, current_y_ と目標位置を比較して到達判定
  // 暫定: 常に到達済みとして返す (実装後に修正)
  return true;
}

}  // namespace sharmech_core

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<sharmech_core::TaskManagerNode>());
  rclcpp::shutdown();
  return 0;
}
