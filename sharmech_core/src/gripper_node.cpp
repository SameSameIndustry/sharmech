#include "sharmech_core/gripper_node.hpp"

namespace sharmech_core
{

GripperNode::GripperNode(const rclcpp::NodeOptions & options)
: Node("gripper_node", options)
{
  gripper_service_ = create_service<std_srvs::srv::SetBool>(
    "/gripper/command",
    std::bind(
      &GripperNode::onGripperCommand, this,
      std::placeholders::_1, std::placeholders::_2));

  gripper_state_pub_ = create_publisher<std_msgs::msg::Bool>("/gripper/state", 10);

  RCLCPP_INFO(get_logger(), "gripper_node started");
}

void GripperNode::onGripperCommand(
  const std_srvs::srv::SetBool::Request::SharedPtr request,
  std_srvs::srv::SetBool::Response::SharedPtr response)
{
  if (request->data) {
    grasp();
    response->message = "grasped";
  } else {
    release();
    response->message = "released";
  }
  response->success = true;

  std_msgs::msg::Bool state_msg;
  state_msg.data = is_grasping_;
  gripper_state_pub_->publish(state_msg);
}

void GripperNode::grasp()
{
  // TODO: グリッパハードウェアへ把持指令を送信
  is_grasping_ = true;
  RCLCPP_INFO(get_logger(), "grasp");
}

void GripperNode::release()
{
  // TODO: グリッパハードウェアへ開放指令を送信
  is_grasping_ = false;
  RCLCPP_INFO(get_logger(), "release");
}

}  // namespace sharmech_core

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<sharmech_core::GripperNode>());
  rclcpp::shutdown();
  return 0;
}
