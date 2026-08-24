#include "sharmech_core/vr_interface_node.hpp"
#include <rclcpp_components/register_node_macro.hpp>

namespace sharmech_core
{

VrInterfaceNode::VrInterfaceNode(const rclcpp::NodeOptions & options)
: Node("vr_interface_node", options)
{
  workspace_x_min_ = declare_parameter("workspace_x_min", -0.20);
  workspace_x_max_ = declare_parameter("workspace_x_max",  0.20);
  workspace_y_min_ = declare_parameter("workspace_y_min",  0.05);
  workspace_y_max_ = declare_parameter("workspace_y_max",  0.30);
  workspace_z_min_ = declare_parameter("workspace_z_min",  0.00);
  workspace_z_max_ = declare_parameter("workspace_z_max",  0.30);

  vr_target_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "/vr/target_pose", 10,
    std::bind(&VrInterfaceNode::onVrTargetPose, this, std::placeholders::_1));

  robot_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "/robot/current_pose", 10,
    std::bind(&VrInterfaceNode::onRobotCurrentPose, this, std::placeholders::_1));

  target_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/target_pose", 10);
  vr_current_pub_  = create_publisher<geometry_msgs::msg::PoseStamped>("/vr/current_pose", 10);

  RCLCPP_INFO(get_logger(), "vr_interface_node started");
}

void VrInterfaceNode::onVrTargetPose(
  const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  // Unity座標系 → ROS2座標系に変換
  const auto ros_pose = CoordinateConverter::unityToRos(*msg);

  if (!isValidTarget(ros_pose)) {
    RCLCPP_WARN(get_logger(), "Target out of workspace: (%.3f, %.3f, %.3f)",
      ros_pose.pose.position.x, ros_pose.pose.position.y, ros_pose.pose.position.z);
    return;
  }

  target_pose_pub_->publish(ros_pose);
}

void VrInterfaceNode::onRobotCurrentPose(
  const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  // ROS2座標系 → Unity座標系に変換してVR側へ返す
  vr_current_pub_->publish(CoordinateConverter::rosToUnity(*msg));
}

bool VrInterfaceNode::isValidTarget(
  const geometry_msgs::msg::PoseStamped & ros_pose) const
{
  const double x = ros_pose.pose.position.x;
  const double y = ros_pose.pose.position.y;
  const double z = ros_pose.pose.position.z;
  return x >= workspace_x_min_ && x <= workspace_x_max_ &&
         y >= workspace_y_min_ && y <= workspace_y_max_ &&
         z >= workspace_z_min_ && z <= workspace_z_max_;
}

}  // namespace sharmech_core

RCLCPP_COMPONENTS_REGISTER_NODE(sharmech_core::VrInterfaceNode)
