#ifndef SHARMECH_CORE__VR_INTERFACE_NODE_HPP_
#define SHARMECH_CORE__VR_INTERFACE_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "sharmech_core/utility/coordinate_converter.hpp"

namespace sharmech_core
{

// Unity (Meta Quest 3) ↔ ROS2 インターフェースノード
//
// [Unity → ROS2]
//   Sub: /vr/target_pose  (Unity座標系 PoseStamped)
//   Pub: /target_pose     (ROS2座標系 PoseStamped, 変換・検証済み)
//
// [ROS2 → Unity]
//   Sub: /robot/current_pose (ROS2座標系 PoseStamped)
//   Pub: /vr/current_pose    (Unity座標系 PoseStamped)
class VrInterfaceNode : public rclcpp::Node
{
public:
  explicit VrInterfaceNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void onVrTargetPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void onRobotCurrentPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  // 入力値が作業領域内か確認
  bool isValidTarget(const geometry_msgs::msg::PoseStamped & ros_pose) const;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr vr_target_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr robot_pose_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr    target_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr    vr_current_pub_;

  // 作業領域制限 [m]
  double workspace_x_min_, workspace_x_max_;
  double workspace_y_min_, workspace_y_max_;
  double workspace_z_min_, workspace_z_max_;
};

}  // namespace sharmech_core

#endif  // SHARMECH_CORE__VR_INTERFACE_NODE_HPP_
