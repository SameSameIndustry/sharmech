#ifndef SHARMECH_CORE__GRIPPER_NODE_HPP_
#define SHARMECH_CORE__GRIPPER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_msgs/msg/bool.hpp>

namespace sharmech_core
{

// グリッパ(エンドエフェクタ)制御ノード
//
// Service: /gripper/command (std_srvs/SetBool)
//   request.data = true  → 把持
//   request.data = false → 開放
// Publish: /gripper/state (std_msgs/Bool) - 現在の把持状態
class GripperNode : public rclcpp::Node
{
public:
  explicit GripperNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void onGripperCommand(
    const std_srvs::srv::SetBool::Request::SharedPtr request,
    std_srvs::srv::SetBool::Response::SharedPtr response);

  void grasp();
  void release();

  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr gripper_service_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr gripper_state_pub_;

  bool is_grasping_{false};
};

}  // namespace sharmech_core

#endif  // SHARMECH_CORE__GRIPPER_NODE_HPP_
