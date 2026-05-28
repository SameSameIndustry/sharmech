#include "sharmech_core/hardware_bridge_node.hpp"
#include <rclcpp_components/register_node_macro.hpp>

namespace sharmech_core
{

HardwareBridgeNode::HardwareBridgeNode(const rclcpp::NodeOptions & options)
: Node("hardware_bridge_node", options)
{
  control_rate_ = declare_parameter("control_rate", 100.0);

  traj_sub_ = create_subscription<trajectory_msgs::msg::JointTrajectory>(
    "/joint_trajectory", 10,
    std::bind(&HardwareBridgeNode::onJointTrajectory, this, std::placeholders::_1));

  gripper_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/gripper/command", 10,
    std::bind(&HardwareBridgeNode::onGripperCommand, this, std::placeholders::_1));

  joint_states_pub_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

  const auto period_ms = static_cast<int>(1000.0 / control_rate_);
  control_timer_ = create_wall_timer(
    std::chrono::milliseconds(period_ms),
    std::bind(&HardwareBridgeNode::controlLoop, this));

  RCLCPP_INFO(get_logger(), "hardware_bridge_node started (%.0f Hz)", control_rate_);
}

HardwareBridgeNode::~HardwareBridgeNode()
{
  // TODO: ハードウェア接続のクローズ処理
}

void HardwareBridgeNode::onJointTrajectory(
  const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
{
  current_traj_ = *msg;
  traj_index_   = 0;
  is_executing_ = !current_traj_.points.empty();

  RCLCPP_INFO(get_logger(), "New trajectory received: %zu points",
    current_traj_.points.size());
}

void HardwareBridgeNode::onGripperCommand(const std_msgs::msg::Bool::SharedPtr msg)
{
  sendGripperCommand(msg->data);
}

void HardwareBridgeNode::controlLoop()
{
  // エンコーダ読み取り
  readEncoders(current_theta1_, current_theta2_);

  // 軌道の現在ウェイポイントを送信
  if (is_executing_ && traj_index_ < current_traj_.points.size()) {
    const auto & pt = current_traj_.points[traj_index_];
    if (pt.positions.size() >= 2) {
      sendJointCommand(pt.positions[0], pt.positions[1]);
    }
    ++traj_index_;

    if (traj_index_ >= current_traj_.points.size()) {
      is_executing_ = false;
      RCLCPP_INFO(get_logger(), "Trajectory execution complete");
    }
  }

  // /joint_states 配信
  sensor_msgs::msg::JointState state;
  state.header.stamp = now();
  state.name         = {"joint1", "joint2"};
  state.position     = {current_theta1_, current_theta2_};
  joint_states_pub_->publish(state);
}

void HardwareBridgeNode::sendJointCommand(double theta1, double theta2)
{
  // TODO: シリアル / CAN でモータへ角度指令を送信
  // 例: serial_->write(buildPacket(theta1, theta2));
  RCLCPP_DEBUG(get_logger(), "cmd: θ1=%.4f θ2=%.4f", theta1, theta2);
}

void HardwareBridgeNode::sendGripperCommand(bool grasp)
{
  // TODO: グリッパハードウェアへ指令を送信
  RCLCPP_INFO(get_logger(), "gripper: %s", grasp ? "grasp" : "release");
}

void HardwareBridgeNode::readEncoders(double & theta1, double & theta2)
{
  // TODO: ハードウェアからエンコーダ値を読み取り角度に変換
  // 暫定: 指令値を保持 (フィードバックなし)
  theta1 = current_theta1_;
  theta2 = current_theta2_;
}

}  // namespace sharmech_core

RCLCPP_COMPONENTS_REGISTER_NODE(sharmech_core::HardwareBridgeNode)
