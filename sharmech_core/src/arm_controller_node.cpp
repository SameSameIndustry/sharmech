#include "sharmech_core/arm_controller_node.hpp"

namespace sharmech_core
{

ArmControllerNode::ArmControllerNode(const rclcpp::NodeOptions & options)
: Node("arm_controller_node", options)
{
  joint_command_sub_ = create_subscription<sensor_msgs::msg::JointState>(
    "/joint_command", 10,
    std::bind(&ArmControllerNode::onJointCommand, this, std::placeholders::_1));

  joint_states_pub_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

  // 50Hz でエンコーダ読み取り & joint_states 配信
  timer_ = create_wall_timer(
    std::chrono::milliseconds(20),
    std::bind(&ArmControllerNode::publishJointStates, this));

  RCLCPP_INFO(get_logger(), "arm_controller_node started");
}

ArmControllerNode::~ArmControllerNode()
{
  // TODO: ハードウェア接続のクローズ処理
}

void ArmControllerNode::onJointCommand(
  const sensor_msgs::msg::JointState::SharedPtr msg)
{
  if (msg->position.size() < 2) {
    RCLCPP_WARN(get_logger(), "joint_command: position size < 2");
    return;
  }
  command_theta1_ = msg->position[0];
  command_theta2_ = msg->position[1];
  sendCommandToHardware(command_theta1_, command_theta2_);
}

void ArmControllerNode::publishJointStates()
{
  readFromHardware(current_theta1_, current_theta2_);

  sensor_msgs::msg::JointState state;
  state.header.stamp = now();
  state.name         = {"joint1", "joint2"};
  state.position     = {current_theta1_, current_theta2_};
  joint_states_pub_->publish(state);
}

void ArmControllerNode::sendCommandToHardware(double theta1, double theta2)
{
  // TODO: シリアル/CAN/その他インターフェースで指令を送信
  // 例: serial_->write(buildPacket(theta1, theta2));
  RCLCPP_DEBUG(get_logger(), "cmd: θ1=%.3f θ2=%.3f", theta1, theta2);
}

void ArmControllerNode::readFromHardware(double & theta1, double & theta2)
{
  // TODO: ハードウェアからエンコーダ値を読み取り角度に変換
  // 例: serial_->read() → デコード → theta1, theta2
  // 暫定: 指令値をそのまま返す (フィードバックなし)
  theta1 = command_theta1_;
  theta2 = command_theta2_;
}

}  // namespace sharmech_core

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<sharmech_core::ArmControllerNode>());
  rclcpp::shutdown();
  return 0;
}
