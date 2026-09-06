#ifndef SHARMECH_CORE__HARDWARE_BRIDGE_NODE_HPP_
#define SHARMECH_CORE__HARDWARE_BRIDGE_NODE_HPP_

#include <optional>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sharmech_msgs/msg/cartesian_command.hpp>
#include <sharmech_msgs/msg/mcu_status.hpp>

#include <netinet/in.h>

namespace sharmech_core
{

// MCU 通信ノード (UDP)
//
// ROS2 トピックと UDP パケットの間の変換と輸送のみを担う。判断はしない。
// 補間は MCU、レート制限・クランプ・ウォッチドッグは motion_generator_node の責務。
//
// 仕様の正本: sharmech_core/docs/hardware_bridge_node.md
//
// Sub: /catchrobo/command/cartesian (command_mode == cartesian)
// Sub: /catchrobo/command/joint     (command_mode == joint。パターンB)
// Sub: /catchrobo/command/gripper
// Sub: /catchrobo/command/orient_vertical  (game_state_manager_node の PLACING 指示)
// Pub: /catchrobo/arm/current_pose  (MCU が FK して返した実姿勢)
// Pub: /joint_states                (実測の関節角)
// Pub: /catchrobo/arm/mcu_status    (MCU の status_flags と疎通状態)
class HardwareBridgeNode : public rclcpp::Node
{
public:
  explicit HardwareBridgeNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~HardwareBridgeNode() override;

private:
  // 送信: サブスクリプション駆動 (タイマーではない)。
  // 上流が止まれば送信も止まり、MCU 側ウォッチドッグが作動して安全側に倒れる
  void onCartesianCommand(const sharmech_msgs::msg::CartesianCommand::SharedPtr msg);
  void onJointCommand(const sensor_msgs::msg::JointState::SharedPtr msg);
  void onGripperCommand(const std_msgs::msg::Bool::SharedPtr msg);
  void onOrientVerticalCommand(const std_msgs::msg::Bool::SharedPtr msg);

  // 受信: タイマーでソケットに溜まったデータグラムを読み切る
  void onFeedbackTimer();

  bool openUdpSocket();
  void publishFeedback();
  // フィードバックの有無に関わらず定期的に疎通状態を配信する
  void publishMcuStatus(bool connected, double silence_sec);

  rclcpp::Subscription<sharmech_msgs::msg::CartesianCommand>::SharedPtr cartesian_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr gripper_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr orient_vertical_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub_;
  rclcpp::Publisher<sharmech_msgs::msg::McuStatus>::SharedPtr mcu_status_pub_;
  rclcpp::TimerBase::SharedPtr feedback_timer_;

  // パラメータ
  std::string command_mode_;
  std::string mcu_ip_;
  int mcu_port_;
  int local_port_;
  double feedback_poll_rate_;        // [Hz]
  double feedback_timeout_;          // [s]
  std::vector<std::string> joint_names_;

  // 内部状態
  int sockfd_{-1};
  sockaddr_in mcu_addr_{};
  bool gripper_state_{false};              // ラッチしたグリッパ状態
  bool orient_vertical_state_{false};         // ラッチした「縦にする」指示
  uint32_t send_seq_{0};
  std::optional<uint32_t> last_recv_seq_;           // 順序逆転の検出用
  std::optional<rclcpp::Time> last_feedback_time_;  // 途絶の検出用
  bool warned_joint_names_{false};
  bool warned_joint_cmd_names_{false};          // 0x02送信側の名前不一致の警告 (1回のみ)
  bool warned_joint_cmd_velocity_{false};         // 同、velocity欠落の警告 (1回のみ)
  uint16_t last_status_flags_{0};
  bool last_gripper_state_{false};
  uint32_t last_seq_echo_{0};
  uint32_t out_of_order_count_{0};
};

}  // namespace sharmech_core

#endif  // SHARMECH_CORE__HARDWARE_BRIDGE_NODE_HPP_
