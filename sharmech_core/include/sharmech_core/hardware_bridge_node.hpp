#ifndef SHARMECH_CORE__HARDWARE_BRIDGE_NODE_HPP_
#define SHARMECH_CORE__HARDWARE_BRIDGE_NODE_HPP_

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/bool.hpp>

#include <netinet/in.h>

namespace sharmech_core
{

// MCU通信ノード (UDP)
//
// 5節リンクIK・Z軸・テーブルヨー軸・手首差動機構の逆運動学はMCU側で行うため、
// 本ノードはエンドエフェクタの目標姿勢 (x, y, z, pitch, yaw) とグリッパ指令のみを
// UDPパケットとしてMCUへ送信する (各モータへの変換はMCUファームウェアの責務)。
//
// Sub: /cartesian_trajectory (trajectory_generator_node から)
// Sub: /gripper/command      (state_manager_node から)
// Pub: /robot/current_pose   (trajectory_generator_node, state_manager_node へ)
//
// 制御タイマー (control_rate Hz) で軌道を順番に送信する
class HardwareBridgeNode : public rclcpp::Node
{
public:
  explicit HardwareBridgeNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~HardwareBridgeNode();

private:
  void onCartesianTrajectory(const nav_msgs::msg::Path::SharedPtr msg);
  void onGripperCommand(const std_msgs::msg::Bool::SharedPtr msg);
  void controlLoop();

  void openUdpSocket();
  void sendTargetPosePacket(const geometry_msgs::msg::Pose & pose, bool gripper_grasp);

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr           traj_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr           gripper_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr  current_pose_pub_;
  rclcpp::TimerBase::SharedPtr                                   control_timer_;

  // 実行中の軌道
  nav_msgs::msg::Path current_traj_;
  size_t traj_index_{0};
  bool   is_executing_{false};

  // 直近の指令姿勢・グリッパ状態
  // MCUからの実フィードバックが無いため、送信済みの指令値を暫定的に現在姿勢として扱う
  // TODO: MCUからのUDP応答（エンコーダ由来の実姿勢）を受信して置き換える
  geometry_msgs::msg::Pose current_pose_{};
  bool gripper_grasp_{false};

  double control_rate_;  // [Hz]

  // UDP送信先 (MCU)
  int sockfd_{-1};
  sockaddr_in mcu_addr_{};
  std::string mcu_ip_;
  int mcu_port_;
  uint32_t packet_seq_{0};
};

}  // namespace sharmech_core

#endif  // SHARMECH_CORE__HARDWARE_BRIDGE_NODE_HPP_
