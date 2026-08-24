#include "sharmech_core/hardware_bridge_node.hpp"
#include "sharmech_core/utility/orientation_utils.hpp"
#include <rclcpp_components/register_node_macro.hpp>

#include <cerrno>
#include <cstring>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

namespace sharmech_core
{

namespace
{
// MCUへ送信するエンドエフェクタ目標姿勢パケット
// MCU側でIK (5節リンク・Z軸・テーブルヨー軸・手首差動機構) を行い、各モータへ変換する
#pragma pack(push, 1)
struct TargetPosePacket
{
  uint32_t seq;      // シーケンス番号
  double x;          // [m]
  double y;          // [m]
  double z;          // [m]
  double pitch;      // [rad]
  double yaw;        // [rad]
  uint8_t gripper;   // 0=release, 1=grasp
};
#pragma pack(pop)
}  // namespace

HardwareBridgeNode::HardwareBridgeNode(const rclcpp::NodeOptions & options)
: Node("hardware_bridge_node", options)
{
  control_rate_ = declare_parameter("control_rate", 100.0);
  mcu_ip_       = declare_parameter("mcu_ip", std::string("192.168.1.50"));
  mcu_port_     = declare_parameter("mcu_port", 8888);

  traj_sub_ = create_subscription<nav_msgs::msg::Path>(
    "/cartesian_trajectory", 10,
    std::bind(&HardwareBridgeNode::onCartesianTrajectory, this, std::placeholders::_1));

  gripper_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/gripper/command", 10,
    std::bind(&HardwareBridgeNode::onGripperCommand, this, std::placeholders::_1));

  current_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
    "/robot/current_pose", 10);

  current_pose_.orientation.w = 1.0;

  openUdpSocket();

  const auto period_ms = static_cast<int>(1000.0 / control_rate_);
  control_timer_ = create_wall_timer(
    std::chrono::milliseconds(period_ms),
    std::bind(&HardwareBridgeNode::controlLoop, this));

  RCLCPP_INFO(get_logger(), "hardware_bridge_node started (%.0f Hz) → udp %s:%d",
    control_rate_, mcu_ip_.c_str(), mcu_port_);
}

HardwareBridgeNode::~HardwareBridgeNode()
{
  if (sockfd_ >= 0) {
    ::close(sockfd_);
  }
}

void HardwareBridgeNode::openUdpSocket()
{
  sockfd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (sockfd_ < 0) {
    RCLCPP_ERROR(get_logger(), "Failed to open UDP socket: %s", std::strerror(errno));
    return;
  }

  std::memset(&mcu_addr_, 0, sizeof(mcu_addr_));
  mcu_addr_.sin_family = AF_INET;
  mcu_addr_.sin_port   = htons(static_cast<uint16_t>(mcu_port_));

  if (::inet_pton(AF_INET, mcu_ip_.c_str(), &mcu_addr_.sin_addr) != 1) {
    RCLCPP_ERROR(get_logger(), "Invalid mcu_ip parameter: %s", mcu_ip_.c_str());
    ::close(sockfd_);
    sockfd_ = -1;
  }
}

void HardwareBridgeNode::onCartesianTrajectory(const nav_msgs::msg::Path::SharedPtr msg)
{
  current_traj_ = *msg;
  traj_index_   = 0;
  is_executing_ = !current_traj_.poses.empty();

  RCLCPP_INFO(get_logger(), "New trajectory received: %zu waypoints",
    current_traj_.poses.size());
}

void HardwareBridgeNode::onGripperCommand(const std_msgs::msg::Bool::SharedPtr msg)
{
  gripper_grasp_ = msg->data;
}

void HardwareBridgeNode::controlLoop()
{
  // 軌道の現在ウェイポイントを送信
  if (is_executing_ && traj_index_ < current_traj_.poses.size()) {
    current_pose_ = current_traj_.poses[traj_index_].pose;
    ++traj_index_;

    if (traj_index_ >= current_traj_.poses.size()) {
      is_executing_ = false;
      RCLCPP_INFO(get_logger(), "Trajectory execution complete");
    }
  }

  sendTargetPosePacket(current_pose_, gripper_grasp_);

  // MCUからの実フィードバックが無いため、送信した目標姿勢を現在姿勢として暫定配信する
  geometry_msgs::msg::PoseStamped pose_msg;
  pose_msg.header.stamp    = now();
  pose_msg.header.frame_id = "world";
  pose_msg.pose            = current_pose_;
  current_pose_pub_->publish(pose_msg);
}

void HardwareBridgeNode::sendTargetPosePacket(
  const geometry_msgs::msg::Pose & pose, bool gripper_grasp)
{
  if (sockfd_ < 0) {return;}

  // 手首は差動2モータでpitch/yawを作る機構だが、ROS2側はEuler角(pitch, yaw)を
  // 送信するのみとし、差動へのミキシングはMCUファームウェアの責務とする
  const auto pitch_yaw = OrientationUtils::toPitchYaw(pose.orientation);

  TargetPosePacket packet{};
  packet.seq     = packet_seq_++;
  packet.x       = pose.position.x;
  packet.y       = pose.position.y;
  packet.z       = pose.position.z;
  packet.pitch   = pitch_yaw.pitch;
  packet.yaw     = pitch_yaw.yaw;
  packet.gripper = gripper_grasp ? 1 : 0;

  const auto sent = ::sendto(
    sockfd_, &packet, sizeof(packet), 0,
    reinterpret_cast<const sockaddr *>(&mcu_addr_), sizeof(mcu_addr_));

  if (sent < 0) {
    RCLCPP_WARN(get_logger(), "UDP send failed: %s", std::strerror(errno));
  }
}

}  // namespace sharmech_core

RCLCPP_COMPONENTS_REGISTER_NODE(sharmech_core::HardwareBridgeNode)
