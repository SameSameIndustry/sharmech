#include "sharmech_core/hardware_bridge_node.hpp"
#include "sharmech_core/utility/orientation_utils.hpp"
#include "sharmech_core/utility/udp_protocol.hpp"

#include <rclcpp_components/register_node_macro.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <cerrno>
#include <cstring>
#include <arpa/inet.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <unistd.h>

namespace sharmech_core
{

HardwareBridgeNode::HardwareBridgeNode(const rclcpp::NodeOptions & options)
: Node("hardware_bridge_node", options)
{
  command_mode_       = declare_parameter("command_mode", std::string("cartesian"));
  mcu_ip_             = declare_parameter("mcu_ip", std::string("192.168.1.50"));
  mcu_port_           = declare_parameter("mcu_port", 8888);
  local_port_         = declare_parameter("local_port", 8889);
  feedback_poll_rate_ = declare_parameter("feedback_poll_rate", 200.0);
  feedback_timeout_   = declare_parameter("feedback_timeout", 0.5);
  joint_names_        = declare_parameter("joint_names", std::vector<std::string>{});

  if (!openUdpSocket()) {
    // ソケットが開けなくてもノード自体は起動させる (送信時に警告が出る)
    RCLCPP_ERROR(get_logger(), "UDP socket unavailable; commands will not reach MCU");
  }

  if (command_mode_ == "cartesian") {
    cartesian_sub_ = create_subscription<sharmech_msgs::msg::CartesianCommand>(
      "/catchrobo/command/cartesian", 10,
      std::bind(&HardwareBridgeNode::onCartesianCommand, this, std::placeholders::_1));
  } else if (command_mode_ == "joint") {
    // パターンB。/catchrobo/command/joint の型が未確定のため未実装
    RCLCPP_FATAL(get_logger(),
      "command_mode 'joint' (pattern B) is not implemented yet");
    throw std::invalid_argument("command_mode 'joint' not implemented");
  } else {
    RCLCPP_FATAL(get_logger(), "Unknown command_mode: %s", command_mode_.c_str());
    throw std::invalid_argument("unknown command_mode");
  }

  gripper_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/catchrobo/command/gripper", 10,
    std::bind(&HardwareBridgeNode::onGripperCommand, this, std::placeholders::_1));
  orient_vertical_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/catchrobo/command/orient_vertical", 10,
    std::bind(&HardwareBridgeNode::onOrientVerticalCommand, this, std::placeholders::_1));

  current_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
    "/catchrobo/arm/current_pose", 10);
  joint_states_pub_ = create_publisher<sensor_msgs::msg::JointState>(
    "/joint_states", 10);

  const auto poll_period =
    std::chrono::duration<double>(1.0 / feedback_poll_rate_);
  feedback_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(poll_period),
    std::bind(&HardwareBridgeNode::onFeedbackTimer, this));

  RCLCPP_INFO(get_logger(),
    "hardware_bridge_node started (mode=%s) → udp %s:%d (recv :%d)",
    command_mode_.c_str(), mcu_ip_.c_str(), mcu_port_, local_port_);
}

HardwareBridgeNode::~HardwareBridgeNode()
{
  if (sockfd_ >= 0) {
    ::close(sockfd_);
  }
}

bool HardwareBridgeNode::openUdpSocket()
{
  sockfd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (sockfd_ < 0) {
    RCLCPP_ERROR(get_logger(), "socket() failed: %s", std::strerror(errno));
    return false;
  }

  // 受信用に local_port へ bind する (送信と同じソケットを共用)
  sockaddr_in local_addr{};
  local_addr.sin_family      = AF_INET;
  local_addr.sin_addr.s_addr = htonl(INADDR_ANY);
  local_addr.sin_port        = htons(static_cast<uint16_t>(local_port_));
  if (::bind(sockfd_, reinterpret_cast<const sockaddr *>(&local_addr),
    sizeof(local_addr)) < 0)
  {
    RCLCPP_ERROR(get_logger(), "bind(:%d) failed: %s",
      local_port_, std::strerror(errno));
    ::close(sockfd_);
    sockfd_ = -1;
    return false;
  }

  std::memset(&mcu_addr_, 0, sizeof(mcu_addr_));
  mcu_addr_.sin_family = AF_INET;
  mcu_addr_.sin_port   = htons(static_cast<uint16_t>(mcu_port_));
  if (::inet_pton(AF_INET, mcu_ip_.c_str(), &mcu_addr_.sin_addr) != 1) {
    RCLCPP_ERROR(get_logger(), "Invalid mcu_ip parameter: %s", mcu_ip_.c_str());
    ::close(sockfd_);
    sockfd_ = -1;
    return false;
  }
  return true;
}

void HardwareBridgeNode::onCartesianCommand(
  const sharmech_msgs::msg::CartesianCommand::SharedPtr msg)
{
  if (sockfd_ < 0) {return;}

  // 手首は pitch/yaw の2自由度のみ。roll 成分は捨てる
  const auto pitch_yaw = OrientationUtils::toPitchYaw(msg->pose.orientation);

  const uint64_t timestamp_us =
    static_cast<uint64_t>(now().nanoseconds() / 1000);

  const auto packet = UdpProtocol::encodeCartesian(
    static_cast<float>(msg->pose.position.x),
    static_cast<float>(msg->pose.position.y),
    static_cast<float>(msg->pose.position.z),
    static_cast<float>(pitch_yaw.pitch),
    static_cast<float>(pitch_yaw.yaw),
    static_cast<float>(msg->twist.linear.x),
    static_cast<float>(msg->twist.linear.y),
    static_cast<float>(msg->twist.linear.z),
    static_cast<float>(msg->twist.angular.y),   // pitch_rate
    static_cast<float>(msg->twist.angular.z),   // yaw_rate
    gripper_state_, orient_vertical_state_, send_seq_++, timestamp_us);

  const auto sent = ::sendto(
    sockfd_, packet.data(), packet.size(), 0,
    reinterpret_cast<const sockaddr *>(&mcu_addr_), sizeof(mcu_addr_));
  if (sent < 0) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
      "UDP send failed: %s", std::strerror(errno));
  }
}

void HardwareBridgeNode::onGripperCommand(const std_msgs::msg::Bool::SharedPtr msg)
{
  // グリッパは Cartesian 指令とは別トピックで届くためラッチしておき、
  // 次の Cartesian 指令のパケットに詰める
  gripper_state_ = msg->data;
}

void HardwareBridgeNode::onOrientVerticalCommand(const std_msgs::msg::Bool::SharedPtr msg)
{
  // グリッパと同様、別トピックで届くためラッチして次の Cartesian パケットに詰める
  orient_vertical_state_ = msg->data;
}

void HardwareBridgeNode::onFeedbackTimer()
{
  if (sockfd_ < 0) {return;}

  // ソケットに溜まっているデータグラムをすべて読み切り、最新のみ採用する
  std::optional<udp_protocol::Feedback> latest;
  uint8_t buffer[512];
  while (true) {
    const auto received =
      ::recvfrom(sockfd_, buffer, sizeof(buffer), MSG_DONTWAIT, nullptr, nullptr);
    if (received < 0) {break;}  // EAGAIN: 読み切った

    auto fb = UdpProtocol::decodeFeedback(buffer, static_cast<size_t>(received));
    if (!fb) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Invalid feedback packet discarded (%zd bytes)", received);
      continue;
    }
    // UDP は順序を保証しない。古い seq のパケットは破棄する
    if (last_recv_seq_ && fb->seq <= *last_recv_seq_) {continue;}
    last_recv_seq_ = fb->seq;
    latest = std::move(fb);
  }

  if (latest) {
    last_feedback_time_ = now();

    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp    = *last_feedback_time_;
    pose_msg.header.frame_id = "field";
    pose_msg.pose.position.x = latest->x;
    pose_msg.pose.position.y = latest->y;
    pose_msg.pose.position.z = latest->z;
    tf2::Quaternion q;
    q.setRPY(0.0, latest->pitch, latest->yaw);
    pose_msg.pose.orientation = tf2::toMsg(q);
    current_pose_pub_->publish(pose_msg);

    sensor_msgs::msg::JointState js;
    js.header.stamp = *last_feedback_time_;
    const size_t n = latest->joint_positions.size();
    if (joint_names_.size() == n) {
      js.name = joint_names_;
    } else {
      if (!warned_joint_names_) {
        RCLCPP_WARN(get_logger(),
          "joint_names size (%zu) != feedback joint_count (%zu); using joint_i",
          joint_names_.size(), n);
        warned_joint_names_ = true;
      }
      for (size_t i = 0; i < n; ++i) {
        js.name.push_back("joint_" + std::to_string(i));
      }
    }
    js.position.assign(
      latest->joint_positions.begin(), latest->joint_positions.end());
    joint_states_pub_->publish(js);

    if (latest->status_flags != 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "MCU status_flags = 0x%04x", latest->status_flags);
    }
  } else if (last_feedback_time_) {
    // 一度でも届いたことがあるのに途絶した場合のみ警告する。
    // MCU 側が未実装のうちからログを埋めないため
    const auto silence = (now() - *last_feedback_time_).seconds();
    if (silence > feedback_timeout_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "No MCU feedback for %.1f s", silence);
    }
  }
}

}  // namespace sharmech_core

RCLCPP_COMPONENTS_REGISTER_NODE(sharmech_core::HardwareBridgeNode)
