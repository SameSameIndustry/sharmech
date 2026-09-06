#include "sharmech_core/hardware_bridge_node.hpp"
#include "sharmech_core/utility/orientation_utils.hpp"
#include "sharmech_core/utility/udp_protocol.hpp"

#include <rclcpp_components/register_node_macro.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <algorithm>
#include <array>
#include <cerrno>
#include <cstring>
#include <iterator>
#include <arpa/inet.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <unistd.h>

namespace sharmech_core
{

HardwareBridgeNode::HardwareBridgeNode(const rclcpp::NodeOptions & options)
: Node("hardware_bridge_node", options)
{
  command_mode_ = declare_parameter("command_mode", std::string("cartesian"));
  mcu_ip_ = declare_parameter("mcu_ip", std::string("192.168.1.50"));
  mcu_port_ = declare_parameter("mcu_port", 8888);
  local_port_ = declare_parameter("local_port", 8889);
  feedback_poll_rate_ = declare_parameter("feedback_poll_rate", 200.0);
  feedback_timeout_ = declare_parameter("feedback_timeout", 0.5);
  joint_names_ = declare_parameter("joint_names", std::vector<std::string>{});

  if (!openUdpSocket()) {
    // ソケットが開けなくてもノード自体は起動させる (送信時に警告が出る)
    RCLCPP_ERROR(get_logger(), "UDP socket unavailable; commands will not reach MCU");
  }

  if (command_mode_ == "cartesian") {
    cartesian_sub_ = create_subscription<sharmech_msgs::msg::CartesianCommand>(
      "/catchrobo/command/cartesian", 10,
      std::bind(&HardwareBridgeNode::onCartesianCommand, this, std::placeholders::_1));
  } else if (command_mode_ == "joint") {
    // パターンB。kinematics_node の出力 (物理5モータの個別角+角速度) を
    // packet_type = 0x02 で送る。並び順の契約は udp_protocol::kJointOrder
    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/catchrobo/command/joint", 10,
      std::bind(&HardwareBridgeNode::onJointCommand, this, std::placeholders::_1));
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
  // latched: 後から接続した可視化クライアントにも現在のMCU状態が即座に届く
  mcu_status_pub_ = create_publisher<sharmech_msgs::msg::McuStatus>(
    "/catchrobo/arm/mcu_status", rclcpp::QoS(1).transient_local());

  const auto poll_period =
    std::chrono::duration<double>(1.0 / feedback_poll_rate_);
  feedback_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(poll_period),
    std::bind(&HardwareBridgeNode::onFeedbackTimer, this));

  RCLCPP_INFO(
    get_logger(),
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
  local_addr.sin_family = AF_INET;
  local_addr.sin_addr.s_addr = htonl(INADDR_ANY);
  local_addr.sin_port = htons(static_cast<uint16_t>(local_port_));
  if (::bind(
      sockfd_, reinterpret_cast<const sockaddr *>(&local_addr),
      sizeof(local_addr)) < 0)
  {
    RCLCPP_ERROR(
      get_logger(), "bind(:%d) failed: %s",
      local_port_, std::strerror(errno));
    ::close(sockfd_);
    sockfd_ = -1;
    return false;
  }

  std::memset(&mcu_addr_, 0, sizeof(mcu_addr_));
  mcu_addr_.sin_family = AF_INET;
  mcu_addr_.sin_port = htons(static_cast<uint16_t>(mcu_port_));
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
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "UDP send failed: %s", std::strerror(errno));
  }
}

void HardwareBridgeNode::onJointCommand(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  if (sockfd_ < 0) {return;}

  // 並び順は名前で照合する (kinematics_node は kJointOrder と同じ順で publish
  // しているが、UDP側の契約が「配列位置」である以上、送信直前に名前で確定させて
  // おけば上流の並び替えがワイヤフォーマットの破壊にならない)
  std::array<float, udp_protocol::kJointCount> q{};
  std::array<float, udp_protocol::kJointCount> qdot{};
  const bool has_velocity = msg->velocity.size() == msg->name.size();
  for (std::size_t i = 0; i < udp_protocol::kJointCount; ++i) {
    const auto it = std::find(
      msg->name.begin(), msg->name.end(), udp_protocol::kJointOrder[i]);
    if (it == msg->name.end()) {
      if (!warned_joint_cmd_names_) {
        RCLCPP_WARN(
          get_logger(),
          "/catchrobo/command/joint is missing joint '%s'; dropping packet "
          "(expected names: shoulder_left, shoulder_right, turntable, knee_left, knee_right)",
          udp_protocol::kJointOrder[i]);
        warned_joint_cmd_names_ = true;
      }
      return;
    }
    const auto idx = static_cast<std::size_t>(std::distance(msg->name.begin(), it));
    if (idx >= msg->position.size()) {return;}  // name と position の長さ不一致
    q[i] = static_cast<float>(msg->position[idx]);
    qdot[i] = has_velocity ? static_cast<float>(msg->velocity[idx]) : 0.0f;
  }
  // 「位置と速度は常に併送する」原則のため、velocity 欠落は設計違反として警告する
  // (0埋めで送ること自体はできるが、MCU側補間がゼロ次ホールドに退化する)
  if (!has_velocity && !warned_joint_cmd_velocity_) {
    RCLCPP_WARN(
      get_logger(),
      "/catchrobo/command/joint has no velocity array; sending qdot=0 "
      "(MCU-side interpolation degrades to zero-order hold)");
    warned_joint_cmd_velocity_ = true;
  }

  const uint64_t timestamp_us =
    static_cast<uint64_t>(now().nanoseconds() / 1000);
  const auto packet = UdpProtocol::encodeJoint(
    q, qdot, gripper_state_, orient_vertical_state_, send_seq_++, timestamp_us);

  const auto sent = ::sendto(
    sockfd_, packet.data(), packet.size(), 0,
    reinterpret_cast<const sockaddr *>(&mcu_addr_), sizeof(mcu_addr_));
  if (sent < 0) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
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
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Invalid feedback packet discarded (%zd bytes)", received);
      continue;
    }
    // UDP は順序を保証しない。古い seq のパケットは破棄する
    if (last_recv_seq_ && fb->seq <= *last_recv_seq_) {
      ++out_of_order_count_;
      continue;
    }
    last_recv_seq_ = fb->seq;
    latest = std::move(fb);
  }

  if (latest) {
    last_feedback_time_ = now();

    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = *last_feedback_time_;
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
        RCLCPP_WARN(
          get_logger(),
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
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "MCU status_flags = 0x%04x", latest->status_flags);
    }

    // ログに出すだけでは可視化クライアントから見えないので、トピックにも出す
    last_status_flags_ = latest->status_flags;
    last_gripper_state_ = latest->gripper_closed;
    last_seq_echo_ = latest->seq_echo;
    publishMcuStatus(true, 0.0);
  } else if (last_feedback_time_) {
    // 一度でも届いたことがあるのに途絶した場合のみ警告する。
    // MCU 側が未実装のうちからログを埋めないため
    const auto silence = (now() - *last_feedback_time_).seconds();
    if (silence > feedback_timeout_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "No MCU feedback for %.1f s", silence);
      publishMcuStatus(false, silence);
    }
  }
}

// MCU の状態を可視化クライアントへ伝える。
// connected=false のときは status_flags の内容は「最後に受け取った値」であり
// 現在値ではない点に注意 (silence_sec を見て判断すること)
void HardwareBridgeNode::publishMcuStatus(bool connected, double silence_sec)
{
  sharmech_msgs::msg::McuStatus msg;
  msg.header.stamp = now();
  msg.header.frame_id = "field";
  msg.connected = connected;
  msg.status_flags = last_status_flags_;
  msg.gripper_closed = last_gripper_state_;
  msg.seq = last_recv_seq_.value_or(0);
  msg.seq_echo = last_seq_echo_;
  msg.silence_sec = silence_sec;
  msg.out_of_order_count = out_of_order_count_;
  mcu_status_pub_->publish(msg);
}

}  // namespace sharmech_core

RCLCPP_COMPONENTS_REGISTER_NODE(sharmech_core::HardwareBridgeNode)
