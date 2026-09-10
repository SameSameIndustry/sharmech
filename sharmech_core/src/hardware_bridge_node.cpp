#include "sharmech_core/hardware_bridge_node.hpp"
#include "sharmech_core/utility/feedback_merge.hpp"
#include "sharmech_core/utility/orientation_utils.hpp"
#include "sharmech_core/utility/polar_utils.hpp"
#include "sharmech_core/utility/udp_protocol.hpp"

#include <rclcpp_components/register_node_macro.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <algorithm>
#include <array>
#include <cerrno>
#include <cmath>
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
  // θ 基板 (ターンテーブル + エンドエフェクタ)。空なら 1 枚構成 (mcu_ip が全部返す)
  mcu_theta_ip_ = declare_parameter("mcu_theta_ip", std::string(""));
  mcu_theta_port_ = declare_parameter("mcu_theta_port", -1);
  if (mcu_theta_port_ < 0) {mcu_theta_port_ = mcu_port_;}
  local_port_ = declare_parameter("local_port", 8889);
  feedback_poll_rate_ = declare_parameter("feedback_poll_rate", 200.0);
  feedback_timeout_ = declare_parameter("feedback_timeout", 0.5);
  joint_names_ = declare_parameter("joint_names", std::vector<std::string>{});
  // 極座標の原点 (ターンテーブル軸)。robot_geometry.yaml → 生成物で上書きされる
  turntable_axis_x_ = declare_parameter("turntable_axis_x_m", 0.0);
  turntable_axis_y_ = declare_parameter("turntable_axis_y_m", 0.0);
  if (!std::isfinite(turntable_axis_x_) || !std::isfinite(turntable_axis_y_)) {
    RCLCPP_FATAL(get_logger(), "turntable_axis_x_m / y_m must be finite");
    throw std::invalid_argument("turntable_axis_x_m / y_m must be finite");
  }
  if (turntable_axis_x_ != 0.0 || turntable_axis_y_ != 0.0) {
    RCLCPP_INFO(
      get_logger(), "Polar origin (turntable axis) offset from base origin: (%.4f, %.4f) m",
      turntable_axis_x_, turntable_axis_y_);
  }
  param_callback_handle_ = add_on_set_parameters_callback(
    std::bind(&HardwareBridgeNode::onSetParameters, this, std::placeholders::_1));

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
    // 動作許可・初期位置要求は JointState に無いので Cartesian ストリームから拾う
    cartesian_flags_sub_ = create_subscription<sharmech_msgs::msg::CartesianCommand>(
      "/catchrobo/command/cartesian", 10,
      std::bind(&HardwareBridgeNode::onCartesianFlagsOnly, this, std::placeholders::_1));
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

  if (boards_.size() >= 2) {
    RCLCPP_INFO(
      get_logger(),
      "hardware_bridge_node started (mode=%s) → udp r/z %s:%d + theta %s:%d (recv :%d)",
      command_mode_.c_str(), mcu_ip_.c_str(), mcu_port_,
      mcu_theta_ip_.c_str(), mcu_theta_port_, local_port_);
  } else {
    RCLCPP_INFO(
      get_logger(),
      "hardware_bridge_node started (mode=%s) → udp %s:%d (recv :%d, single board)",
      command_mode_.c_str(), mcu_ip_.c_str(), mcu_port_, local_port_);
  }
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

  // 宛先 = 基板の一覧。[0] r/z 基板は必須、[1] θ 基板は mcu_theta_ip が空なら無し
  boards_.clear();
  const auto add_board = [this](const std::string & name, const std::string & ip, int port) {
      McuBoard board;
      board.name = name;
      board.ip = ip;
      board.port = port;
      std::memset(&board.addr, 0, sizeof(board.addr));
      board.addr.sin_family = AF_INET;
      board.addr.sin_port = htons(static_cast<uint16_t>(port));
      if (::inet_pton(AF_INET, ip.c_str(), &board.addr.sin_addr) != 1) {
        RCLCPP_ERROR(
          get_logger(), "Invalid IP for %s board: '%s'", name.c_str(), ip.c_str());
        return false;
      }
      boards_.push_back(board);
      return true;
    };
  bool ok = add_board("r/z", mcu_ip_, mcu_port_);
  if (ok && !mcu_theta_ip_.empty()) {
    ok = add_board("theta", mcu_theta_ip_, mcu_theta_port_);
    if (ok && boards_[0].addr.sin_addr.s_addr == boards_[1].addr.sin_addr.s_addr &&
      boards_[0].port == boards_[1].port)
    {
      // 送信元で見分けられない (mock を 2 つ立てるときはポートを変えること)
      RCLCPP_ERROR(
        get_logger(), "mcu_ip and mcu_theta_ip resolve to the same %s:%d; "
        "feedback cannot be attributed to a board", mcu_ip_.c_str(), mcu_port_);
      ok = false;
    }
  }
  if (!ok) {
    ::close(sockfd_);
    sockfd_ = -1;
    boards_.clear();
    return false;
  }
  return true;
}

void HardwareBridgeNode::sendToAllBoards(const std::vector<uint8_t> & packet)
{
  for (const auto & board : boards_) {
    const auto sent = ::sendto(
      sockfd_, packet.data(), packet.size(), 0,
      reinterpret_cast<const sockaddr *>(&board.addr), sizeof(board.addr));
    if (sent < 0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "UDP send to %s board (%s:%d) failed: %s",
        board.name.c_str(), board.ip.c_str(), board.port, std::strerror(errno));
    }
  }
}

HardwareBridgeNode::McuBoard * HardwareBridgeNode::findBoard(const sockaddr_in & from)
{
  // IP で照合する。同じ IP の基板が複数ある (localhost の mock を 2 つ) ときだけ
  // 送信元ポートでも区別する。実機は送信元ポートを契約に含めていないので、
  // IP が一意なら port は見ない
  McuBoard * ip_match = nullptr;
  int ip_match_count = 0;
  for (auto & board : boards_) {
    if (board.addr.sin_addr.s_addr == from.sin_addr.s_addr) {
      ip_match = &board;
      ++ip_match_count;
    }
  }
  if (ip_match_count <= 1) {return ip_match;}
  for (auto & board : boards_) {
    if (board.addr.sin_addr.s_addr == from.sin_addr.s_addr &&
      board.addr.sin_port == from.sin_port)
    {
      return &board;
    }
  }
  return nullptr;
}

void HardwareBridgeNode::onCartesianCommand(
  const sharmech_msgs::msg::CartesianCommand::SharedPtr msg)
{
  if (sockfd_ < 0) {return;}

  // 動作許可・初期位置要求は位置と同じメッセージで届く (取りこぼしても次で回復)
  enable_state_ = msg->enable;
  init_request_state_ = msg->init_request;

  // 手首は pitch/yaw の2自由度のみ。roll 成分は捨てる
  const auto pitch_yaw = OrientationUtils::toPitchYaw(msg->pose.orientation);

  // xy平面は極座標 (r, θ) で送る契約 (protocol_version 2)。z/pitch/yaw は素通し。
  // 原点はターンテーブル軸 (turntable_axis_x/y_m)。ベース原点からずれていても
  // ここで引くだけで上流・MCU とも影響を受けない。
  // θ は直前に送った値を基準にアンラップし、-x軸をまたいでも +π ⇄ -π に
  // 飛ばない連続値にする (飛ぶとターンテーブルが1回転逆走する)
  const auto polar = PolarUtils::toPolar(
    msg->pose.position.x, msg->pose.position.y,
    msg->twist.linear.x, msg->twist.linear.y, last_sent_theta_,
    turntable_axis_x_, turntable_axis_y_);
  last_sent_theta_ = polar.theta;

  const uint64_t timestamp_us =
    static_cast<uint64_t>(now().nanoseconds() / 1000);

  const auto packet = UdpProtocol::encodePolar(
    static_cast<float>(polar.r),
    static_cast<float>(polar.theta),
    static_cast<float>(msg->pose.position.z),
    static_cast<float>(pitch_yaw.pitch),
    static_cast<float>(pitch_yaw.yaw),
    static_cast<float>(polar.r_dot),
    static_cast<float>(polar.theta_dot),
    static_cast<float>(msg->twist.linear.z),
    static_cast<float>(msg->twist.angular.y),   // pitch_rate
    static_cast<float>(msg->twist.angular.z),   // yaw_rate
    gripper_state_, controlFlags(), send_seq_++, timestamp_us);
  has_sent_command_ = true;
  // 同じパケットを全基板へ送る。各基板は自分の担当フィールドだけ使う
  sendToAllBoards(packet);
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
    q, qdot, gripper_state_, controlFlags(), send_seq_++, timestamp_us);
  has_sent_command_ = true;
  sendToAllBoards(packet);
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

void HardwareBridgeNode::onCartesianFlagsOnly(
  const sharmech_msgs::msg::CartesianCommand::SharedPtr msg)
{
  // joint モード用。位置は使わずフラグだけラッチする
  enable_state_ = msg->enable;
  init_request_state_ = msg->init_request;
}

rcl_interfaces::msg::SetParametersResult HardwareBridgeNode::onSetParameters(
  const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  double nx = turntable_axis_x_;
  double ny = turntable_axis_y_;
  for (const auto & p : params) {
    if (p.get_name() == "turntable_axis_x_m" || p.get_name() == "turntable_axis_y_m") {
      if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE ||
        !std::isfinite(p.as_double()))
      {
        result.successful = false;
        result.reason = p.get_name() + " must be a finite double";
        return result;
      }
      (p.get_name() == "turntable_axis_x_m" ? nx : ny) = p.as_double();
    }
  }
  if (nx != turntable_axis_x_ || ny != turntable_axis_y_) {
    // 原点が変わると同じ (x,y) でも r,θ が変わる = MCU から見ると目標が跳ぶ。
    // MCU のスルーレート制限で追従はするが、変更は停止中に行うこと (docs 参照)
    turntable_axis_x_ = nx;
    turntable_axis_y_ = ny;
    RCLCPP_INFO(
      get_logger(), "Polar origin (turntable axis) updated: (%.4f, %.4f) m",
      turntable_axis_x_, turntable_axis_y_);
  }
  return result;
}

udp_protocol::ControlFlags HardwareBridgeNode::controlFlags() const
{
  udp_protocol::ControlFlags flags;
  flags.enable = enable_state_;
  flags.orient_vertical = orient_vertical_state_;
  flags.init_request = init_request_state_;
  return flags;
}

void HardwareBridgeNode::onFeedbackTimer()
{
  if (sockfd_ < 0) {return;}

  // ソケットに溜まっているデータグラムをすべて読み切り、送信元の基板ごとに
  // 最新のみ採用する
  bool updated = false;
  uint8_t buffer[512];
  const auto t_now = now();
  while (true) {
    sockaddr_in from{};
    socklen_t from_len = sizeof(from);
    const auto received = ::recvfrom(
      sockfd_, buffer, sizeof(buffer), MSG_DONTWAIT,
      reinterpret_cast<sockaddr *>(&from), &from_len);
    if (received < 0) {break;}  // EAGAIN: 読み切った

    auto fb = UdpProtocol::decodeFeedback(buffer, static_cast<size_t>(received));
    if (!fb) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Invalid feedback packet discarded (%zd bytes)", received);
      continue;
    }
    McuBoard * board = findBoard(from);
    if (board == nullptr) {
      char ip_text[INET_ADDRSTRLEN] = {};
      ::inet_ntop(AF_INET, &from.sin_addr, ip_text, sizeof(ip_text));
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Feedback from unknown source %s:%u discarded (not mcu_ip / mcu_theta_ip)",
        ip_text, static_cast<unsigned>(ntohs(from.sin_port)));
      continue;
    }
    // feedback_timeout を超えて途絶した後の再開は基板の再起動とみなし、seq の基準を
    // 捨てる。MCU は再起動すると seq を 0 から振り直すので、前回の最大値を覚えたままだと
    // 追いつくまで (長時間運転後なら数十秒〜) 全パケットを順序逆転として捨ててしまう
    if (board->last_seq && board->last_time &&
      (t_now - *board->last_time).seconds() > feedback_timeout_)
    {
      RCLCPP_INFO(
        get_logger(), "Feedback from %s board resumed after %.1f s (seq %u → %u); "
        "treating as MCU restart",
        board->name.c_str(), (t_now - *board->last_time).seconds(),
        *board->last_seq, fb->seq);
      board->last_seq.reset();
    }
    // UDP は順序を保証しない。古い seq のパケットは破棄する (連番は基板ごと)
    if (board->last_seq && fb->seq <= *board->last_seq) {
      ++board->out_of_order;
      continue;
    }
    board->last_seq = fb->seq;
    board->latest = std::move(fb);
    board->last_time = t_now;
    updated = true;
  }

  if (updated) {
    // 全基板が一度以上届いていて、かつ全部が feedback_timeout 以内に更新されている
    // ときだけ合成して publish する。片方しか無い姿勢を流すと motion_generator_node が
    // 半端な姿勢に同期してしまう (「フィードバックが無いときにエコーしない」と同じ理由)
    const McuBoard * missing = nullptr;
    const McuBoard * stale = nullptr;
    double max_silence = 0.0;
    for (const auto & board : boards_) {
      if (!board.latest) {
        missing = &board;
        break;
      }
      const double silence = (t_now - *board.last_time).seconds();
      max_silence = std::max(max_silence, silence);
      if (silence > feedback_timeout_) {stale = &board;}
    }
    if (missing != nullptr) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Waiting for feedback from %s board (%s:%d); /catchrobo/arm/current_pose is "
        "withheld until every board reports",
        missing->name.c_str(), missing->ip.c_str(), missing->port);
      return;
    }

    udp_protocol::Feedback merged =
      boards_.size() >= 2 ?
      FeedbackMerge::merge(*boards_[0].latest, *boards_[1].latest) :
      *boards_[0].latest;
    // ログに出すだけでは可視化クライアントから見えないので、トピックにも出す
    last_status_flags_ = merged.status_flags;
    last_gripper_state_ = merged.gripper_closed;
    last_seq_ = merged.seq;
    last_seq_echo_ = merged.seq_echo;

    if (stale != nullptr) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "No feedback from %s board for %.1f s; /catchrobo/arm/current_pose withheld",
        stale->name.c_str(), (t_now - *stale->last_time).seconds());
      publishMcuStatus(false, max_silence);
      return;
    }

    last_feedback_time_ = t_now;
    publishFeedback(merged, t_now);
    publishMcuStatus(true, 0.0);
  } else if (last_feedback_time_) {
    // 一度でも揃って届いたことがあるのに途絶した場合のみ警告する。
    // MCU 側が未実装のうちからログを埋めないため
    const auto silence = (t_now - *last_feedback_time_).seconds();
    if (silence > feedback_timeout_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "No MCU feedback for %.1f s", silence);
      publishMcuStatus(false, silence);
    }
  }
}

void HardwareBridgeNode::publishFeedback(
  const udp_protocol::Feedback & fb, const rclcpp::Time & stamp)
{
  // ROS2 がまだ駆動していない間は、θ のアンラップ基準を MCU の実 θ に合わせておく
  // (ヘッダの last_sent_theta_ のコメント参照)。駆動中は送信値が基準
  if (!has_sent_command_ || !enable_state_ || init_request_state_) {
    last_sent_theta_ = fb.theta;
  }

  geometry_msgs::msg::PoseStamped pose_msg;
  pose_msg.header.stamp = stamp;
  pose_msg.header.frame_id = "field";
  // フィードバックも極座標で届く (0x81)。/catchrobo/arm/current_pose は
  // VR・シミュレータとの契約で base 座標系の直交座標なのでここで戻す
  // (送信時と同じターンテーブル軸オフセットを足す)
  pose_msg.pose.position.x = PolarUtils::toX(fb.r, fb.theta, turntable_axis_x_);
  pose_msg.pose.position.y = PolarUtils::toY(fb.r, fb.theta, turntable_axis_y_);
  pose_msg.pose.position.z = fb.z;
  tf2::Quaternion q;
  q.setRPY(0.0, fb.pitch, fb.yaw);
  pose_msg.pose.orientation = tf2::toMsg(q);
  current_pose_pub_->publish(pose_msg);

  sensor_msgs::msg::JointState js;
  js.header.stamp = stamp;
  const size_t n = fb.joint_positions.size();
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
  js.position.assign(fb.joint_positions.begin(), fb.joint_positions.end());
  joint_states_pub_->publish(js);

  if (fb.status_flags != 0) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "MCU status_flags = 0x%04x", fb.status_flags);
  }
}

// MCU の状態を可視化クライアントへ伝える。
// connected=false のときは status_flags の内容は「最後に受け取った値」であり
// 現在値ではない点に注意 (silence_sec を見て判断すること)。
// 2 枚構成では connected = 全基板が feedback_timeout 以内、seq は r/z 基板の連番、
// out_of_order_count は全基板の合計
void HardwareBridgeNode::publishMcuStatus(bool connected, double silence_sec)
{
  sharmech_msgs::msg::McuStatus msg;
  msg.header.stamp = now();
  msg.header.frame_id = "field";
  msg.connected = connected;
  msg.status_flags = last_status_flags_;
  msg.gripper_closed = last_gripper_state_;
  msg.seq = last_seq_;
  msg.seq_echo = last_seq_echo_;
  msg.silence_sec = silence_sec;
  uint32_t out_of_order = 0;
  for (const auto & board : boards_) {
    out_of_order += board.out_of_order;
  }
  msg.out_of_order_count = out_of_order;
  mcu_status_pub_->publish(msg);
}

}  // namespace sharmech_core

RCLCPP_COMPONENTS_REGISTER_NODE(sharmech_core::HardwareBridgeNode)
