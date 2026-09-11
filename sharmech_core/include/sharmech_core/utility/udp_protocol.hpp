#ifndef SHARMECH_CORE__UTILITY__UDP_PROTOCOL_HPP_
#define SHARMECH_CORE__UTILITY__UDP_PROTOCOL_HPP_

#include <array>
#include <cstdint>
#include <cstring>
#include <optional>
#include <vector>

namespace sharmech_core
{

// ROS2 ⇄ MCU 間 UDP プロトコル定義
//
// 仕様の正本は sharmech/docs/mcu_spec.md (MCU担当者に渡す仕様書)。
// リトルエンディアン・パディングなし。x86 も STM32 (Cortex-M) もリトルエンディアン
// なのでバイトオーダ変換は行わない。
//
// **座標系: xy平面は極座標 (r, θ) で送る (protocol_version 2 で変更)。**
// この機構の xy 平面はターンテーブルで θ を、肩の対称二軸駆動で r を与える
// r-θ 型のアームなので、直交座標で渡すとマイコン側が毎回 atan2/hypot を
// やり直すことになる。ROS2 層は内部では直交座標のままで、UDP 送信直前に
// PolarUtils で変換する (utility/polar_utils.hpp)。
// z・pitch・yaw は変換の対象外。
//
// packet_type:
//   0x01 = 極座標指令 (パターンA・現在)
//   0x02 = 関節指令   (パターンB・将来)
//   0x81 = 状態フィードバック (MCU → ROS2。0x80 以上がフィードバック方向)
namespace udp_protocol
{

// 2 = xy平面を極座標 (r, θ) に変更した版。1 (直交座標 x, y) とは互換性が無い
constexpr uint8_t kProtocolVersion = 2;

enum class PacketType : uint8_t
{
  kPolarCommand = 0x01,
  kJointCommand = 0x02,  // パターンB
  kStateFeedback = 0x81,
};

// 関節の並び順の契約 (2026-09-01 ユーザー確定)。
// 0x02 指令の q[]/qdot[] と、0x81 フィードバックの joint_positions[] の両方が
// この順に従う。kinematics_node が publish する /catchrobo/command/joint の
// name 順と同一 (ROSトピック・UDP指令・UDPフィードバックの3箇所で1つの順序)
constexpr std::size_t kJointCount = 5;
constexpr std::array<const char *, kJointCount> kJointOrder = {
  "shoulder_left", "shoulder_right", "turntable", "knee_left", "knee_right",
};

// control_flags のビット定義。いずれも「毎パケットに載る状態 (レベル)」であって
// イベントではない (仕様の正本: sharmech/docs/mcu_spec.md §3.2)。
//
// bit0 動作許可。0 のパケットを受けた MCU は現在位置をホールドする (ウォッチドッグ
//      発動時と同じ挙動)。motion_generator_node は起動直後、MCU のフィードバックに
//      目標姿勢を同期し終えるまで 0 を送る —— 同期前の目標姿勢は原点 (0,0,0) の
//      仮値なので、これを MCU が追従すると起動しただけでアームが動いてしまう
constexpr uint8_t kControlFlagEnable = 0x01;
// bit1 横倒しのワークを縦向きにしてから置け、という指示 (game_state_manager_node の
//      PLACING 状態でのみ立てる)。MCU側がどう実現するかは未定義で、ROS2 側はこの
//      フラグを不透明に渡すだけ (グリッパの0/1と同じ扱い)
constexpr uint8_t kControlFlagOrientVertical = 0x02;
// bit2 **予約 (2026-09-11〜 ROS2 は送らない)。** 初期位置は ROS2 側
//      (game_state_manager_node の init_pose) が持ち、普通の 0x01 ストリームで
//      そこへ動かす契約に変えたため、このビットは常に 0 になる。
//      MCU 側に「自前の初期関節角へ行く」実装が残っていても害は無い
//      (ROS2 が立てないので発動しない)。ビット割当だけを互換のために残す
constexpr uint8_t kControlFlagInitRequest = 0x04;

// control_flags を組み立てるための入力。既定値 (動作許可のみ) は
// 「通常運転で r/θ/z に追従せよ」を意味する
// (bit2 = kControlFlagInitRequest は予約。ROS2 は立てないのでメンバを持たない)
struct ControlFlags
{
  bool enable{true};
  bool orient_vertical{false};

  uint8_t pack() const
  {
    return static_cast<uint8_t>(
      (enable ? kControlFlagEnable : 0) |
      (orient_vertical ? kControlFlagOrientVertical : 0));
  }
};

#pragma pack(push, 1)

// 共通ヘッダ (16 バイト)
struct CommandHeader
{
  uint8_t protocol_version;
  uint8_t packet_type;
  uint16_t payload_length;
  uint32_t seq;
  uint64_t timestamp_us;
};

// packet_type = 0x01 のペイロード (44 バイト)。
// xy平面は極座標。θ は ±π を超えうる連続値で届く (-x軸をまたぐ移動で
// +π ⇄ -π に飛ばないよう ROS2 側がアンラップして送るため)
struct PolarPayload
{
  float r;              // [m]   ターンテーブル軸からの距離
  float theta;          // [rad] +x軸から時計回り正 (PolarUtils::kThetaSign)。連続値 (±π を超えうる)
  float z;              // [m]
  float pitch;          // [rad]
  float yaw;            // [rad]
  float r_dot;          // [m/s]   r の時間微分
  float theta_dot;      // [rad/s] θ の時間微分
  float vz;             // [m/s]
  float pitch_rate;     // [rad/s]
  float yaw_rate;       // [rad/s]
  uint8_t gripper;      // 0=開, 1=閉
  uint8_t control_flags;
  uint16_t reserved;
};

struct PolarPacket
{
  CommandHeader header;
  PolarPayload payload;
};

// packet_type = 0x02 のペイロード (44 バイト)。パターンB (ROS2側IK)。
// 0x01 と同じ「位置と速度の併送」原則。関節数は5固定 (このロボット専用の契約
// なので可変長にしない)。並び順は kJointOrder を参照
struct JointPayload
{
  float q[kJointCount];         // 関節角 [rad]
  float qdot[kJointCount];      // 関節角速度 [rad/s]
  uint8_t gripper;              // 0=開, 1=閉 (0x01 と同じ)
  uint8_t control_flags;        // 0x01 と同じビット定義
  uint16_t reserved;
};

struct JointPacket
{
  CommandHeader header;
  JointPayload payload;
};

// packet_type = 0x81 の固定部 (ヘッダ込み 44 バイト)。
// 直後に float32 × joint_count の関節角配列が続く。
// 指令 (0x01) と同じくxy平面は極座標で返す (MCU は xy の運動学を持たなくてよい)
struct FeedbackFixedPart
{
  CommandHeader header;
  uint32_t seq_echo;      // 最後に受信した指令の seq
  float r;                // 実位置 [m]   (FK結果。指令のエコーではない)
  float theta;            // 実位置 [rad] (連続値でなくてよい。ROS2側は cos/sin で戻す)
  float z;                // [m]
  float pitch;            // 実姿勢 [rad]
  float yaw;
  uint16_t status_flags;
  uint8_t gripper_state;
  uint8_t joint_count;
};

#pragma pack(pop)

static_assert(sizeof(CommandHeader) == 16, "CommandHeader must be 16 bytes");
static_assert(sizeof(PolarPayload) == 44, "PolarPayload must be 44 bytes");
static_assert(sizeof(PolarPacket) == 60, "PolarPacket must be 60 bytes");
static_assert(sizeof(JointPayload) == 44, "JointPayload must be 44 bytes");
static_assert(sizeof(JointPacket) == 60, "JointPacket must be 60 bytes");
static_assert(sizeof(FeedbackFixedPart) == 44, "FeedbackFixedPart must be 44 bytes");

// status_flags のビット定義
constexpr uint16_t kStatusTrackingError = 1 << 0;     // 追従誤差過大
constexpr uint16_t kStatusDriverFault = 1 << 1;       // ドライバ異常
constexpr uint16_t kStatusWatchdog = 1 << 2;          // ウォッチドッグ作動中
constexpr uint16_t kStatusUninitialized = 1 << 3;     // 未初期化・原点未確定
constexpr uint16_t kStatusCommandRejected = 1 << 4;   // 直近の指令を破棄した (作業領域外・seq逆転等)
// **予約 (2026-09-11〜 ROS2 は使わない)。** 初期位置要求 (bit2) への到達通知だったが、
// 初期位置が ROS2 側へ移り bit2 を送らなくなったので実機では立たない。
// MCU が返しても ROS2 側の挙動は変わらない (feedback_merge は 2 枚の AND のまま)
constexpr uint16_t kStatusAtInitPose = 1 << 5;

// デコード済みフィードバック。r/theta は極座標のまま
// (直交座標へ戻すのは hardware_bridge_node の責務)
struct Feedback
{
  uint32_t seq;
  uint32_t seq_echo;
  uint64_t timestamp_us;
  float r, theta, z, pitch, yaw;
  uint16_t status_flags;
  bool gripper_closed;
  std::vector<float> joint_positions;
};

}  // namespace udp_protocol

// パケットの組立と解釈 (Static・ヘッダーオンリー)
class UdpProtocol
{
public:
  UdpProtocol() = delete;

  // 極座標指令パケット (packet_type = 0x01) を組み立てる。
  // r/theta/r_dot/theta_dot は PolarUtils::toPolar で変換済みのものを渡すこと
  // (theta はアンラップ済みの連続値であることが呼び出し側の責務)
  static std::vector<uint8_t> encodePolar(
    float r, float theta, float z, float pitch, float yaw,
    float r_dot, float theta_dot, float vz, float pitch_rate, float yaw_rate,
    bool gripper_closed, const udp_protocol::ControlFlags & flags,
    uint32_t seq, uint64_t timestamp_us)
  {
    udp_protocol::PolarPacket packet{};
    packet.header.protocol_version = udp_protocol::kProtocolVersion;
    packet.header.packet_type =
      static_cast<uint8_t>(udp_protocol::PacketType::kPolarCommand);
    packet.header.payload_length = sizeof(udp_protocol::PolarPayload);
    packet.header.seq = seq;
    packet.header.timestamp_us = timestamp_us;

    packet.payload.r = r;
    packet.payload.theta = theta;
    packet.payload.z = z;
    packet.payload.pitch = pitch;
    packet.payload.yaw = yaw;
    packet.payload.r_dot = r_dot;
    packet.payload.theta_dot = theta_dot;
    packet.payload.vz = vz;
    packet.payload.pitch_rate = pitch_rate;
    packet.payload.yaw_rate = yaw_rate;
    packet.payload.gripper = gripper_closed ? 1 : 0;
    packet.payload.control_flags = flags.pack();
    packet.payload.reserved = 0;

    std::vector<uint8_t> buffer(sizeof(packet));
    std::memcpy(buffer.data(), &packet, sizeof(packet));
    return buffer;
  }

  // 関節指令パケット (packet_type = 0x02、パターンB) を組み立てる。
  // q / qdot の並び順は udp_protocol::kJointOrder に従うこと (呼び出し側の責務)
  static std::vector<uint8_t> encodeJoint(
    const std::array<float, udp_protocol::kJointCount> & q,
    const std::array<float, udp_protocol::kJointCount> & qdot,
    bool gripper_closed, const udp_protocol::ControlFlags & flags,
    uint32_t seq, uint64_t timestamp_us)
  {
    udp_protocol::JointPacket packet{};
    packet.header.protocol_version = udp_protocol::kProtocolVersion;
    packet.header.packet_type =
      static_cast<uint8_t>(udp_protocol::PacketType::kJointCommand);
    packet.header.payload_length = sizeof(udp_protocol::JointPayload);
    packet.header.seq = seq;
    packet.header.timestamp_us = timestamp_us;

    for (std::size_t i = 0; i < udp_protocol::kJointCount; ++i) {
      packet.payload.q[i] = q[i];
      packet.payload.qdot[i] = qdot[i];
    }
    packet.payload.gripper = gripper_closed ? 1 : 0;
    packet.payload.control_flags = flags.pack();
    packet.payload.reserved = 0;

    std::vector<uint8_t> buffer(sizeof(packet));
    std::memcpy(buffer.data(), &packet, sizeof(packet));
    return buffer;
  }

  // フィードバックパケット (packet_type = 0x81) を検証してデコードする。
  // 検証に失敗したら std::nullopt (呼び出し側が破棄・ログを担当)
  static std::optional<udp_protocol::Feedback> decodeFeedback(
    const uint8_t * data, size_t size)
  {
    using namespace udp_protocol;

    if (size < sizeof(FeedbackFixedPart)) {return std::nullopt;}

    FeedbackFixedPart fixed;
    std::memcpy(&fixed, data, sizeof(fixed));

    if (fixed.header.protocol_version != kProtocolVersion) {return std::nullopt;}
    if (fixed.header.packet_type !=
      static_cast<uint8_t>(PacketType::kStateFeedback)) {return std::nullopt;}

    const size_t expected_payload =
      sizeof(FeedbackFixedPart) - sizeof(CommandHeader) +
      sizeof(float) * fixed.joint_count;
    if (fixed.header.payload_length != expected_payload) {return std::nullopt;}
    if (size != sizeof(CommandHeader) + expected_payload) {return std::nullopt;}

    Feedback fb;
    fb.seq = fixed.header.seq;
    fb.seq_echo = fixed.seq_echo;
    fb.timestamp_us = fixed.header.timestamp_us;
    fb.r = fixed.r;
    fb.theta = fixed.theta;
    fb.z = fixed.z;
    fb.pitch = fixed.pitch;
    fb.yaw = fixed.yaw;
    fb.status_flags = fixed.status_flags;
    fb.gripper_closed = fixed.gripper_state != 0;

    fb.joint_positions.resize(fixed.joint_count);
    if (fixed.joint_count > 0) {
      std::memcpy(
        fb.joint_positions.data(), data + sizeof(FeedbackFixedPart),
        sizeof(float) * fixed.joint_count);
    }
    return fb;
  }
};

}  // namespace sharmech_core

#endif  // SHARMECH_CORE__UTILITY__UDP_PROTOCOL_HPP_
