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
// packet_type:
//   0x01 = Cartesian 指令 (パターンA・現在)
//   0x02 = 関節指令       (パターンB・将来)
//   0x81 = 状態フィードバック (MCU → ROS2。0x80 以上がフィードバック方向)
namespace udp_protocol
{

constexpr uint8_t kProtocolVersion = 1;

enum class PacketType : uint8_t
{
  kCartesianCommand = 0x01,
  kJointCommand     = 0x02,  // パターンB
  kStateFeedback    = 0x81,
};

// 関節の並び順の契約 (2026-09-01 ユーザー確定)。
// 0x02 指令の q[]/qdot[] と、0x81 フィードバックの joint_positions[] の両方が
// この順に従う。kinematics_node が publish する /catchrobo/command/joint の
// name 順と同一 (ROSトピック・UDP指令・UDPフィードバックの3箇所で1つの順序)
constexpr std::size_t kJointCount = 5;
constexpr std::array<const char *, kJointCount> kJointOrder = {
  "shoulder_left", "shoulder_right", "turntable", "knee_left", "knee_right",
};

// 動作許可フラグ (control_flags)。現時点では常に kEnable を立てる
constexpr uint8_t kControlFlagEnable = 0x01;
// 横倒しのワークを縦向きにしてから置け、という指示 (game_state_manager_node の
// PLACING 状態でのみ立てる)。MCU側がどう実現するかは未定義で、ROS2 側はこの
// フラグを不透明に渡すだけ (グリッパの0/1と同じ扱い)
constexpr uint8_t kControlFlagOrientVertical = 0x02;

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

// packet_type = 0x01 のペイロード (44 バイト)
struct CartesianPayload
{
  float x;              // [m]
  float y;              // [m]
  float z;              // [m]
  float pitch;          // [rad]
  float yaw;            // [rad]
  float vx;             // [m/s]
  float vy;             // [m/s]
  float vz;             // [m/s]
  float pitch_rate;     // [rad/s]
  float yaw_rate;       // [rad/s]
  uint8_t gripper;      // 0=開, 1=閉
  uint8_t control_flags;
  uint16_t reserved;
};

struct CartesianPacket
{
  CommandHeader header;
  CartesianPayload payload;
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
// 直後に float32 × joint_count の関節角配列が続く
struct FeedbackFixedPart
{
  CommandHeader header;
  uint32_t seq_echo;      // 最後に受信した指令の seq
  float x;                // 実位置 [m]
  float y;
  float z;
  float pitch;            // 実姿勢 [rad]
  float yaw;
  uint16_t status_flags;
  uint8_t gripper_state;
  uint8_t joint_count;
};

#pragma pack(pop)

static_assert(sizeof(CommandHeader) == 16, "CommandHeader must be 16 bytes");
static_assert(sizeof(CartesianPayload) == 44, "CartesianPayload must be 44 bytes");
static_assert(sizeof(CartesianPacket) == 60, "CartesianPacket must be 60 bytes");
static_assert(sizeof(JointPayload) == 44, "JointPayload must be 44 bytes");
static_assert(sizeof(JointPacket) == 60, "JointPacket must be 60 bytes");
static_assert(sizeof(FeedbackFixedPart) == 44, "FeedbackFixedPart must be 44 bytes");

// status_flags のビット定義
constexpr uint16_t kStatusTrackingError = 1 << 0;     // 追従誤差過大
constexpr uint16_t kStatusDriverFault = 1 << 1;       // ドライバ異常
constexpr uint16_t kStatusWatchdog = 1 << 2;          // ウォッチドッグ作動中
constexpr uint16_t kStatusUninitialized = 1 << 3;     // 未初期化・原点未確定
constexpr uint16_t kStatusCommandRejected = 1 << 4;   // 直近の指令を破棄した (作業領域外・seq逆転等)

// デコード済みフィードバック
struct Feedback
{
  uint32_t seq;
  uint32_t seq_echo;
  uint64_t timestamp_us;
  float x, y, z, pitch, yaw;
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

  // Cartesian 指令パケット (packet_type = 0x01) を組み立てる
  static std::vector<uint8_t> encodeCartesian(
    float x, float y, float z, float pitch, float yaw,
    float vx, float vy, float vz, float pitch_rate, float yaw_rate,
    bool gripper_closed, bool orient_vertical, uint32_t seq, uint64_t timestamp_us)
  {
    udp_protocol::CartesianPacket packet{};
    packet.header.protocol_version = udp_protocol::kProtocolVersion;
    packet.header.packet_type =
      static_cast<uint8_t>(udp_protocol::PacketType::kCartesianCommand);
    packet.header.payload_length = sizeof(udp_protocol::CartesianPayload);
    packet.header.seq = seq;
    packet.header.timestamp_us = timestamp_us;

    packet.payload.x = x;
    packet.payload.y = y;
    packet.payload.z = z;
    packet.payload.pitch = pitch;
    packet.payload.yaw = yaw;
    packet.payload.vx = vx;
    packet.payload.vy = vy;
    packet.payload.vz = vz;
    packet.payload.pitch_rate = pitch_rate;
    packet.payload.yaw_rate = yaw_rate;
    packet.payload.gripper = gripper_closed ? 1 : 0;
    packet.payload.control_flags = udp_protocol::kControlFlagEnable |
      (orient_vertical ? udp_protocol::kControlFlagOrientVertical : 0);
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
    bool gripper_closed, bool orient_vertical, uint32_t seq, uint64_t timestamp_us)
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
    packet.payload.control_flags = udp_protocol::kControlFlagEnable |
      (orient_vertical ? udp_protocol::kControlFlagOrientVertical : 0);
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
    fb.x = fixed.x;
    fb.y = fixed.y;
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
