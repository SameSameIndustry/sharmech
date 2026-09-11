#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstring>
#include <utility>
#include <vector>

#include "sharmech_core/utility/polar_utils.hpp"
#include "sharmech_core/utility/udp_protocol.hpp"

using namespace sharmech_core;              // NOLINT
using namespace sharmech_core::udp_protocol;  // NOLINT

TEST(UdpProtocol, EncodePolarProducesExpectedByteLayout)
{
  const auto buffer = UdpProtocol::encodePolar(
    0.1f, 0.2f, 0.3f, 0.4f, 0.5f,
    0.01f, 0.02f, 0.03f, 0.04f, 0.05f,
    /*gripper_closed=*/ true, ControlFlags{},
    /*seq=*/ 42, /*timestamp_us=*/ 123456789ULL);

  ASSERT_EQ(buffer.size(), sizeof(PolarPacket));

  PolarPacket packet;
  std::memcpy(&packet, buffer.data(), sizeof(packet));

  EXPECT_EQ(packet.header.protocol_version, kProtocolVersion);
  EXPECT_EQ(packet.header.packet_type, static_cast<uint8_t>(PacketType::kPolarCommand));
  EXPECT_EQ(packet.header.payload_length, sizeof(PolarPayload));
  EXPECT_EQ(packet.header.seq, 42u);
  EXPECT_EQ(packet.header.timestamp_us, 123456789ULL);

  EXPECT_FLOAT_EQ(packet.payload.r, 0.1f);
  EXPECT_FLOAT_EQ(packet.payload.theta, 0.2f);
  EXPECT_FLOAT_EQ(packet.payload.z, 0.3f);
  EXPECT_FLOAT_EQ(packet.payload.pitch, 0.4f);
  EXPECT_FLOAT_EQ(packet.payload.yaw, 0.5f);
  EXPECT_FLOAT_EQ(packet.payload.r_dot, 0.01f);
  EXPECT_FLOAT_EQ(packet.payload.theta_dot, 0.02f);
  EXPECT_FLOAT_EQ(packet.payload.vz, 0.03f);
  EXPECT_FLOAT_EQ(packet.payload.pitch_rate, 0.04f);
  EXPECT_FLOAT_EQ(packet.payload.yaw_rate, 0.05f);
  EXPECT_EQ(packet.payload.gripper, 1);
  EXPECT_EQ(packet.payload.control_flags, kControlFlagEnable);
}

TEST(UdpProtocol, EncodePolarGripperOpenIsZero)
{
  const auto buffer = UdpProtocol::encodePolar(
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    /*gripper_closed=*/ false, ControlFlags{}, 0, 0);
  PolarPacket packet;
  std::memcpy(&packet, buffer.data(), sizeof(packet));
  EXPECT_EQ(packet.payload.gripper, 0);
}

TEST(UdpProtocol, EncodePolarOrientVerticalSetsBit1)
{
  const auto buffer = UdpProtocol::encodePolar(
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    /*gripper_closed=*/ false, ControlFlags{true, true}, 0, 0);
  PolarPacket packet;
  std::memcpy(&packet, buffer.data(), sizeof(packet));
  EXPECT_EQ(packet.payload.control_flags, kControlFlagEnable | kControlFlagOrientVertical);
}

TEST(UdpProtocol, EncodePolarOrientVerticalFalseLeavesBit1Clear)
{
  const auto buffer = UdpProtocol::encodePolar(
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    /*gripper_closed=*/ false, ControlFlags{}, 0, 0);
  PolarPacket packet;
  std::memcpy(&packet, buffer.data(), sizeof(packet));
  EXPECT_EQ(packet.payload.control_flags, kControlFlagEnable);
}

TEST(UdpProtocol, EncodePolarNeverSetsReservedBit2)
{
  // bit2 (旧・初期位置要求) は 2026-09-11 に予約になった。初期位置は ROS2 側
  // (game_state_manager_node) が普通のゴールで動かすので、ワイヤ上は常に 0
  EXPECT_EQ(kControlFlagInitRequest, 0x04);
  for (const auto flags : {ControlFlags{}, ControlFlags{true, true},
      ControlFlags{false, false}, ControlFlags{false, true}})
  {
    const auto buffer = UdpProtocol::encodePolar(
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      /*gripper_closed=*/ false, flags, 0, 0);
    PolarPacket packet;
    std::memcpy(&packet, buffer.data(), sizeof(packet));
    EXPECT_EQ(packet.payload.control_flags & kControlFlagInitRequest, 0);
  }
}

TEST(UdpProtocol, EncodePolarEnableFalseClearsBit0)
{
  // 動作許可 0 (起動直後・同期前)。他のビットはそのまま載る
  const auto buffer = UdpProtocol::encodePolar(
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    /*gripper_closed=*/ false, ControlFlags{false, false}, 0, 0);
  PolarPacket packet;
  std::memcpy(&packet, buffer.data(), sizeof(packet));
  EXPECT_EQ(packet.payload.control_flags, 0);
  EXPECT_EQ((ControlFlags{false, true}).pack(), kControlFlagOrientVertical);
}

TEST(UdpProtocol, StatusAtInitPoseIsBit5)
{
  // フィードバック側の「初期位置到達」ビット (2026-09-11〜 予約。ROS2 は使わないが
  // ビット割当は互換のため残す)。既存ビットと重ならない
  EXPECT_EQ(kStatusAtInitPose, 1 << 5);
  EXPECT_EQ(
    kStatusAtInitPose & (kStatusTrackingError | kStatusDriverFault |
    kStatusWatchdog | kStatusUninitialized | kStatusCommandRejected), 0);
}

TEST(UdpProtocol, EncodeJointProducesExpectedByteLayout)
{
  const std::array<float, kJointCount> q{0.1f, 0.2f, 0.3f, 0.4f, 0.5f};
  const std::array<float, kJointCount> qdot{0.01f, 0.02f, 0.03f, 0.04f, 0.05f};
  const auto buffer = UdpProtocol::encodeJoint(
    q, qdot, /*gripper_closed=*/ true, ControlFlags{},
    /*seq=*/ 42, /*timestamp_us=*/ 123456789ULL);

  ASSERT_EQ(buffer.size(), sizeof(JointPacket));

  JointPacket packet;
  std::memcpy(&packet, buffer.data(), sizeof(packet));

  EXPECT_EQ(packet.header.protocol_version, kProtocolVersion);
  EXPECT_EQ(packet.header.packet_type, static_cast<uint8_t>(PacketType::kJointCommand));
  EXPECT_EQ(packet.header.payload_length, sizeof(JointPayload));
  EXPECT_EQ(packet.header.seq, 42u);
  EXPECT_EQ(packet.header.timestamp_us, 123456789ULL);

  for (std::size_t i = 0; i < kJointCount; ++i) {
    EXPECT_FLOAT_EQ(packet.payload.q[i], q[i]);
    EXPECT_FLOAT_EQ(packet.payload.qdot[i], qdot[i]);
  }
  EXPECT_EQ(packet.payload.gripper, 1);
  EXPECT_EQ(packet.payload.control_flags, kControlFlagEnable);
}

TEST(UdpProtocol, EncodeJointHasSameSizeAndFlagSemanticsAsPolar)
{
  // ワイヤフォーマットの回帰検出: 0x02 は 0x01 と同じ60バイト・同じフラグ位置。
  // gripper (offset 56) / control_flags (offset 57) が両パケットで同一オフセット
  // であることをバイト列レベルで確認する (MCU側が共通処理にできる根拠)
  const std::array<float, kJointCount> zeros{};
  const auto joint_buf = UdpProtocol::encodeJoint(
    zeros, zeros, /*gripper_closed=*/ true, ControlFlags{true, true}, 0, 0);
  const auto polar_buf = UdpProtocol::encodePolar(
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    /*gripper_closed=*/ true, ControlFlags{true, true}, 0, 0);

  ASSERT_EQ(joint_buf.size(), polar_buf.size());
  EXPECT_EQ(joint_buf[56], polar_buf[56]);  // gripper
  EXPECT_EQ(joint_buf[57], polar_buf[57]);  // control_flags
  EXPECT_EQ(joint_buf[57], kControlFlagEnable | kControlFlagOrientVertical);
}

TEST(UdpProtocol, JointOrderContractIsFiveMotorsInKinematicsNodeOrder)
{
  // 並び順の契約の回帰検出。kinematics_node の JointState name 順・
  // 0x81 フィードバックの joint_positions 順と同一であること (README参照)
  ASSERT_EQ(kJointOrder.size(), 5u);
  EXPECT_STREQ(kJointOrder[0], "shoulder_left");
  EXPECT_STREQ(kJointOrder[1], "shoulder_right");
  EXPECT_STREQ(kJointOrder[2], "turntable");
  EXPECT_STREQ(kJointOrder[3], "knee_left");
  EXPECT_STREQ(kJointOrder[4], "knee_right");
}

namespace
{

// テスト用に MCU が送ってくるフィードバックパケットを模擬して組み立てる
std::vector<uint8_t> buildFeedbackPacket(
  const std::vector<float> & joint_positions,
  uint8_t protocol_version = kProtocolVersion,
  uint8_t packet_type = static_cast<uint8_t>(PacketType::kStateFeedback),
  std::optional<uint16_t> payload_length_override = std::nullopt)
{
  FeedbackFixedPart fixed{};
  fixed.header.protocol_version = protocol_version;
  fixed.header.packet_type = packet_type;
  const uint16_t expected_payload = static_cast<uint16_t>(
    sizeof(FeedbackFixedPart) - sizeof(CommandHeader) +
    sizeof(float) * joint_positions.size());
  fixed.header.payload_length =
    payload_length_override.value_or(expected_payload);
  fixed.header.seq = 7;
  fixed.header.timestamp_us = 99;
  fixed.seq_echo = 6;
  fixed.r = 0.11f;
  fixed.theta = 0.22f;
  fixed.z = 0.33f;
  fixed.pitch = 0.44f;
  fixed.yaw = 0.55f;
  fixed.status_flags = kStatusWatchdog;
  fixed.gripper_state = 1;
  fixed.joint_count = static_cast<uint8_t>(joint_positions.size());

  std::vector<uint8_t> buffer(sizeof(fixed) + sizeof(float) * joint_positions.size());
  std::memcpy(buffer.data(), &fixed, sizeof(fixed));
  if (!joint_positions.empty()) {
    std::memcpy(
      buffer.data() + sizeof(fixed), joint_positions.data(),
      sizeof(float) * joint_positions.size());
  }
  return buffer;
}

}  // namespace

TEST(UdpProtocol, DecodeFeedbackRoundTripsWithJoints)
{
  const std::vector<float> joints{0.1f, 0.2f, 0.3f, 0.4f, 0.5f, 0.6f};
  const auto buffer = buildFeedbackPacket(joints);

  const auto fb = UdpProtocol::decodeFeedback(buffer.data(), buffer.size());
  ASSERT_TRUE(fb.has_value());
  EXPECT_EQ(fb->seq, 7u);
  EXPECT_EQ(fb->seq_echo, 6u);
  EXPECT_EQ(fb->timestamp_us, 99u);
  EXPECT_FLOAT_EQ(fb->r, 0.11f);
  EXPECT_FLOAT_EQ(fb->theta, 0.22f);
  EXPECT_FLOAT_EQ(fb->z, 0.33f);
  EXPECT_FLOAT_EQ(fb->pitch, 0.44f);
  EXPECT_FLOAT_EQ(fb->yaw, 0.55f);
  EXPECT_EQ(fb->status_flags, kStatusWatchdog);
  EXPECT_TRUE(fb->gripper_closed);
  ASSERT_EQ(fb->joint_positions.size(), joints.size());
  for (size_t i = 0; i < joints.size(); ++i) {
    EXPECT_FLOAT_EQ(fb->joint_positions[i], joints[i]);
  }
}

TEST(UdpProtocol, DecodeFeedbackWithZeroJoints)
{
  const auto buffer = buildFeedbackPacket({});
  const auto fb = UdpProtocol::decodeFeedback(buffer.data(), buffer.size());
  ASSERT_TRUE(fb.has_value());
  EXPECT_TRUE(fb->joint_positions.empty());
}

TEST(UdpProtocol, DecodeFeedbackRejectsWrongProtocolVersion)
{
  const auto buffer = buildFeedbackPacket({}, /*protocol_version=*/ kProtocolVersion + 1);
  EXPECT_FALSE(UdpProtocol::decodeFeedback(buffer.data(), buffer.size()).has_value());
}

TEST(UdpProtocol, DecodeFeedbackRejectsWrongPacketType)
{
  const auto buffer = buildFeedbackPacket(
    {}, kProtocolVersion, static_cast<uint8_t>(PacketType::kPolarCommand));
  EXPECT_FALSE(UdpProtocol::decodeFeedback(buffer.data(), buffer.size()).has_value());
}

TEST(UdpProtocol, DecodeFeedbackRejectsMismatchedPayloadLength)
{
  const auto buffer = buildFeedbackPacket(
    {0.1f, 0.2f}, kProtocolVersion,
    static_cast<uint8_t>(PacketType::kStateFeedback), /*payload_length_override=*/ 999);
  EXPECT_FALSE(UdpProtocol::decodeFeedback(buffer.data(), buffer.size()).has_value());
}

TEST(UdpProtocol, DecodeFeedbackRejectsTruncatedBuffer)
{
  const auto buffer = buildFeedbackPacket({0.1f, 0.2f, 0.3f});
  // 関節配列を全部受け取る前に切り詰められたパケットを模擬する
  const std::vector<uint8_t> truncated(
    buffer.begin(), buffer.begin() + sizeof(FeedbackFixedPart));
  EXPECT_FALSE(UdpProtocol::decodeFeedback(truncated.data(), truncated.size()).has_value());
}

TEST(UdpProtocol, DecodeFeedbackRejectsBufferShorterThanFixedPart)
{
  const std::vector<uint8_t> tiny(10, 0);
  EXPECT_FALSE(UdpProtocol::decodeFeedback(tiny.data(), tiny.size()).has_value());
}


// ---- xy → r-θ 変換 (PolarUtils) ----
// UDP に載る値そのものを作る変換なので、ワイヤフォーマットと同じ場所で回帰を見る。
// **ワイヤ上の θ は時計回り (+x → −y) が正** (2026-09-11 ユーザー決定。polar_utils.hpp の
// kThetaSign)。以下の期待値はすべてその向きで書いてある

TEST(PolarUtils, ThetaIsClockwisePositiveOnTheWire)
{
  // +y 方向の点は数学的には +π/2 だが、ワイヤでは時計回り正なので -π/2 になる。
  // +x 上の点で +y へ動く速度も θ̇ < 0 (位置と速度で向きが食い違うと MCU の外挿が逆走する)
  EXPECT_NEAR(PolarUtils::toPolar(0.0, 1.0, 0.0, 0.0, 0.0).theta, -M_PI / 2.0, 1e-12);
  EXPECT_NEAR(PolarUtils::toPolar(0.0, -1.0, 0.0, 0.0, 0.0).theta, +M_PI / 2.0, 1e-12);
  EXPECT_LT(PolarUtils::toPolar(1.0, 0.0, 0.0, 1.0, 0.0).theta_dot, 0.0);
  // 復元も同じ向き: θ = -π/2 は +y の点に戻る
  EXPECT_NEAR(PolarUtils::toX(1.0, -M_PI / 2.0), 0.0, 1e-12);
  EXPECT_NEAR(PolarUtils::toY(1.0, -M_PI / 2.0), 1.0, 1e-12);
  EXPECT_EQ(PolarUtils::kThetaSign, -1.0);
}

TEST(PolarUtils, ConvertsPositionAndVelocityToPolar)
{
  // (x, y) = (3, 4) → r = 5, θ = -atan2(4,3) (時計回り正)
  // 速度は純粋な半径方向 (単位ベクトル (0.6, 0.8) 方向に 1 m/s) にとると
  // ṙ = 1, θ̇ = 0 になるはず
  const auto p = PolarUtils::toPolar(3.0, 4.0, 0.6, 0.8, /*theta_ref=*/ 0.0);
  EXPECT_NEAR(p.r, 5.0, 1e-9);
  EXPECT_NEAR(p.theta, -std::atan2(4.0, 3.0), 1e-9);
  EXPECT_NEAR(p.r_dot, 1.0, 1e-9);
  EXPECT_NEAR(p.theta_dot, 0.0, 1e-9);
}

TEST(PolarUtils, PureTangentialVelocityGivesZeroRadialRate)
{
  // (x, y) = (2, 0) で +y 方向に 1 m/s → ṙ = 0, |θ̇| = v/r = 0.5 rad/s
  // (+y へ動く = 反時計回りなので、時計回り正のワイヤでは負)
  const auto p = PolarUtils::toPolar(2.0, 0.0, 0.0, 1.0, 0.0);
  EXPECT_NEAR(p.r, 2.0, 1e-9);
  EXPECT_NEAR(p.r_dot, 0.0, 1e-9);
  EXPECT_NEAR(p.theta_dot, -0.5, 1e-9);
}

TEST(PolarUtils, UnwrapsThetaAcrossNegativeXAxis)
{
  // 作業領域は X が -2.045〜+0.941 なので -x 軸 (θ = ±π) を実際にまたぐ。
  // atan2 の生値なら ±3.14 で飛ぶところを、直前値の近傍へ連続化する
  // (時計回り正なので +y 側の点は負: (-1, 0.05) ≒ -3.09、(-1, -0.05) は生値で +3.09)
  const double before = PolarUtils::toPolar(-1.0, 0.05, 0, 0, 0.0).theta;   // ≒ -3.09
  const double after = PolarUtils::toPolar(-1.0, -0.05, 0, 0, before).theta;
  EXPECT_LT(after, -M_PI);                      // +π 側へ飛ばず -π を超えて連続
  EXPECT_LT(std::abs(after - before), 0.2);     // 1回転ぶんの飛びが無い
  // cos/sin で戻せば元の直交座標に一致する (アンラップは等価変換)
  EXPECT_NEAR(PolarUtils::toX(1.0, after), -1.0 / std::hypot(1.0, 0.05), 1e-9);
  EXPECT_NEAR(PolarUtils::toY(1.0, after), -0.05 / std::hypot(1.0, 0.05), 1e-9);
}

TEST(PolarUtils, KeepsWindingAfterMultipleTurns)
{
  // 何周しても直前値の近傍を選び続ける (θ は ±π に丸められない)。
  // 反時計回りに a だけ進んだ点は、時計回り正のワイヤでは -a
  double theta = 0.0;
  for (int i = 1; i <= 40; ++i) {
    const double a = i * (M_PI / 4.0);
    theta = PolarUtils::toPolar(std::cos(a), std::sin(a), 0, 0, theta).theta;
    EXPECT_NEAR(theta, -a, 1e-9);
  }
}

// ---- ターンテーブル軸がベース原点からずれている場合 (origin_x/y) ----

TEST(PolarUtils, PolarIsMeasuredFromTurntableAxisNotBaseOrigin)
{
  // 軸が (0.10, -0.05) にある。点 (0.40, 0.35) は軸から見て (0.30, 0.40) → r=0.5
  const auto p = PolarUtils::toPolar(0.40, 0.35, 0.0, 0.0, 0.0, 0.10, -0.05);
  EXPECT_NEAR(p.r, 0.5, 1e-12);
  EXPECT_NEAR(p.theta, -std::atan2(0.40, 0.30), 1e-12);   // 時計回り正

  // オフセット無しなら別の値になる (回帰: オフセットが無視されていないこと)
  const auto q = PolarUtils::toPolar(0.40, 0.35, 0.0, 0.0, 0.0);
  EXPECT_NEAR(q.r, std::hypot(0.40, 0.35), 1e-12);
  EXPECT_GT(std::abs(p.r - q.r), 1e-3);
}

TEST(PolarUtils, VelocityIsAlsoTakenAboutTurntableAxis)
{
  // 軸 (1.0, 0.0)。点 (2.0, 0.0) が +y に 1 m/s → 軸から見て r=1 の純接線速度
  // (反時計回りなので時計回り正のワイヤでは -1 rad/s)
  const auto p = PolarUtils::toPolar(2.0, 0.0, 0.0, 1.0, 0.0, 1.0, 0.0);
  EXPECT_NEAR(p.r, 1.0, 1e-12);
  EXPECT_NEAR(p.r_dot, 0.0, 1e-12);
  EXPECT_NEAR(p.theta_dot, -1.0, 1e-12);
}

TEST(PolarUtils, RoundTripWithAxisOffsetRestoresBaseCoordinates)
{
  const double ax = -0.0123, ay = 0.0456;
  const double xs[] = {0.3, -1.5, 0.0, -0.0123};
  const double ys[] = {0.2, 0.05, -0.6, 0.0456 + 0.2};
  for (int i = 0; i < 4; ++i) {
    const auto p = PolarUtils::toPolar(xs[i], ys[i], 0.0, 0.0, 0.0, ax, ay);
    EXPECT_NEAR(PolarUtils::toX(p.r, p.theta, ax), xs[i], 1e-12) << "case " << i;
    EXPECT_NEAR(PolarUtils::toY(p.r, p.theta, ay), ys[i], 1e-12) << "case " << i;
  }
}

TEST(PolarUtils, SingularityIsAtTurntableAxisNotBaseOrigin)
{
  // ベース原点 (0,0) は軸 (0.1, 0) から r=0.1 なので特異点ではない
  const auto at_base = PolarUtils::toPolar(0.0, 0.0, 1.0, 0.0, 0.0, 0.1, 0.0);
  EXPECT_NEAR(at_base.r, 0.1, 1e-12);
  // 軸から見て真後ろ = ±π。基準 0 からは両方等距離なのでどちらの分岐でもよい
  EXPECT_NEAR(std::abs(at_base.theta), M_PI, 1e-12);
  // 軸の真上が特異点: θ は直前値を保持し速度は 0
  const auto at_axis = PolarUtils::toPolar(0.1, 0.0, 1.0, 0.0, 0.7, 0.1, 0.0);
  EXPECT_NEAR(at_axis.r, 0.0, 1e-12);
  EXPECT_NEAR(at_axis.theta, 0.7, 1e-12);
  EXPECT_NEAR(at_axis.r_dot, 0.0, 1e-12);
  EXPECT_NEAR(at_axis.theta_dot, 0.0, 1e-12);
}

TEST(PolarUtils, OriginSingularityHoldsThetaAndZeroesRates)
{
  // r ≒ 0 では θ が定義できない。NaN を出さず直前の θ を保持し速度を 0 にする
  const auto p = PolarUtils::toPolar(0.0, 0.0, 1.0, 1.0, /*theta_ref=*/ 1.23);
  EXPECT_DOUBLE_EQ(p.r, 0.0);
  EXPECT_DOUBLE_EQ(p.theta, 1.23);
  EXPECT_DOUBLE_EQ(p.r_dot, 0.0);
  EXPECT_DOUBLE_EQ(p.theta_dot, 0.0);
}

TEST(PolarUtils, RoundTripsBackToCartesian)
{
  for (const auto & xy : {std::pair<double, double>{0.5, -0.3},
      {-2.045, 0.675}, {-1.0, 0.0}, {0.941, -0.675}})
  {
    const auto p = PolarUtils::toPolar(xy.first, xy.second, 0, 0, 0.0);
    EXPECT_NEAR(PolarUtils::toX(p.r, p.theta), xy.first, 1e-9);
    EXPECT_NEAR(PolarUtils::toY(p.r, p.theta), xy.second, 1e-9);
  }
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
