#include <gtest/gtest.h>

#include <cstring>
#include <vector>

#include "sharmech_core/utility/udp_protocol.hpp"

using namespace sharmech_core;              // NOLINT
using namespace sharmech_core::udp_protocol;  // NOLINT

TEST(UdpProtocol, EncodeCartesianProducesExpectedByteLayout)
{
  const auto buffer = UdpProtocol::encodeCartesian(
    0.1f, 0.2f, 0.3f, 0.4f, 0.5f,
    0.01f, 0.02f, 0.03f, 0.04f, 0.05f,
    /*gripper_closed=*/ true, /*seq=*/ 42, /*timestamp_us=*/ 123456789ULL);

  ASSERT_EQ(buffer.size(), sizeof(CartesianPacket));

  CartesianPacket packet;
  std::memcpy(&packet, buffer.data(), sizeof(packet));

  EXPECT_EQ(packet.header.protocol_version, kProtocolVersion);
  EXPECT_EQ(packet.header.packet_type, static_cast<uint8_t>(PacketType::kCartesianCommand));
  EXPECT_EQ(packet.header.payload_length, sizeof(CartesianPayload));
  EXPECT_EQ(packet.header.seq, 42u);
  EXPECT_EQ(packet.header.timestamp_us, 123456789ULL);

  EXPECT_FLOAT_EQ(packet.payload.x, 0.1f);
  EXPECT_FLOAT_EQ(packet.payload.y, 0.2f);
  EXPECT_FLOAT_EQ(packet.payload.z, 0.3f);
  EXPECT_FLOAT_EQ(packet.payload.pitch, 0.4f);
  EXPECT_FLOAT_EQ(packet.payload.yaw, 0.5f);
  EXPECT_FLOAT_EQ(packet.payload.vx, 0.01f);
  EXPECT_FLOAT_EQ(packet.payload.vy, 0.02f);
  EXPECT_FLOAT_EQ(packet.payload.vz, 0.03f);
  EXPECT_FLOAT_EQ(packet.payload.pitch_rate, 0.04f);
  EXPECT_FLOAT_EQ(packet.payload.yaw_rate, 0.05f);
  EXPECT_EQ(packet.payload.gripper, 1);
  EXPECT_EQ(packet.payload.control_flags, kControlFlagEnable);
}

TEST(UdpProtocol, EncodeCartesianGripperOpenIsZero)
{
  const auto buffer = UdpProtocol::encodeCartesian(
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, /*gripper_closed=*/ false, 0, 0);
  CartesianPacket packet;
  std::memcpy(&packet, buffer.data(), sizeof(packet));
  EXPECT_EQ(packet.payload.gripper, 0);
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
  fixed.x = 0.11f;
  fixed.y = 0.22f;
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
  EXPECT_FLOAT_EQ(fb->x, 0.11f);
  EXPECT_FLOAT_EQ(fb->y, 0.22f);
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
  const auto buffer = buildFeedbackPacket({}, /*protocol_version=*/ 2);
  EXPECT_FALSE(UdpProtocol::decodeFeedback(buffer.data(), buffer.size()).has_value());
}

TEST(UdpProtocol, DecodeFeedbackRejectsWrongPacketType)
{
  const auto buffer = buildFeedbackPacket(
    {}, kProtocolVersion, static_cast<uint8_t>(PacketType::kCartesianCommand));
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

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
