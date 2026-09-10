#include <gtest/gtest.h>

#include <vector>

#include "sharmech_core/utility/feedback_merge.hpp"
#include "sharmech_core/utility/udp_protocol.hpp"

using namespace sharmech_core;                // NOLINT
using namespace sharmech_core::udp_protocol;  // NOLINT

namespace
{

Feedback makeArm()
{
  Feedback fb;
  fb.seq = 100;
  fb.seq_echo = 50;
  fb.timestamp_us = 1000;
  fb.r = 0.4f;
  fb.theta = 0.0f;   // 担当外 → 0
  fb.z = 0.15f;
  fb.pitch = 0.0f;
  fb.yaw = 0.0f;
  fb.status_flags = 0;
  fb.gripper_closed = false;
  fb.joint_positions = {0.11f, 0.12f, 0.0f, 0.14f, 0.15f};
  return fb;
}

Feedback makeTheta()
{
  Feedback fb;
  fb.seq = 7;
  fb.seq_echo = 49;
  fb.timestamp_us = 2000;
  fb.r = 0.0f;       // 担当外 → 0
  fb.theta = 2.9f;
  fb.z = 0.0f;
  fb.pitch = 1.57f;
  fb.yaw = -0.3f;
  fb.status_flags = 0;
  fb.gripper_closed = true;
  fb.joint_positions = {0.0f, 0.0f, 2.9f, 0.0f, 0.0f};
  return fb;
}

}  // namespace

TEST(FeedbackMerge, TurntableIndexPointsAtTurntable)
{
  EXPECT_STREQ(kJointOrder[FeedbackMerge::kTurntableJointIndex], "turntable");
}

TEST(FeedbackMerge, FieldsComeFromTheOwningBoard)
{
  const auto merged = FeedbackMerge::merge(makeArm(), makeTheta());

  // r/z 基板から
  EXPECT_FLOAT_EQ(merged.r, 0.4f);
  EXPECT_FLOAT_EQ(merged.z, 0.15f);
  EXPECT_EQ(merged.seq, 100u);
  EXPECT_EQ(merged.seq_echo, 50u);
  EXPECT_EQ(merged.timestamp_us, 1000u);
  // θ 基板から
  EXPECT_FLOAT_EQ(merged.theta, 2.9f);
  EXPECT_FLOAT_EQ(merged.pitch, 1.57f);
  EXPECT_FLOAT_EQ(merged.yaw, -0.3f);
  EXPECT_TRUE(merged.gripper_closed);

  ASSERT_EQ(merged.joint_positions.size(), 5u);
  EXPECT_FLOAT_EQ(merged.joint_positions[0], 0.11f);
  EXPECT_FLOAT_EQ(merged.joint_positions[1], 0.12f);
  EXPECT_FLOAT_EQ(merged.joint_positions[2], 2.9f);   // turntable は θ 基板
  EXPECT_FLOAT_EQ(merged.joint_positions[3], 0.14f);
  EXPECT_FLOAT_EQ(merged.joint_positions[4], 0.15f);
}

TEST(FeedbackMerge, TurntableSlotLeftAloneWhenThetaBoardSendsFewerJoints)
{
  auto theta = makeTheta();
  theta.joint_positions = {1.0f};   // joint_count = 1 (契約違反だが落ちない)
  const auto merged = FeedbackMerge::merge(makeArm(), theta);
  ASSERT_EQ(merged.joint_positions.size(), 5u);
  EXPECT_FLOAT_EQ(merged.joint_positions[2], 0.0f);   // r/z 基板の値 (0) のまま
}

TEST(FeedbackMerge, StatusFlagsOrExceptAtInitPose)
{
  // bit0〜4 は OR
  EXPECT_EQ(
    FeedbackMerge::mergeStatusFlags(kStatusUninitialized, 0), kStatusUninitialized);
  EXPECT_EQ(
    FeedbackMerge::mergeStatusFlags(0, kStatusUninitialized), kStatusUninitialized);
  EXPECT_EQ(
    FeedbackMerge::mergeStatusFlags(kStatusWatchdog, kStatusDriverFault),
    kStatusWatchdog | kStatusDriverFault);
  EXPECT_EQ(FeedbackMerge::mergeStatusFlags(0, 0), 0);

  // bit5 (初期位置到達) は AND: 片方だけでは立たない
  EXPECT_EQ(FeedbackMerge::mergeStatusFlags(kStatusAtInitPose, 0), 0);
  EXPECT_EQ(FeedbackMerge::mergeStatusFlags(0, kStatusAtInitPose), 0);
  EXPECT_EQ(
    FeedbackMerge::mergeStatusFlags(kStatusAtInitPose, kStatusAtInitPose), kStatusAtInitPose);

  // 混在: 片方が到達済み・もう片方が未初期化 → 未初期化だけ残る
  EXPECT_EQ(
    FeedbackMerge::mergeStatusFlags(kStatusAtInitPose, kStatusUninitialized),
    kStatusUninitialized);
}

TEST(FeedbackMerge, MergeAppliesStatusFlagRule)
{
  auto arm = makeArm();
  auto theta = makeTheta();
  arm.status_flags = kStatusAtInitPose | kStatusCommandRejected;
  theta.status_flags = kStatusAtInitPose;
  const auto merged = FeedbackMerge::merge(arm, theta);
  EXPECT_EQ(merged.status_flags, kStatusAtInitPose | kStatusCommandRejected);

  theta.status_flags = 0;
  EXPECT_EQ(FeedbackMerge::merge(arm, theta).status_flags, kStatusCommandRejected);
}
