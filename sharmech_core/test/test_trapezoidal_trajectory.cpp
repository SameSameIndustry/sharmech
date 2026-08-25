#include <gtest/gtest.h>

#include "sharmech_core/utility/trapezoidal_trajectory.hpp"

using sharmech_core::CartesianState;
using sharmech_core::TrapezoidalTrajectory;

namespace
{
constexpr double kV_max = 0.10;
constexpr double kA_max = 0.20;
constexpr double kW_max = 1.0;
constexpr double kAlpha_max = 2.0;
constexpr double kTol = 1e-6;
}  // namespace

TEST(TrapezoidalTrajectory, ZeroDistanceHasZeroDuration)
{
  CartesianState start{};
  TrapezoidalTrajectory traj(start, start, kV_max, kA_max, kW_max, kAlpha_max);

  EXPECT_NEAR(traj.duration(), 0.0, kTol);

  CartesianState pose, vel;
  traj.sample(0.0, pose, vel);
  EXPECT_NEAR(pose.x, start.x, kTol);
  EXPECT_NEAR(vel.x, 0.0, kTol);
}

TEST(TrapezoidalTrajectory, TriangularProfileWhenDistanceIsShort)
{
  // distance = 0.02 m は 2*(v_max^2/(2*a_max)) = 0.05 m 未満 → 三角プロファイル
  CartesianState start{};
  CartesianState goal;
  goal.x = 0.02;
  TrapezoidalTrajectory traj(start, goal, kV_max, kA_max, kW_max, kAlpha_max);

  const double expected_t_acc = std::sqrt(0.02 / kA_max);
  const double expected_duration = 2.0 * expected_t_acc;
  EXPECT_NEAR(traj.duration(), expected_duration, kTol);

  // ピーク速度は v_max に到達しない
  const double peak_vel = kA_max * expected_t_acc;
  EXPECT_LT(peak_vel, kV_max);

  CartesianState pose, vel;
  traj.sample(expected_t_acc, pose, vel);
  EXPECT_NEAR(vel.x, peak_vel, kTol);
}

TEST(TrapezoidalTrajectory, TrapezoidalProfileWhenDistanceIsLong)
{
  // distance = 0.15 m は 0.05 m を超える → 台形プロファイル。既知の解析解と比較する
  CartesianState start{};
  CartesianState goal;
  goal.x = 0.15;
  TrapezoidalTrajectory traj(start, goal, kV_max, kA_max, kW_max, kAlpha_max);

  EXPECT_NEAR(traj.duration(), 2.0, kTol);  // t_acc=0.5*2 + t_flat=1.0

  CartesianState pose, vel;

  // 加速終了時点で v_max に到達している
  traj.sample(0.5, pose, vel);
  EXPECT_NEAR(vel.x, kV_max, kTol);

  // 定速区間の途中でも v_max を維持する
  traj.sample(1.0, pose, vel);
  EXPECT_NEAR(vel.x, kV_max, kTol);
}

TEST(TrapezoidalTrajectory, SampleAtEndpointsMatchesStartAndGoal)
{
  CartesianState start;
  start.x = 0.1;
  start.y = 0.2;
  CartesianState goal;
  goal.x = 0.15;
  goal.y = 0.05;
  TrapezoidalTrajectory traj(start, goal, kV_max, kA_max, kW_max, kAlpha_max);

  CartesianState pose, vel;
  traj.sample(0.0, pose, vel);
  EXPECT_NEAR(pose.x, start.x, kTol);
  EXPECT_NEAR(pose.y, start.y, kTol);
  EXPECT_NEAR(vel.x, 0.0, kTol);
  EXPECT_NEAR(vel.y, 0.0, kTol);

  traj.sample(traj.duration(), pose, vel);
  EXPECT_NEAR(pose.x, goal.x, kTol);
  EXPECT_NEAR(pose.y, goal.y, kTol);
  EXPECT_NEAR(vel.x, 0.0, kTol);
  EXPECT_NEAR(vel.y, 0.0, kTol);
}

TEST(TrapezoidalTrajectory, TimeIsClampedToTrajectoryRange)
{
  CartesianState start{};
  CartesianState goal;
  goal.x = 0.15;
  TrapezoidalTrajectory traj(start, goal, kV_max, kA_max, kW_max, kAlpha_max);

  CartesianState pose_before, vel_before, pose_at_zero, vel_at_zero;
  traj.sample(-1.0, pose_before, vel_before);
  traj.sample(0.0, pose_at_zero, vel_at_zero);
  EXPECT_NEAR(pose_before.x, pose_at_zero.x, kTol);

  CartesianState pose_after, vel_after, pose_at_end, vel_at_end;
  traj.sample(traj.duration() + 10.0, pose_after, vel_after);
  traj.sample(traj.duration(), pose_at_end, vel_at_end);
  EXPECT_NEAR(pose_after.x, pose_at_end.x, kTol);
  EXPECT_NEAR(vel_after.x, 0.0, kTol);
}

TEST(TrapezoidalTrajectory, PureRotationLeavesPositionUnchanged)
{
  // 並進が無く姿勢のみ変化する場合、姿勢側のプロファイルが支配的になる
  CartesianState start{};
  CartesianState goal;
  goal.yaw = 1.5;
  TrapezoidalTrajectory traj(start, goal, kV_max, kA_max, kW_max, kAlpha_max);

  EXPECT_GT(traj.duration(), 0.0);

  CartesianState pose, vel;
  traj.sample(traj.duration() / 2.0, pose, vel);
  EXPECT_NEAR(pose.x, 0.0, kTol);
  EXPECT_NEAR(pose.y, 0.0, kTol);
  EXPECT_GT(pose.yaw, 0.0);
  EXPECT_LT(pose.yaw, goal.yaw);

  traj.sample(traj.duration(), pose, vel);
  EXPECT_NEAR(pose.yaw, goal.yaw, kTol);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
