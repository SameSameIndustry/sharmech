#include <gtest/gtest.h>

#include "sharmech_core/utility/game_state_machine.hpp"

using sharmech_core::CartesianState;
using sharmech_core::GameState;
using sharmech_core::GameStateMachine;

namespace
{

GameStateMachine::Config makeConfig()
{
  GameStateMachine::Config config;
  config.slots.resize(4);
  for (int i = 0; i < 4; ++i) {
    config.slots[i].x = 0.1 * i;
    config.slots[i].y = 0.2;
    config.slots[i].z = 0.05;
  }
  config.placement_order = {0, 1, 2, 3};
  config.slot_clamp_margin_m = 0.03;
  config.transport_clearance_z = 0.20;
  config.retract_clearance_z = 0.20;
  config.grasp_dwell_sec = 0.3;
  return config;
}

CartesianState makePose(double x, double y, double z)
{
  CartesianState s;
  s.x = x;
  s.y = y;
  s.z = z;
  return s;
}

}  // namespace

TEST(GameStateMachine, StartsInWaitingForPick)
{
  GameStateMachine machine(makeConfig());
  EXPECT_EQ(machine.state(), GameState::kWaitingForPick);
  ASSERT_TRUE(machine.currentSlotId().has_value());
  EXPECT_EQ(*machine.currentSlotId(), 0);
}

TEST(GameStateMachine, PickPoseTransitionsToApproachingWithGripperOpen)
{
  GameStateMachine machine(makeConfig());
  machine.onPickPoseReceived(makePose(0.5, 0.1, 0.0));

  EXPECT_EQ(machine.state(), GameState::kApproaching);
  ASSERT_TRUE(machine.hasPendingGoal());
  const auto goal = machine.consumePendingGoal();
  EXPECT_DOUBLE_EQ(goal.x, 0.5);
  ASSERT_TRUE(machine.hasPendingGripper());
  EXPECT_FALSE(machine.consumePendingGripper());
}

TEST(GameStateMachine, PickPoseIgnoredUnlessWaitingForPick)
{
  GameStateMachine machine(makeConfig());
  machine.onPickPoseReceived(makePose(0.5, 0.1, 0.0));
  machine.consumePendingGoal();
  ASSERT_EQ(machine.state(), GameState::kApproaching);

  // 既にAPPROACHING中に別のpickが来ても無視される
  machine.onPickPoseReceived(makePose(0.9, 0.9, 0.0));
  EXPECT_EQ(machine.state(), GameState::kApproaching);
  EXPECT_FALSE(machine.hasPendingGoal());
}

TEST(GameStateMachine, GoalReachedWhileApproachingClosesGripperAndDwells)
{
  GameStateMachine machine(makeConfig());
  machine.onPickPoseReceived(makePose(0.5, 0.1, 0.0));
  machine.consumePendingGoal();

  machine.onGoalReached(/*now_sec=*/ 10.0);
  EXPECT_EQ(machine.state(), GameState::kGrasping);
  ASSERT_TRUE(machine.hasPendingGripper());
  EXPECT_TRUE(machine.consumePendingGripper());

  // dwell (0.3s) が経過するまではTRANSPORTINGへ進まない
  machine.tick(10.1);
  EXPECT_EQ(machine.state(), GameState::kGrasping);
  EXPECT_FALSE(machine.hasPendingGoal());

  machine.tick(10.4);
  EXPECT_EQ(machine.state(), GameState::kTransporting);
  ASSERT_TRUE(machine.hasPendingGoal());
  const auto goal = machine.consumePendingGoal();
  // TRANSPORTINGの目標はスロット0のxyに、transport_clearance_zの高さ
  EXPECT_DOUBLE_EQ(goal.x, 0.0);
  EXPECT_DOUBLE_EQ(goal.y, 0.2);
  EXPECT_DOUBLE_EQ(goal.z, 0.20);
}

TEST(GameStateMachine, PlaceRequestOnlyValidWhileTransporting)
{
  GameStateMachine machine(makeConfig());
  machine.onPlaceRequested();  // WAITING_FOR_PICK中なので無視される
  EXPECT_EQ(machine.state(), GameState::kWaitingForPick);
}

TEST(GameStateMachine, FullCycleAdvancesToNextSlot)
{
  GameStateMachine machine(makeConfig());

  machine.onPickPoseReceived(makePose(0.5, 0.1, 0.0));
  machine.consumePendingGoal();
  machine.onGoalReached(0.0);   // → GRASPING
  machine.tick(1.0);            // dwell経過 → TRANSPORTING
  ASSERT_EQ(machine.state(), GameState::kTransporting);
  machine.consumePendingGoal();

  machine.onPlaceRequested();   // → PLACING
  ASSERT_EQ(machine.state(), GameState::kPlacing);
  ASSERT_TRUE(machine.hasPendingWorkspaceClamp());
  const auto clamp = machine.consumePendingWorkspaceClamp();
  EXPECT_FALSE(clamp.reset);
  EXPECT_NEAR(clamp.x_min, 0.0 - 0.03, 1e-9);
  EXPECT_NEAR(clamp.x_max, 0.0 + 0.03, 1e-9);
  ASSERT_TRUE(machine.hasPendingGoal());
  const auto place_goal = machine.consumePendingGoal();
  EXPECT_DOUBLE_EQ(place_goal.z, 0.05);  // スロット0のz
  ASSERT_TRUE(machine.hasPendingOrientVertical());
  EXPECT_TRUE(machine.consumePendingOrientVertical());

  machine.onGoalReached(2.0);   // → RETRACTING
  ASSERT_EQ(machine.state(), GameState::kRetracting);
  ASSERT_TRUE(machine.hasPendingGripper());
  EXPECT_FALSE(machine.consumePendingGripper());  // 置いたので開く
  ASSERT_TRUE(machine.hasPendingWorkspaceClamp());
  EXPECT_TRUE(machine.consumePendingWorkspaceClamp().reset);
  ASSERT_TRUE(machine.hasPendingGoal());
  const auto retract_goal = machine.consumePendingGoal();
  EXPECT_DOUBLE_EQ(retract_goal.z, 0.20);

  machine.onGoalReached(3.0);   // → 次のスロットのWAITING_FOR_PICK
  EXPECT_EQ(machine.state(), GameState::kWaitingForPick);
  ASSERT_TRUE(machine.currentSlotId().has_value());
  EXPECT_EQ(*machine.currentSlotId(), 1);
}

TEST(GameStateMachine, CompletesAfterLastSlot)
{
  GameStateMachine::Config config = makeConfig();
  config.placement_order = {0};  // スロット1つだけの構成でCOMPLETEまで確認する
  GameStateMachine machine(config);

  machine.onPickPoseReceived(makePose(0.5, 0.1, 0.0));
  machine.consumePendingGoal();
  machine.onGoalReached(0.0);
  machine.tick(1.0);
  machine.consumePendingGoal();
  machine.onPlaceRequested();
  machine.consumePendingGoal();
  machine.onGoalReached(2.0);   // → RETRACTING
  machine.consumePendingGoal();

  machine.onGoalReached(3.0);   // 最後のスロットなのでCOMPLETEへ
  EXPECT_EQ(machine.state(), GameState::kComplete);
  EXPECT_FALSE(machine.currentSlotId().has_value());
}

TEST(GameStateMachine, RejectedGoalReturnsToWaitingForPick)
{
  GameStateMachine machine(makeConfig());
  machine.onPickPoseReceived(makePose(0.5, 0.1, 0.0));
  ASSERT_EQ(machine.state(), GameState::kApproaching);

  machine.onGoalRejectedOrAborted();
  EXPECT_EQ(machine.state(), GameState::kWaitingForPick);
}

TEST(GameStateMachine, ToStringCoversAllStates)
{
  EXPECT_EQ(sharmech_core::toString(GameState::kWaitingForPick), "WAITING_FOR_PICK");
  EXPECT_EQ(sharmech_core::toString(GameState::kApproaching), "APPROACHING");
  EXPECT_EQ(sharmech_core::toString(GameState::kGrasping), "GRASPING");
  EXPECT_EQ(sharmech_core::toString(GameState::kTransporting), "TRANSPORTING");
  EXPECT_EQ(sharmech_core::toString(GameState::kPlacing), "PLACING");
  EXPECT_EQ(sharmech_core::toString(GameState::kRetracting), "RETRACTING");
  EXPECT_EQ(sharmech_core::toString(GameState::kComplete), "COMPLETE");
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
