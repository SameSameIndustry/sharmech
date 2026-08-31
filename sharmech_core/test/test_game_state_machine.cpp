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

// pick から退避完了までの1サイクルを一気に進める。box_count は
// 「通算何個目か」なので、サイクルごとに1ずつ増やして渡す
void runOneCycle(GameStateMachine & machine, int box_count)
{
  machine.onPickPoseReceived(makePose(0.5, 0.1, 0.0));
  machine.consumePendingGoal();
  machine.onGoalReached(0.0);   // → GRASPING
  machine.onBoxCount(box_count);
  machine.tick(1.0);            // → TRANSPORTING
  machine.consumePendingGoal();
  machine.onGoalReached(2.0);   // → PLACING
  machine.consumePendingGoal();
  machine.onGoalReached(3.0);   // → RETRACTING
  machine.consumePendingGoal();
  machine.onGoalReached(4.0);   // → 次のスロット or COMPLETE
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

  // dwellが経過しても、box_count が来ていない (キューが空の) 間は待ち続ける
  machine.tick(10.4);
  EXPECT_EQ(machine.state(), GameState::kGrasping);
  EXPECT_FALSE(machine.hasPendingGoal());

  machine.onBoxCount(1);        // 1個目 → placement_order[0] へ置きに行く
  machine.tick(10.4);
  EXPECT_EQ(machine.state(), GameState::kTransporting);
  ASSERT_TRUE(machine.hasPendingGoal());
  const auto goal = machine.consumePendingGoal();
  // TRANSPORTINGの目標はスロット0のxyに、transport_clearance_zの高さ
  EXPECT_DOUBLE_EQ(goal.x, 0.0);
  EXPECT_DOUBLE_EQ(goal.y, 0.2);
  EXPECT_DOUBLE_EQ(goal.z, 0.20);
}

// box_count はキューに積むだけで、それ自体は状態を動かさない
TEST(GameStateMachine, BoxCountAloneDoesNotChangeState)
{
  GameStateMachine machine(makeConfig());
  machine.onBoxCount(1);
  EXPECT_EQ(machine.state(), GameState::kWaitingForPick);
  EXPECT_TRUE(machine.hasQueuedSlot());
  EXPECT_FALSE(machine.hasPendingGoal());
}

// 掴んだ後に box_count が届いたら、その時点で運搬を始める
// (0->1 なら placement_order[0] のスロットへ)
TEST(GameStateMachine, BoxCountWhileGraspingStartsTransportToFirstSlot)
{
  GameStateMachine machine(makeConfig());
  machine.onPickPoseReceived(makePose(0.5, 0.1, 0.0));
  machine.consumePendingGoal();
  machine.onGoalReached(0.0);
  machine.consumePendingGripper();
  machine.tick(1.0);            // dwellは経過済みだがキューが空
  ASSERT_EQ(machine.state(), GameState::kGrasping);

  machine.onBoxCount(1);
  machine.tick(1.1);
  EXPECT_EQ(machine.state(), GameState::kTransporting);
  ASSERT_TRUE(machine.hasPendingGoal());
  const auto goal = machine.consumePendingGoal();
  EXPECT_DOUBLE_EQ(goal.x, 0.0);   // placement_order[0] = スロット0
  EXPECT_DOUBLE_EQ(goal.z, 0.20);  // transport_clearance_z
}

// 受け取った値が常に正本。0リセット・飛び・減少にそのまま追従する
TEST(GameStateMachine, BoxCountIsAuthoritativeAndFollowsResetAndJumps)
{
  GameStateMachine machine(makeConfig());
  machine.onBoxCount(3);            // 一気に3個目まで飛んでもキューは残る
  EXPECT_TRUE(machine.hasQueuedSlot());

  machine.onBoxCount(0);            // フィールド再設置等でのリセット
  EXPECT_FALSE(machine.hasQueuedSlot());
  ASSERT_TRUE(machine.currentSlotId().has_value());
  EXPECT_EQ(*machine.currentSlotId(), 0);

  machine.onBoxCount(99);           // placement_order の長さ(4)を超える値はクランプ
  EXPECT_TRUE(machine.hasQueuedSlot());
  machine.onBoxCount(-1);           // 負値は0扱い
  EXPECT_FALSE(machine.hasQueuedSlot());
}

// 宛先を確定済みのサイクル (TRANSPORTING/PLACING/RETRACTING) の途中で
// box_count が減っても、そのサイクルは中断しない (ワークを保持したまま
// 別の箱の上へ動いてしまわないようにするため)
TEST(GameStateMachine, BoxCountDecreaseDoesNotInterruptCommittedCycle)
{
  GameStateMachine machine(makeConfig());
  machine.onPickPoseReceived(makePose(0.5, 0.1, 0.0));
  machine.consumePendingGoal();
  machine.onGoalReached(0.0);
  machine.onBoxCount(1);
  machine.tick(1.0);
  ASSERT_EQ(machine.state(), GameState::kTransporting);
  machine.consumePendingGoal();

  machine.onBoxCount(0);            // 運搬中のリセットは効かせない
  EXPECT_EQ(machine.state(), GameState::kTransporting);
  ASSERT_TRUE(machine.currentSlotId().has_value());
  EXPECT_EQ(*machine.currentSlotId(), 0);
}

// 全スロット消化後 (COMPLETE) に box_count が巻き戻ったら待機状態へ戻る
TEST(GameStateMachine, BoxCountRewindLeavesComplete)
{
  GameStateMachine::Config config = makeConfig();
  config.placement_order = {0};
  GameStateMachine machine(config);
  runOneCycle(machine, 1);
  ASSERT_EQ(machine.state(), GameState::kComplete);

  machine.onBoxCount(0);
  EXPECT_EQ(machine.state(), GameState::kWaitingForPick);
  ASSERT_TRUE(machine.currentSlotId().has_value());
  EXPECT_EQ(*machine.currentSlotId(), 0);
}

TEST(GameStateMachine, FullCycleAdvancesToNextSlot)
{
  GameStateMachine machine(makeConfig());

  machine.onPickPoseReceived(makePose(0.5, 0.1, 0.0));
  machine.consumePendingGoal();
  machine.onGoalReached(0.0);   // → GRASPING
  machine.onBoxCount(1);        // 1個目のスロットをキューへ
  machine.tick(1.0);            // dwell経過 + キューあり → TRANSPORTING
  ASSERT_EQ(machine.state(), GameState::kTransporting);
  machine.consumePendingGoal();

  machine.onGoalReached(1.5);   // スロット上空に到達 → PLACING
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

  runOneCycle(machine, 1);      // 最後のスロットなのでCOMPLETEへ
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

TEST(GameStateMachine, FromStringRoundTripsAllStates)
{
  for (const auto state : {
    GameState::kWaitingForPick, GameState::kApproaching, GameState::kGrasping,
    GameState::kTransporting, GameState::kPlacing, GameState::kRetracting,
    GameState::kComplete})
  {
    const auto parsed = sharmech_core::gameStateFromString(sharmech_core::toString(state));
    ASSERT_TRUE(parsed.has_value());
    EXPECT_EQ(*parsed, state);
  }
  EXPECT_FALSE(sharmech_core::gameStateFromString("NOT_A_STATE").has_value());
}

// デバッグ用の強制遷移は状態だけを変え、目標姿勢などは一切出さない。
// これが崩れると、状態を見たいだけのときに実機/シムが動き出してしまう
TEST(GameStateMachine, ForceStateDoesNotEmitPendingCommands)
{
  GameStateMachine machine(makeConfig());
  machine.onPickPoseReceived(makePose(0.5, 0.1, 0.0));
  ASSERT_EQ(machine.state(), GameState::kApproaching);
  ASSERT_TRUE(machine.hasPendingGoal());

  machine.forceState(GameState::kPlacing, 0.0);

  EXPECT_EQ(machine.state(), GameState::kPlacing);
  EXPECT_FALSE(machine.hasPendingGoal());
  EXPECT_FALSE(machine.hasPendingGripper());
  EXPECT_FALSE(machine.hasPendingOrientVertical());
  EXPECT_FALSE(machine.hasPendingWorkspaceClamp());
}

// GRASPING へ飛ばしたときは待機時間の基準時刻が入り直り、
// grasp_dwell_sec 経過後 (かつ box_count のキューがある状態) で TRANSPORTING へ進む
TEST(GameStateMachine, ForceStateToGraspingResetsDwellTimer)
{
  GameStateMachine machine(makeConfig());
  machine.onBoxCount(1);
  machine.forceState(GameState::kGrasping, 100.0);
  ASSERT_EQ(machine.state(), GameState::kGrasping);

  machine.tick(100.1);
  EXPECT_EQ(machine.state(), GameState::kGrasping);

  machine.tick(100.0 + 1.0);
  EXPECT_EQ(machine.state(), GameState::kTransporting);
}

// toggleManualControlは「どの状態からでも」入れる。ここでは自動シーケンスの
// 途中(kApproaching)から入れることを確認する
TEST(GameStateMachine, ToggleManualControlEntersFromAnyState)
{
  GameStateMachine machine(makeConfig());
  machine.onPickPoseReceived(makePose(0.5, 0.1, 0.0));
  machine.consumePendingGoal();
  ASSERT_EQ(machine.state(), GameState::kApproaching);

  machine.toggleManualControl(0.0);
  EXPECT_EQ(machine.state(), GameState::kManualControl);
}

// 再度トグルすると、入る前の状態に戻る
TEST(GameStateMachine, ToggleManualControlTwiceRestoresPreviousState)
{
  GameStateMachine machine(makeConfig());
  machine.onPickPoseReceived(makePose(0.5, 0.1, 0.0));
  machine.consumePendingGoal();
  ASSERT_EQ(machine.state(), GameState::kApproaching);

  machine.toggleManualControl(0.0);
  ASSERT_EQ(machine.state(), GameState::kManualControl);

  machine.toggleManualControl(1.0);
  EXPECT_EQ(machine.state(), GameState::kApproaching);
}

// forceStateと同様、目標姿勢等の副作用コマンドは一切出さない。
// ただし作業領域クランプだけは必ずデフォルトへリセットする
TEST(GameStateMachine, ToggleManualControlResetsWorkspaceClampButNoOtherCommands)
{
  GameStateMachine machine(makeConfig());
  machine.onPickPoseReceived(makePose(0.5, 0.1, 0.0));
  ASSERT_TRUE(machine.hasPendingGoal());
  machine.consumePendingGoal();
  ASSERT_TRUE(machine.hasPendingGripper());
  machine.consumePendingGripper();

  machine.toggleManualControl(0.0);

  EXPECT_FALSE(machine.hasPendingGoal());
  EXPECT_FALSE(machine.hasPendingGripper());
  EXPECT_FALSE(machine.hasPendingOrientVertical());
  ASSERT_TRUE(machine.hasPendingWorkspaceClamp());
  EXPECT_TRUE(machine.consumePendingWorkspaceClamp().reset);
}

// PLACING中(作業領域クランプが絞られている)に自由操作へ入っても、
// クランプはデフォルトへ戻る (ジョグ操作を妨げないようにするため)
TEST(GameStateMachine, ToggleManualControlFromPlacingResetsNarrowedClamp)
{
  GameStateMachine machine(makeConfig());
  machine.onPickPoseReceived(makePose(0.5, 0.1, 0.0));
  machine.consumePendingGoal();
  machine.onGoalReached(0.0);
  machine.onBoxCount(1);
  machine.tick(1.0);
  machine.consumePendingGoal();
  machine.onGoalReached(1.5);
  ASSERT_EQ(machine.state(), GameState::kPlacing);
  ASSERT_TRUE(machine.hasPendingWorkspaceClamp());
  EXPECT_FALSE(machine.consumePendingWorkspaceClamp().reset);  // 絞られている

  machine.toggleManualControl(2.0);

  EXPECT_EQ(machine.state(), GameState::kManualControl);
  ASSERT_TRUE(machine.hasPendingWorkspaceClamp());
  EXPECT_TRUE(machine.consumePendingWorkspaceClamp().reset);  // デフォルトに戻る
}

// 自由操作中は、通常なら WAITING_FOR_PICK に引き戻す却下・中断イベントが来ても
// 状態を維持する (脱出ハッチとして機能し続けるため)
TEST(GameStateMachine, ManualControlIgnoresGoalRejectedOrAborted)
{
  GameStateMachine machine(makeConfig());
  machine.toggleManualControl(0.0);
  ASSERT_EQ(machine.state(), GameState::kManualControl);

  machine.onGoalRejectedOrAborted();
  EXPECT_EQ(machine.state(), GameState::kManualControl);
}

// 自由操作中は pick_request を受理しない (kWaitingForPick 以外では無視される
// 既存ガードがそのまま効く)。box_count はキューに積まれるだけで状態は動かない
TEST(GameStateMachine, ManualControlIgnoresPickRequestAndOnlyQueuesBoxCount)
{
  GameStateMachine machine(makeConfig());
  machine.toggleManualControl(0.0);
  ASSERT_EQ(machine.state(), GameState::kManualControl);

  machine.onPickPoseReceived(makePose(0.5, 0.1, 0.0));
  EXPECT_EQ(machine.state(), GameState::kManualControl);
  EXPECT_FALSE(machine.hasPendingGoal());

  machine.onBoxCount(1);
  machine.tick(1.0);
  EXPECT_EQ(machine.state(), GameState::kManualControl);
  EXPECT_TRUE(machine.hasQueuedSlot());
  EXPECT_FALSE(machine.hasPendingGoal());
}

// kComplete からも自由操作に入れる (試合終了後の片付け等でも使えるように)
TEST(GameStateMachine, ToggleManualControlWorksFromComplete)
{
  GameStateMachine::Config config = makeConfig();
  config.placement_order = {0};
  GameStateMachine machine(config);
  runOneCycle(machine, 1);
  ASSERT_EQ(machine.state(), GameState::kComplete);

  machine.toggleManualControl(4.0);
  EXPECT_EQ(machine.state(), GameState::kManualControl);

  machine.toggleManualControl(5.0);
  EXPECT_EQ(machine.state(), GameState::kComplete);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
