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
  config.orient_dwell_sec = 0.5;
  config.init_pose = CartesianState{};
  config.init_pose.x = 0.0;
  config.init_pose.y = 0.15;
  config.init_pose.z = 0.15;
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
// ワークを掴むところまで進める。接近は「水平移動 → 垂直降下」の2段なので
// onGoalReached が2回要る
void advanceToGrasping(GameStateMachine & machine, const CartesianState & pose)
{
  machine.onPickPoseReceived(pose);
  machine.consumePendingGoal();
  machine.onGoalReached(0.0);   // ワーク上空に到達 → APPROACH_DESCEND
  machine.consumePendingGoal();
  machine.onGoalReached(0.0);   // 降下完了 → GRASPING
}

void runOneCycle(GameStateMachine & machine, int box_count)
{
  advanceToGrasping(machine, makePose(0.5, 0.1, 0.0));
  machine.onBoxCount(box_count);
  machine.tick(1.0);            // → TRANSPORT_LIFT (掴んだ位置で垂直上昇)
  machine.consumePendingGoal();
  machine.onGoalReached(1.5);   // 上昇完了 → TRANSPORTING (水平移動)
  machine.consumePendingGoal();
  machine.onGoalReached(2.0);   // スロット上空に到達 → ORIENTING (ゴールは出ない)
  machine.consumePendingOrientVertical();  // 縦にする指示
  machine.tick(2.6);            // orient dwell (0.5s) 経過 → PLACING
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
  machine.consumePendingGripper();   // 接近開始時の「開」

  // 1段目の到達 (ワーク上空) では掴まない。垂直降下に入るだけ
  machine.onGoalReached(/*now_sec=*/ 9.0);
  EXPECT_EQ(machine.state(), GameState::kApproachDescend);
  EXPECT_FALSE(machine.hasPendingGripper());
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
  EXPECT_EQ(machine.state(), GameState::kTransportLift);
  ASSERT_TRUE(machine.hasPendingGoal());
  const auto lift = machine.consumePendingGoal();
  // 上昇は掴んだ場所の真上。xyは掴んだ位置のまま、zだけ上がる (斜めに持ち上げない)
  EXPECT_DOUBLE_EQ(lift.x, 0.5);
  EXPECT_DOUBLE_EQ(lift.y, 0.1);
  EXPECT_DOUBLE_EQ(lift.z, 0.20);

  machine.onGoalReached(11.0);
  EXPECT_EQ(machine.state(), GameState::kTransporting);
  ASSERT_TRUE(machine.hasPendingGoal());
  const auto goal = machine.consumePendingGoal();
  // TRANSPORTINGの目標はスロット0のxyに、transport_clearance_zの高さ。
  // 上昇後と同じ高さなので、この区間は完全な水平移動になる
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
  advanceToGrasping(machine, makePose(0.5, 0.1, 0.0));
  machine.consumePendingGripper();
  machine.tick(1.0);            // dwellは経過済みだがキューが空
  ASSERT_EQ(machine.state(), GameState::kGrasping);

  machine.onBoxCount(1);
  machine.tick(1.1);
  ASSERT_EQ(machine.state(), GameState::kTransportLift);
  machine.consumePendingGoal();
  machine.onGoalReached(1.5);
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
  advanceToGrasping(machine, makePose(0.5, 0.1, 0.0));
  machine.onBoxCount(1);
  machine.tick(1.0);
  machine.consumePendingGoal();
  machine.onGoalReached(1.5);
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
  machine.onGoalReached(0.0);   // ワーク上空 → APPROACH_DESCEND
  machine.consumePendingGoal();
  machine.onGoalReached(0.2);   // 降下完了 → GRASPING
  machine.onBoxCount(1);        // 1個目のスロットをキューへ
  machine.tick(1.0);            // dwell経過 + キューあり → TRANSPORT_LIFT
  ASSERT_EQ(machine.state(), GameState::kTransportLift);
  machine.consumePendingGoal();
  machine.onGoalReached(1.5);   // 上昇完了 → TRANSPORTING (水平移動)
  ASSERT_EQ(machine.state(), GameState::kTransporting);
  machine.consumePendingGoal();

  // スロット上空に到達 → ORIENTING。ここでは縦にする指示とクランプ絞りだけを行い、
  // **ゴールは出さない** (アームは静止したまま回転し切るのを待つ)
  machine.onGoalReached(1.5);
  ASSERT_EQ(machine.state(), GameState::kOrienting);
  ASSERT_TRUE(machine.hasPendingOrientVertical());
  EXPECT_TRUE(machine.consumePendingOrientVertical());
  EXPECT_FALSE(machine.hasPendingGoal());
  ASSERT_TRUE(machine.hasPendingWorkspaceClamp());
  const auto clamp = machine.consumePendingWorkspaceClamp();
  EXPECT_FALSE(clamp.reset);
  EXPECT_NEAR(clamp.x_min, 0.0 - 0.03, 1e-9);
  EXPECT_NEAR(clamp.x_max, 0.0 + 0.03, 1e-9);

  // orient dwell 経過 → PLACING。ここで初めて降下ゴールが出る
  machine.tick(2.1);
  ASSERT_EQ(machine.state(), GameState::kPlacing);
  ASSERT_TRUE(machine.hasPendingGoal());
  const auto place_goal = machine.consumePendingGoal();
  EXPECT_DOUBLE_EQ(place_goal.z, 0.05);  // スロット0のz

  machine.onGoalReached(2.5);   // → RETRACTING
  ASSERT_EQ(machine.state(), GameState::kRetracting);
  ASSERT_TRUE(machine.hasPendingGripper());
  EXPECT_FALSE(machine.consumePendingGripper());  // 置いたので開く
  // **縦のまま抜く**ので orient_vertical の指令は出ない (箱の中で回さないため)
  EXPECT_FALSE(machine.hasPendingOrientVertical());
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
  EXPECT_EQ(machine.state(), GameState::kTransportLift);
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
  machine.consumePendingGoal();
  machine.onGoalReached(0.2);
  machine.onBoxCount(1);
  machine.tick(1.0);
  machine.consumePendingGoal();
  machine.onGoalReached(1.5);   // 上昇完了 → TRANSPORTING
  machine.consumePendingGoal();
  machine.onGoalReached(2.0);   // スロット上空 → ORIENTING
  ASSERT_EQ(machine.state(), GameState::kOrienting);
  ASSERT_TRUE(machine.hasPendingWorkspaceClamp());
  EXPECT_FALSE(machine.consumePendingWorkspaceClamp().reset);  // 絞られている
  machine.tick(2.6);                                           // → PLACING (絞りは継続中)
  ASSERT_EQ(machine.state(), GameState::kPlacing);

  machine.toggleManualControl(2.5);

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

// スロット上空に着いても、orient dwell が経過するまでは降下しない。
// 降下と同時に縦にすると回転が間に合わず缶が斜めのまま箱へ入るため、
// 動かない待ち時間をここで確保しているという設計の回帰テスト
TEST(GameStateMachine, OrientingHoldsPositionUntilDwellElapses)
{
  GameStateMachine machine(makeConfig());
  advanceToGrasping(machine, makePose(0.5, 0.1, 0.0));
  machine.onBoxCount(1);
  machine.tick(1.0);
  machine.consumePendingGoal();
  machine.onGoalReached(1.5);   // 上昇完了 → TRANSPORTING
  machine.consumePendingGoal();

  machine.onGoalReached(2.0);   // スロット上空に到達
  ASSERT_EQ(machine.state(), GameState::kOrienting);
  // 縦にする指示だけが出て、移動のゴールは出ない (静止したまま回す)
  ASSERT_TRUE(machine.hasPendingOrientVertical());
  EXPECT_TRUE(machine.consumePendingOrientVertical());
  EXPECT_FALSE(machine.hasPendingGoal());

  // dwell (0.5s) 未満では ORIENTING のまま。降下ゴールも出ない
  // (ORIENTING に入ったのは t=2.0)
  machine.tick(2.4);
  EXPECT_EQ(machine.state(), GameState::kOrienting);
  EXPECT_FALSE(machine.hasPendingGoal());

  // dwell 経過でようやく降下に入る
  machine.tick(2.5);
  EXPECT_EQ(machine.state(), GameState::kPlacing);
  ASSERT_TRUE(machine.hasPendingGoal());
  EXPECT_DOUBLE_EQ(machine.consumePendingGoal().z, 0.05);  // スロットのz
}

// 移動はすべて「垂直 → 水平 → 垂直」のL字に分解されており、斜めに動く区間が
// 無いことの回帰テスト。斜めに降りると隣のワーク(200mmピッチ)を薙ぎ払い、
// 斜めに持ち上げると缶を引きずるため
TEST(GameStateMachine, EveryMoveIsEitherPurelyVerticalOrPurelyHorizontal)
{
  GameStateMachine machine(makeConfig());
  const auto pick = makePose(0.5, 0.1, 0.0);

  // 1. 接近: ワークの真上まで。高さは approach_clearance_z で、xyはワークの真上
  machine.onPickPoseReceived(pick);
  ASSERT_EQ(machine.state(), GameState::kApproaching);
  ASSERT_TRUE(machine.hasPendingGoal());
  const auto approach = machine.consumePendingGoal();
  EXPECT_DOUBLE_EQ(approach.x, pick.x);
  EXPECT_DOUBLE_EQ(approach.y, pick.y);
  EXPECT_DOUBLE_EQ(approach.z, 0.20);   // approach_clearance_z

  // 2. 降下: xyは動かさず、zだけワークの高さへ (完全な垂直移動)
  machine.onGoalReached(1.0);
  ASSERT_EQ(machine.state(), GameState::kApproachDescend);
  ASSERT_TRUE(machine.hasPendingGoal());
  const auto descend = machine.consumePendingGoal();
  EXPECT_DOUBLE_EQ(descend.x, approach.x);
  EXPECT_DOUBLE_EQ(descend.y, approach.y);
  EXPECT_DOUBLE_EQ(descend.z, pick.z);

  // 3. 掴む
  machine.onGoalReached(2.0);
  ASSERT_EQ(machine.state(), GameState::kGrasping);
  machine.onBoxCount(1);

  // 4. 上昇: 掴んだ場所の真上へ (xyは掴んだ位置のまま = 完全な垂直移動)
  machine.tick(2.5);
  ASSERT_EQ(machine.state(), GameState::kTransportLift);
  ASSERT_TRUE(machine.hasPendingGoal());
  const auto lift = machine.consumePendingGoal();
  EXPECT_DOUBLE_EQ(lift.x, pick.x);
  EXPECT_DOUBLE_EQ(lift.y, pick.y);
  EXPECT_DOUBLE_EQ(lift.z, 0.20);       // transport_clearance_z

  // 5. 運搬: 高さを変えずにスロット上空へ (完全な水平移動)
  machine.onGoalReached(3.0);
  ASSERT_EQ(machine.state(), GameState::kTransporting);
  ASSERT_TRUE(machine.hasPendingGoal());
  const auto traverse = machine.consumePendingGoal();
  EXPECT_DOUBLE_EQ(traverse.z, lift.z);   // ★上昇後と同じ高さ = 水平
  EXPECT_DOUBLE_EQ(traverse.x, 0.0);      // スロット0のxy
  EXPECT_DOUBLE_EQ(traverse.y, 0.2);

  // 6. 縦にする (静止)、7. 降下: xyは変えずスロットの高さへ (完全な垂直移動)
  machine.onGoalReached(4.0);
  ASSERT_EQ(machine.state(), GameState::kOrienting);
  machine.tick(4.6);
  ASSERT_EQ(machine.state(), GameState::kPlacing);
  ASSERT_TRUE(machine.hasPendingGoal());
  const auto place = machine.consumePendingGoal();
  EXPECT_DOUBLE_EQ(place.x, traverse.x);
  EXPECT_DOUBLE_EQ(place.y, traverse.y);
  EXPECT_DOUBLE_EQ(place.z, 0.05);        // スロットのz

  // 8. 退避: xyは変えず真上へ (完全な垂直移動)
  machine.onGoalReached(5.0);
  ASSERT_EQ(machine.state(), GameState::kRetracting);
  ASSERT_TRUE(machine.hasPendingGoal());
  const auto retract = machine.consumePendingGoal();
  EXPECT_DOUBLE_EQ(retract.x, place.x);
  EXPECT_DOUBLE_EQ(retract.y, place.y);
  EXPECT_DOUBLE_EQ(retract.z, 0.20);      // retract_clearance_z
}

// 退避高さと接近高さが揃っていれば、次サイクルの接近は高さを変えずに始められる
// (揃っていないとその差分だけ斜めになる、という config 上の約束の回帰テスト)
TEST(GameStateMachine, ApproachClearanceMatchesRetractClearanceSoTraverseIsFlat)
{
  auto config = makeConfig();
  EXPECT_DOUBLE_EQ(config.approach_clearance_z, config.retract_clearance_z);
}

// 箱の中でグリッパを回さないため、RETRACTING では縦のまま抜き、
// 横へ戻すのは次の APPROACHING (空中の長い移動) で行う
TEST(GameStateMachine, StaysVerticalThroughRetractAndReturnsFlatOnNextApproach)
{
  GameStateMachine machine(makeConfig());
  runOneCycle(machine, 1);
  ASSERT_EQ(machine.state(), GameState::kWaitingForPick);
  // 1サイクルを通して、最後に出た orient_vertical 指令は ORIENTING の true のみ。
  // RETRACTING では false を出していない (runOneCycle 中に消費済みでないことを確認)
  EXPECT_FALSE(machine.hasPendingOrientVertical());

  // 次の pick でようやく横へ戻す指示が出る
  machine.onPickPoseReceived(makePose(0.4, 0.1, 0.0));
  ASSERT_TRUE(machine.hasPendingOrientVertical());
  EXPECT_FALSE(machine.consumePendingOrientVertical());
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

// --- 状態のリセット (/catchrobo/game/reset → INIT) ---------------------------

// どの状態からでも INIT に入り、初期位置へのゴールを1本出す。
// グリッパは開き、縦は解除し、作業領域クランプはデフォルトへ戻す
TEST(GameStateMachine, ResetEntersInitAndGoesToInitPose)
{
  GameStateMachine machine(makeConfig());
  advanceToGrasping(machine, makePose(0.5, 0.1, 0.0));
  machine.consumePendingGripper();
  ASSERT_EQ(machine.state(), GameState::kGrasping);

  machine.requestInit();
  EXPECT_EQ(machine.state(), GameState::kInit);

  ASSERT_TRUE(machine.hasPendingGoal());
  const auto goal = machine.consumePendingGoal();
  EXPECT_DOUBLE_EQ(goal.x, 0.0);
  EXPECT_DOUBLE_EQ(goal.y, 0.15);
  EXPECT_DOUBLE_EQ(goal.z, 0.15);

  ASSERT_TRUE(machine.hasPendingGripper());
  EXPECT_FALSE(machine.consumePendingGripper());          // 開く
  ASSERT_TRUE(machine.hasPendingOrientVertical());
  EXPECT_FALSE(machine.consumePendingOrientVertical());    // 横へ戻す
  ASSERT_TRUE(machine.hasPendingWorkspaceClamp());
  EXPECT_TRUE(machine.consumePendingWorkspaceClamp().reset);
}

// 初期位置へ着いたら待機状態に戻り、そのまま次の pick を受けられる
TEST(GameStateMachine, InitReturnsToWaitingForPickOnGoalReached)
{
  GameStateMachine machine(makeConfig());
  machine.requestInit();
  machine.consumePendingGoal();

  machine.onGoalReached(1.0);
  EXPECT_EQ(machine.state(), GameState::kWaitingForPick);

  machine.onPickPoseReceived(makePose(0.4, 0.1, 0.0));
  EXPECT_EQ(machine.state(), GameState::kApproaching);
}

// **自由操作中でもリセットできる。** リセットは VR のボタンなので、押せている
// 時点で VR は生きている (kManualControl は「VRが使えないときの脱出ハッチ」)。
// そのまま初期位置へ戻し、到達したら待機へ復帰する
TEST(GameStateMachine, ResetPullsOutOfManualControl)
{
  GameStateMachine machine(makeConfig());
  machine.onPickPoseReceived(makePose(0.5, 0.1, 0.0));
  machine.consumePendingGoal();
  ASSERT_EQ(machine.state(), GameState::kApproaching);

  machine.toggleManualControl(1.0);
  ASSERT_EQ(machine.state(), GameState::kManualControl);

  machine.requestInit();
  EXPECT_EQ(machine.state(), GameState::kInit);
  ASSERT_TRUE(machine.hasPendingGoal());
  EXPECT_DOUBLE_EQ(machine.consumePendingGoal().z, 0.15);  // init_pose

  machine.onGoalReached(2.0);
  EXPECT_EQ(machine.state(), GameState::kWaitingForPick);
}

// 初期位置へのゴールが却下・中断されたら、他の自動シーケンスと同じ扱いで待機へ落ちる
TEST(GameStateMachine, InitFallsBackToWaitingForPickWhenGoalRejected)
{
  GameStateMachine machine(makeConfig());
  machine.requestInit();
  machine.consumePendingGoal();

  machine.onGoalRejectedOrAborted();
  EXPECT_EQ(machine.state(), GameState::kWaitingForPick);
}

// **配置の進み具合は消さない。** 正本は VR 側の box_count なので、
// こちらだけ巻き戻すと既に置いたスロットへもう一度置きに行くことになる
TEST(GameStateMachine, ResetKeepsPlacementProgress)
{
  GameStateMachine machine(makeConfig());
  runOneCycle(machine, 1);
  ASSERT_EQ(machine.currentSlotId(), 1);  // 1個目を消化済み

  machine.requestInit();
  machine.consumePendingGoal();
  machine.onGoalReached(5.0);
  ASSERT_EQ(machine.state(), GameState::kWaitingForPick);

  // キューも消化済み個数もそのまま (次に置くのは2個目のスロット)
  EXPECT_EQ(machine.currentSlotId(), 1);
  EXPECT_FALSE(machine.hasQueuedSlot());
}

// 状態名の文字列は /catchrobo/game/state の契約そのもの (VR側が色分けに使う)
TEST(GameStateMachine, InitStateNameRoundTrips)
{
  EXPECT_EQ(sharmech_core::toString(GameState::kInit), "INIT");
  const auto parsed = sharmech_core::gameStateFromString("INIT");
  ASSERT_TRUE(parsed.has_value());
  EXPECT_EQ(*parsed, GameState::kInit);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
