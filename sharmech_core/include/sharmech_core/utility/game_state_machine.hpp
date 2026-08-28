#ifndef SHARMECH_CORE__UTILITY__GAME_STATE_MACHINE_HPP_
#define SHARMECH_CORE__UTILITY__GAME_STATE_MACHINE_HPP_

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "sharmech_core/utility/trapezoidal_trajectory.hpp"  // CartesianState

namespace sharmech_core
{

// ゲーム全体(掴む→運ぶ→置く→退避)の状態。game_state_manager_node が
// /catchrobo/game/state (std_msgs/String) としてそのまま配信する。
//
// 各状態での動作:
//   kWaitingForPick 次に運ぶワークの選択待ち。VR からの pick_request を受理する
//   kApproaching    選択したワークの姿勢へ直線1本で接近中 (グリッパは開)
//   kGrasping       到達後グリッパを閉じ、grasp_dwell_sec だけ待つ
//   kTransporting   現在のスロットのxy・transport_clearance_zまで運搬中。
//                   VR の place_request (「置け」指示) を待つ
//   kPlacing        作業領域クランプをスロット周辺に一時的に絞り、スロット姿勢まで
//                   直線1本で降下。orient_vertical を true にして MCU に伝える
//   kRetracting     グリッパを開き、同じxyでretract_clearance_zまで直線1本で退避。
//                   作業領域クランプはデフォルトに戻す
//   kComplete       24箇所すべて配置完了
enum class GameState : uint8_t
{
  kWaitingForPick = 0,
  kApproaching = 1,
  kGrasping = 2,
  kTransporting = 3,
  kPlacing = 4,
  kRetracting = 5,
  kComplete = 6,
};

inline std::string toString(GameState state)
{
  switch (state) {
    case GameState::kWaitingForPick: return "WAITING_FOR_PICK";
    case GameState::kApproaching: return "APPROACHING";
    case GameState::kGrasping: return "GRASPING";
    case GameState::kTransporting: return "TRANSPORTING";
    case GameState::kPlacing: return "PLACING";
    case GameState::kRetracting: return "RETRACTING";
    case GameState::kComplete: return "COMPLETE";
  }
  return "UNKNOWN";
}

// motion_generator_node へ送る作業領域クランプの上書き指令 (ROS 非依存。
// sharmech_msgs/WorkspaceClamp と1対1に対応する)
struct WorkspaceClampCommand
{
  bool reset{true};
  double x_min{0.0}, x_max{0.0};
  double y_min{0.0}, y_max{0.0};
  double z_min{0.0}, z_max{0.0};
};

// 24箇所のシューティングボックス配置を自動シーケンス実行するステートマシン。
// ROS に依存しない純粋ロジック (test_game_state_machine.cpp で検証)。
// game_state_manager_node はイベントをこのクラスに流し込み、状態遷移直後に
// 一度だけセットされる「保留中の指令」を読んで publish するだけでよい。
//
// 経由点を持つ軌道は作らない、という既存方針 (sharmech/README.md) に合わせ、
// 各状態のゴールは直線1本のみ (待避点や中間点を挟まない)。
//
// 仕様の正本: sharmech_core/docs/game_state_manager_node.md
class GameStateMachine
{
public:
  struct Config
  {
    std::vector<CartesianState> slots;   // インデックス = スロットID (0..N-1)
    std::vector<int> placement_order;    // 配置する順番のスロットID列 (長さ = N)
    double slot_clamp_margin_m{0.03};    // PLACING/RETRACTING中の作業領域クランプの片側マージン
    double transport_clearance_z{0.20};  // TRANSPORTING中に上げるZ [m]
    double retract_clearance_z{0.20};    // 設置後に上げるZ [m] (箱に当たらない高さ)
    double grasp_dwell_sec{0.3};         // GRASPING状態での待機時間 [s]
  };

  explicit GameStateMachine(Config config)
  : config_(std::move(config))
  {
  }

  GameState state() const {return state_;}

  // 現在(または直近)扱っているスロットID。全配置完了後は nullopt
  std::optional<int> currentSlotId() const
  {
    if (order_index_ >= config_.placement_order.size()) {return std::nullopt;}
    return config_.placement_order[order_index_];
  }

  // --- イベント: 対応する状態でないときは無視する ---

  void onPickPoseReceived(const CartesianState & pose)
  {
    if (state_ != GameState::kWaitingForPick) {return;}
    pick_pose_ = pose;
    state_ = GameState::kApproaching;
    pending_goal_ = pose;
    pending_gripper_ = false;         // 掴む前は開いている
    pending_orient_vertical_ = false;
  }

  void onPlaceRequested()
  {
    if (state_ != GameState::kTransporting) {return;}
    enterPlacing();
  }

  // motion_generator_node の /catchrobo/arm/status で現在のゴールが
  // 到達したことを検知したら呼ぶ
  void onGoalReached(double now_sec)
  {
    switch (state_) {
      case GameState::kApproaching:
        state_ = GameState::kGrasping;
        grasp_start_sec_ = now_sec;
        pending_gripper_ = true;      // 閉じる
        break;
      case GameState::kPlacing:
        enterRetracting();
        break;
      case GameState::kRetracting:
        advanceSlot();
        break;
      // kTransporting へのゴール到達自体は無視する。
      // 次に進むのは VR の onPlaceRequested を待ってから
      default:
        break;
    }
  }

  // 却下・中断は安全側 (WAITING_FOR_PICK) に戻す。経由点なしの直線軌道である以上、
  // 中断されたら掴み直しになるが、これが最も単純で安全
  void onGoalRejectedOrAborted()
  {
    if (state_ != GameState::kWaitingForPick && state_ != GameState::kComplete) {
      state_ = GameState::kWaitingForPick;
    }
  }

  // 経過時間依存の遷移 (grasp dwell) を進める。定期的に (control loop相当で) 呼ぶ
  void tick(double now_sec)
  {
    if (state_ == GameState::kGrasping &&
      now_sec - grasp_start_sec_ >= config_.grasp_dwell_sec)
    {
      state_ = GameState::kTransporting;
      const auto id = currentSlotId();
      CartesianState goal = id ? config_.slots.at(*id) : pick_pose_;
      goal.z = config_.transport_clearance_z;
      pending_goal_ = goal;
    }
  }

  // --- 保留中の指令。状態遷移直後にのみセットされる (edge-triggered) ---

  bool hasPendingGoal() const {return pending_goal_.has_value();}
  CartesianState consumePendingGoal()
  {
    const auto v = *pending_goal_;
    pending_goal_.reset();
    return v;
  }

  bool hasPendingGripper() const {return pending_gripper_.has_value();}
  bool consumePendingGripper()
  {
    const auto v = *pending_gripper_;
    pending_gripper_.reset();
    return v;
  }

  bool hasPendingOrientVertical() const {return pending_orient_vertical_.has_value();}
  bool consumePendingOrientVertical()
  {
    const auto v = *pending_orient_vertical_;
    pending_orient_vertical_.reset();
    return v;
  }

  bool hasPendingWorkspaceClamp() const {return pending_clamp_.has_value();}
  WorkspaceClampCommand consumePendingWorkspaceClamp()
  {
    const auto v = *pending_clamp_;
    pending_clamp_.reset();
    return v;
  }

private:
  void enterPlacing()
  {
    state_ = GameState::kPlacing;
    const auto id = currentSlotId();
    if (!id) {return;}  // placement_order を使い切っている場合の安全側フォールバック
    const CartesianState & slot = config_.slots.at(*id);

    WorkspaceClampCommand clamp;
    clamp.reset = false;
    clamp.x_min = slot.x - config_.slot_clamp_margin_m;
    clamp.x_max = slot.x + config_.slot_clamp_margin_m;
    clamp.y_min = slot.y - config_.slot_clamp_margin_m;
    clamp.y_max = slot.y + config_.slot_clamp_margin_m;
    // TRANSPORTING の到達点 (slot.x, slot.y, transport_clearance_z) から
    // スロット姿勢まで直線で降下するので、両端を含む範囲にする
    clamp.z_min = std::min(slot.z, config_.transport_clearance_z) - config_.slot_clamp_margin_m;
    clamp.z_max = std::max(slot.z, config_.transport_clearance_z) + config_.slot_clamp_margin_m;
    pending_clamp_ = clamp;

    pending_goal_ = slot;
    pending_orient_vertical_ = true;  // 横倒しのワークを縦にする指示
  }

  void enterRetracting()
  {
    state_ = GameState::kRetracting;
    const auto id = currentSlotId();
    CartesianState goal = id ? config_.slots.at(*id) : CartesianState{};
    goal.z = config_.retract_clearance_z;
    pending_goal_ = goal;
    pending_gripper_ = false;         // 置いたので開く
    pending_orient_vertical_ = false;

    WorkspaceClampCommand reset_clamp;
    reset_clamp.reset = true;
    pending_clamp_ = reset_clamp;
  }

  void advanceSlot()
  {
    ++order_index_;
    state_ = order_index_ >= config_.placement_order.size() ?
      GameState::kComplete : GameState::kWaitingForPick;
  }

  Config config_;
  GameState state_{GameState::kWaitingForPick};
  std::size_t order_index_{0};
  CartesianState pick_pose_{};
  double grasp_start_sec_{0.0};

  std::optional<CartesianState> pending_goal_;
  std::optional<bool> pending_gripper_;
  std::optional<bool> pending_orient_vertical_;
  std::optional<WorkspaceClampCommand> pending_clamp_;
};

}  // namespace sharmech_core

#endif  // SHARMECH_CORE__UTILITY__GAME_STATE_MACHINE_HPP_
