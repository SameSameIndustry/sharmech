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
//   kTransporting   キュー先頭のスロットのxy・transport_clearance_zまで運搬中。
//                   到達したらそのまま kPlacing へ進む
//   kPlacing        作業領域クランプをスロット周辺に一時的に絞り、スロット姿勢まで
//                   直線1本で降下。orient_vertical を true にして MCU に伝える
//   kRetracting     グリッパを開き、同じxyでretract_clearance_zまで直線1本で退避。
//                   作業領域クランプはデフォルトに戻す
//   kComplete       24箇所すべて配置完了
//   kManualControl  VRが使えない場合の脱出ハッチ。DualSenseの特定ボタン同時押しで
//                   どの状態からでも入れ、自動シーケンスを完全に止めて
//                   ジョグ操作(cmd_twist)だけで試合を進められるようにする。
//                   詳細は toggleManualControl() のコメント参照
enum class GameState : uint8_t
{
  kWaitingForPick = 0,
  kApproaching = 1,
  kGrasping = 2,
  kTransporting = 3,
  kPlacing = 4,
  kRetracting = 5,
  kComplete = 6,
  kManualControl = 7,
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
    case GameState::kManualControl: return "MANUAL_CONTROL";
  }
  return "UNKNOWN";
}

// toString の逆変換。未知の名前なら nullopt。
// デバッグ用の状態強制遷移 (/catchrobo/debug/change_state) で使う
inline std::optional<GameState> gameStateFromString(const std::string & name)
{
  for (const auto state : {
      GameState::kWaitingForPick, GameState::kApproaching, GameState::kGrasping,
      GameState::kTransporting, GameState::kPlacing, GameState::kRetracting,
      GameState::kComplete, GameState::kManualControl})
  {
    if (toString(state) == name) {return state;}
  }
  return std::nullopt;
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
// **どのスロットへ何個目を置くかは VR の box_count が決める。** VRクライアントは
// 仮想フィールドの指定箱にワークを離すたびに「通算何個目か」を
// /catchrobo/game/box_count で送ってくる。本クラスはそれを「置きに行くべき
// スロット座標のキュー」として保持し (onBoxCount)、キューが空でない間だけ
// 運搬→設置→退避を自動で進める。**count-1 が常に最新スロットIDの正本**
// (キュー長 = box_count - 消化済み個数)。
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

  // VRクライアントが指定箱にワークを離すたびに送ってくる「通算何個目か」(1始まり)。
  // 受け取った値そのものを正本として扱い、「置きに行くべきスロット」のキューを
  // 更新する (キュー長 = count - 消化済み個数)。**このイベント自体は動き出しの
  // 契機ではない**。実際に動くのは kGrasping の dwell 経過後、キューが空でない
  // ときに kTransporting へ進むところから。
  //
  // 飛び・減少・0リセットも一律「その値が正本」として追従する
  // (VRの再接続やフィールド再設置でカウントが0に戻る実装になっているため)。
  // **ただし宛先を確定済みのサイクル (kTransporting/kPlacing/kRetracting) は
  // 中断しない。** 途中でスロットが差し替わると、既に publish 済みのゴールと
  // 退避先の xy がずれて、ワークを保持したまま別の箱の上へ動くことになるため。
  // 減少がこの3状態中に届いた場合は、そのサイクルを最後まで終えてから効く
  void onBoxCount(int count)
  {
    const std::size_t capacity = config_.placement_order.size();
    authorized_count_ = count <= 0 ? 0 :
      std::min(static_cast<std::size_t>(count), capacity);

    const bool destination_committed =
      state_ == GameState::kTransporting || state_ == GameState::kPlacing ||
      state_ == GameState::kRetracting;
    if (destination_committed) {return;}

    if (order_index_ > authorized_count_) {order_index_ = authorized_count_;}
    // 巻き戻しでスロットが復活したら、終了状態から待機へ戻す
    if (state_ == GameState::kComplete && order_index_ < capacity) {
      state_ = GameState::kWaitingForPick;
    }
  }

  // 置きに行くべきスロットがキューに残っているか (box_count が消化済み個数より先行しているか)
  bool hasQueuedSlot() const {return order_index_ < authorized_count_;}

  // placement_order の長さ。box_count の範囲チェック用にノードから読む
  std::size_t placementCount() const {return config_.placement_order.size();}

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
      case GameState::kTransporting:
        // スロットの上空へ着いたので、そのまま降下に入る。
        // 「置け」の指示は box_count が既に兼ねている (onBoxCount 参照)
        enterPlacing();
        break;
      case GameState::kRetracting:
        advanceSlot();
        break;
      default:
        break;
    }
  }

  // 却下・中断は安全側 (WAITING_FOR_PICK) に戻す。経由点なしの直線軌道である以上、
  // 中断されたら掴み直しになるが、これが最も単純で安全。
  // ただし kManualControl 中は対象外 (手動ジョグ中のゴール却下で自動シーケンスに
  // 引き戻されると、脱出ハッチとして機能しなくなるため)
  void onGoalRejectedOrAborted()
  {
    if (state_ != GameState::kWaitingForPick && state_ != GameState::kComplete &&
      state_ != GameState::kManualControl)
    {
      state_ = GameState::kWaitingForPick;
    }
  }

  // 経過時間 (grasp dwell) と box_count キューによる遷移を進める。
  // 定期的に (control loop相当で) 呼ぶ。dwell が経過していても**キューが空の間は
  // kGrasping のまま待つ** (掴んだ位置で宛先の指示待ちになる)
  void tick(double now_sec)
  {
    if (state_ == GameState::kGrasping && hasQueuedSlot() &&
      now_sec - grasp_start_sec_ >= config_.grasp_dwell_sec)
    {
      state_ = GameState::kTransporting;
      const auto id = currentSlotId();
      CartesianState goal = id ? config_.slots.at(*id) : pick_pose_;
      goal.z = config_.transport_clearance_z;
      pending_goal_ = goal;
    }
  }

  // デバッグ用に状態を直接書き換える (/catchrobo/debug/change_state)。
  //
  // 通常の遷移と違い、その状態の目標姿勢・グリッパ・作業領域クランプは
  // 一切セットしない。状態だけを見たいのに実機/シムが勝手に動き出すのを
  // 避けるため (保留中の指令が残っていれば破棄する)。
  // 動かしたい場合は遷移後に通常のイベント (pick_request 等) を送ること
  void forceState(GameState state, double now_sec)
  {
    state_ = state;
    pending_goal_.reset();
    pending_gripper_.reset();
    pending_orient_vertical_.reset();
    pending_clamp_.reset();
    // GRASPING は経過時間で TRANSPORTING に進むので、基準時刻を入れ直す
    if (state == GameState::kGrasping) {grasp_start_sec_ = now_sec;}
  }

  // VRが使えない場合の脱出ハッチ。DualSenseの特定ボタン同時押し(L1+R1+L3+R3)を
  // game_state_manager_node が検知して呼ぶ。**どの状態からでも呼べる** (kComplete
  // からも入れる。試合終了後の片付け等でも手動操作したいことがあるため)。
  //
  // kManualControl でなければ現在の状態を退避して kManualControl へ、
  // 既に kManualControl ならば退避しておいた状態へ戻る (トグル)。
  //
  // forceState と同様、目標姿勢・グリッパ等の副作用コマンドは一切出さない
  // (ジョグは cmd_twist 経由で常に効いており、このメソッドが動かす必要はない)。
  // **ただし作業領域クランプだけは必ずデフォルトへリセットする。** PLACING中の
  // 絞り込みが残ったままだと、脱出ハッチのはずがジョグ操作を妨げてしまうため。
  //
  // 既知の制限: 自由操作中に人間が手動で配置を進めても、スロットの消化
  // (order_index_) はステートマシンには反映されない (センサでオブジェクトの
  // 状態を検知していないため)。元の状態(kApproaching等)に戻ったとき、
  // 直前に自動シーケンスが把握していたスロット割付のまま再開される
  void toggleManualControl(double now_sec)
  {
    if (state_ == GameState::kManualControl) {
      state_ = pre_manual_state_;
    } else {
      pre_manual_state_ = state_;
      state_ = GameState::kManualControl;
    }
    pending_goal_.reset();
    pending_gripper_.reset();
    pending_orient_vertical_.reset();
    WorkspaceClampCommand reset_clamp;
    reset_clamp.reset = true;
    pending_clamp_ = reset_clamp;
    // GRASPING に戻った場合、経過時間で TRANSPORTING に進むので基準時刻を入れ直す
    if (state_ == GameState::kGrasping) {grasp_start_sec_ = now_sec;}
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
  GameState pre_manual_state_{GameState::kWaitingForPick};  // toggleManualControl の復帰先
  std::size_t order_index_{0};       // 消化済み個数 = 次に置くスロットのキュー先頭
  std::size_t authorized_count_{0};  // 直近の box_count (キュー末尾。count-1 が最新スロットID)
  CartesianState pick_pose_{};
  double grasp_start_sec_{0.0};

  std::optional<CartesianState> pending_goal_;
  std::optional<bool> pending_gripper_;
  std::optional<bool> pending_orient_vertical_;
  std::optional<WorkspaceClampCommand> pending_clamp_;
};

}  // namespace sharmech_core

#endif  // SHARMECH_CORE__UTILITY__GAME_STATE_MACHINE_HPP_
