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
//   kInit           初期位置 (Config::init_pose) へ動かしている最中。**起動時
//                   (Config::init_on_startup)・/catchrobo/game/reset・MCU フィードバックの
//                   途絶** で、どの状態からでもここへ入る (グリッパは開・縦は解除・
//                   作業領域クランプはデフォルトへ)。motion_generator_node の動作許可
//                   (enable) が出ていれば初期位置へ直線 1 本のゴールを出し、まだなら
//                   立ち上がりを待って出す。到達で kWaitingForPick へ、MANUAL_CONTROL に
//                   入ると抜ける (requestInit() / onMotionEnabled() のコメント参照)
//   kWaitingForPick 次に運ぶワークの選択待ち。VR からの pick_request を受理する
//   kApproaching    ワークの真上まで approach_clearance_z のまま**水平移動**する
//                   (グリッパは開)。斜めに降りながら近づくと、200mmピッチで並んだ
//                   隣のワークを薙ぎ払うため、高さを変えずに移動する
//   kApproachDescend ワークの真上から**垂直に降下**して掴む位置へ着ける
//   kGrasping       到達後グリッパを閉じ、grasp_dwell_sec だけ待つ
//   kTransportLift  掴んだ位置で transport_clearance_z まで**垂直に上昇**する。
//                   斜めに持ち上げると、缶を引きずったまま横へ動くことになる
//   kTransporting   キュー先頭のスロットの上空まで transport_clearance_z のまま
//                   **水平移動**する。到達したらそのまま kOrienting へ進む
//   kOrienting      スロット上空で**静止したまま**、横倒しのワークを縦にする
//                   (orient_vertical=true)。orient_dwell_sec だけ待ってから降下する。
//                   降下と同時に回すと回転が間に合わず缶が斜めのまま箱に入るため、
//                   動かない時間をここで確保する (enterOrienting() のコメント参照)
//   kPlacing        スロット上空 (transport_clearance_z) で**降下せず**離す段へ進む。
//                   ゴールは xy = スロット、z = 運搬高さのまま (2026-09-11 ユーザー決定:
//                   箱へ下げる動作は行わない。缶はこの高さから落とす)
//   kRetracting     グリッパを開き、同じxyでretract_clearance_zまで直線1本で退避。
//                   作業領域クランプはデフォルトに戻す。**縦のまま抜く**
//                   (箱の中でグリッパを回さないため。横へ戻すのは次の kApproaching)
//   kComplete       24箇所すべて配置完了
//   kManualControl  VRが使えない場合の脱出ハッチ。DualSenseの特定ボタン同時押しで
//                   どの状態からでも入れ、自動シーケンスを完全に止めて
//                   ジョグ操作(cmd_twist)だけで試合を進められるようにする。
//                   詳細は toggleManualControl() のコメント参照
// 値は /catchrobo/game/state の文字列が正本なので、数値の並びに意味は無い
// (kOrienting は後から追加したため末尾に置いてある)
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
  kOrienting = 8,
  kInit = 9,
  kApproachDescend = 10,
  kTransportLift = 11,
  kAdjustingPick = 12,
  kAdjustingPlace = 13,
};

inline std::string toString(GameState state)
{
  switch (state) {
    case GameState::kWaitingForPick: return "WAITING_FOR_PICK";
    case GameState::kApproaching: return "APPROACHING";
    case GameState::kApproachDescend: return "APPROACH_DESCEND";
    case GameState::kAdjustingPick: return "ADJUSTING_PICK";
    case GameState::kAdjustingPlace: return "ADJUSTING_PLACE";
    case GameState::kGrasping: return "GRASPING";
    case GameState::kTransportLift: return "TRANSPORT_LIFT";
    case GameState::kTransporting: return "TRANSPORTING";
    case GameState::kOrienting: return "ORIENTING";
    case GameState::kPlacing: return "PLACING";
    case GameState::kRetracting: return "RETRACTING";
    case GameState::kComplete: return "COMPLETE";
    case GameState::kManualControl: return "MANUAL_CONTROL";
    case GameState::kInit: return "INIT";
  }
  return "UNKNOWN";
}

// toString の逆変換。未知の名前なら nullopt。
// デバッグ用の状態強制遷移 (/catchrobo/debug/change_state) で使う
inline std::optional<GameState> gameStateFromString(const std::string & name)
{
  for (const auto state : {
      GameState::kWaitingForPick, GameState::kApproaching,
      GameState::kApproachDescend, GameState::kAdjustingPick,
      GameState::kAdjustingPlace, GameState::kGrasping, GameState::kTransportLift,
      GameState::kTransporting, GameState::kOrienting, GameState::kPlacing,
      GameState::kRetracting, GameState::kComplete, GameState::kManualControl,
      GameState::kInit
    })
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
    double slot_clamp_margin_m{0.03};    // ORIENTING〜RETRACTING中の作業領域クランプの片側マージン
    // ワーク上空まで水平移動するときのZ [m] (グリッパは空)。**retract_clearance_z と
    // 同じ値にしておくこと。** 揃っていれば RETRACTING の到達高さのまま
    // APPROACHING に入るので、接近が完全な水平移動になる (揃っていないとその差分
    // だけ斜めになる)
    double approach_clearance_z{0.20};
    double transport_clearance_z{0.20};  // TRANSPORT_LIFT/TRANSPORTING中のZ [m] (缶を保持)
    double retract_clearance_z{0.20};    // 設置後に上げるZ [m] (箱に当たらない高さ)
    // 掴みに降りる先の絶対 Z [m]。pick_request の z は使わず常にこれで上書きする
    // (VR は缶の底面 z=0 を送ってくる。2026-09-11 ユーザー指示)
    double pick_z{0.20};
    double grasp_dwell_sec{0.3};         // GRASPING状態での待機時間 [s]
    // ORIENTING状態での待機時間 [s]。ピッチ機構が横→縦を回し切るのに要する時間。
    // MCUがピッチの実状態を返さないため、grasp_dwell_sec と同じく固定時間待ちの
    // 暫定実装 (実フィードバックが使えるようになったら実確認へ置き換える)
    double orient_dwell_sec{0.5};
    // true なら「掴む直前」「離す直前」で止まり、操縦者の確定 (onConfirm) を待つ。
    // false なら止まらずそのまま掴む/離す (完全自動)。
    // 実機で位置合わせの精度が出るまでは true を推奨
    bool require_manual_confirm{true};
    // 初期位置 (kInit の行き先)。ベース座標系の直交座標。正本は robot_geometry.yaml の
    // init_pose (極座標 r/θ/z、赤・青別) で、game_state_manager_node が直交座標へ直して渡す。
    // pitch/yaw は 0。**そこまでは直線 1 本で動く** (L 字にはしない。起動時は人間が
    // おおよその初期位置に置いてから電源を入れる運用でカバーする)
    CartesianState init_pose{};
    // true なら起動直後に kInit から始まり、動作許可が出た時点で初期位置へ動く。
    // false なら kWaitingForPick から始まる (reset / 途絶による kInit は使える)
    bool init_on_startup{true};
    // kInit で動作許可が出てから実際に初期位置へのゴールを出すまでの待ち時間 [s]
    // (ユーザー指示 2026-09-11。同期直後に即動き出さず、一呼吸置く)
    double init_delay_sec{3.0};
  };

  explicit GameStateMachine(Config config)
  : config_(std::move(config))
  {
    if (config_.init_on_startup) {state_ = GameState::kInit;}
    // ゴールはまだ出さない。motion_generator_node が MCU の実姿勢へ同期して
    // 動作許可を出したとき (onMotionEnabled の立ち上がり) に出す
  }

  GameState state() const {return state_;}

  const Config & config() const {return config_;}

  // 実行中の設定差し替え (ros2 param set からの現場合わせ用)。
  // **状態 (state_ / order_index_ / 保持中のワーク等) は保持したまま設定だけ入れ替える。**
  // 当日フィールドに合わなかったときに、シーケンスを最初からやり直さずに
  // スロット座標や高さだけを直せるようにするためのもの。
  //
  // 次にゴールを出す状態へ進んだ時点から新しい値が効く (すでに publish 済みの
  // ゴールは追いかけて書き換えない。動作中に目標が飛ぶのを避けるため)。
  // placement_order の長さが縮んで order_index_ が範囲外になる場合だけは、
  // 「全部置き終わった」扱いに丸めて範囲外参照を防ぐ
  void setConfig(Config config)
  {
    config_ = std::move(config);
    if (order_index_ > config_.placement_order.size()) {
      order_index_ = config_.placement_order.size();
    }
  }

  // 現在(または直近)扱っているスロットID。全配置完了後は nullopt
  std::optional<int> currentSlotId() const
  {
    if (order_index_ >= config_.placement_order.size()) {return std::nullopt;}
    return config_.placement_order[order_index_];
  }

  // --- イベント: 対応する状態でないときは無視する ---

  // 状態のリセット要求 (/catchrobo/game/reset)。**どの状態からでも受け付ける**
  // (kManualControl・kComplete を含む)。試合中に手順が崩れたときの立て直しや、
  // 練習のやり直しのために、VR / PS4 のどちらからでも押せる1つの出口として置く。
  //
  // 「リセット」は**アームを初期位置 (Config::init_pose) へ戻すところまで**を指す。
  // 動作許可が出ていれば初期位置へのゴールを 1 本出し、まだなら onMotionEnabled の
  // 立ち上がりで出す。到達 (last_result = SUCCEEDED) で kWaitingForPick へ戻る
  // (onGoalReached)。却下・中断されたときは他の自動シーケンスと同じ扱いで
  // kWaitingForPick へ落ちる (onGoalRejectedOrAborted。理由はノード側が警告ログに出す)。
  //
  // MCU フィードバックの途絶 (motion_generator_node が動作許可を落とす) でも
  // ノードがこれを呼ぶ (onMotionEnabled の立ち下がり)。ワークを保持していても
  // グリッパは開ける (ユーザー決定 2026-09-11)
  //
  // **配置の進み具合 (order_index_ / authorized_count_) は消さない。**
  // 消してしまうと、VR側が持っている通算カウント (box_count) と食い違い、
  // 次に届いた box_count で既に置いたスロットへもう一度置きに行くことになる。
  // 「何個目まで置いたか」の正本はあくまで VR の box_count 側にある
  // (docs/game_state_manager_node.md の「box_count のキュー」参照)。
  void requestInit(double now_sec)
  {
    state_ = GameState::kInit;
    pending_goal_.reset();
    init_goal_sent_ = false;
    init_goal_due_sec_.reset();
    if (motion_enabled_) {scheduleInitGoal(now_sec);}
    pending_gripper_ = false;          // 掴んだままにしない
    pending_orient_vertical_ = false;  // 縦にしていたら横へ戻す
    WorkspaceClampCommand reset_clamp;
    reset_clamp.reset = true;          // PLACING中の絞り込みが残っていても解除する
    pending_clamp_ = reset_clamp;
    // **自由操作中に押されたら、そこから引き出して初期位置へ戻す。**
    // リセットは VR 側のボタンなので、押せている時点で VR は生きている
    // (kManualControl は「VRが使えないときの脱出ハッチ」)。
    // 退避先 (pre_manual_state_) はここでは触らない —— 次に自由操作へ
    // 入るときに、そのときの状態で上書きされるため
  }

  // motion_generator_node の動作許可 (/catchrobo/command/cartesian の enable)。
  // MCU の実姿勢へ同期し終えると false → true、MCU 未初期化 (bit3) やフィードバックの
  // 途絶で true → false になる。
  //   立ち上がり: kInit で待っていれば、init_delay_sec 後に初期位置へのゴールを出す
  //               (起動時・reset 直後・途絶からの復帰。実際に出すのは tick)。
  //               それ以外の状態では何もしない
  //   立ち下がり: どの状態からでも kInit へ (途絶 → 強制 INIT。ユーザー決定 2026-09-11)。
  //               ゴールは復帰後の立ち上がりで出る
  void onMotionEnabled(bool enabled, double now_sec)
  {
    const bool was_enabled = motion_enabled_;
    motion_enabled_ = enabled;
    if (enabled && !was_enabled && state_ == GameState::kInit) {
      scheduleInitGoal(now_sec);
    } else if (!enabled && was_enabled) {
      requestInit(now_sec);
    }
  }

  bool motionEnabled() const {return motion_enabled_;}

  // ワークへは「水平移動 → 垂直降下」の2段で近づく (斜めに降りない)。
  // ここで出すのは1段目、ワークの**真上**までのゴール
  void onPickPoseReceived(const CartesianState & pose)
  {
    if (state_ != GameState::kWaitingForPick) {return;}
    pick_pose_ = pose;
    pick_pose_.z = config_.pick_z;    // 要求の z は信用しない (上記 Config::pick_z)
    state_ = GameState::kApproaching;
    CartesianState above = pose;
    above.z = config_.approach_clearance_z;
    pending_goal_ = above;
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
  // **ただし宛先を確定済みのサイクル (kTransportLift/kTransporting/kOrienting/
  // kPlacing/kRetracting) は中断しない。** 途中でスロットが差し替わると、既に
  // publish 済みのゴールと退避先の xy がずれて、ワークを保持したまま別の箱の上へ
  // 動くことになるため。減少がこの5状態中に届いた場合は、そのサイクルを
  // 最後まで終えてから効く
  void onBoxCount(int count)
  {
    const std::size_t capacity = config_.placement_order.size();
    authorized_count_ = count <= 0 ? 0 :
      std::min(static_cast<std::size_t>(count), capacity);

    const bool destination_committed =
      state_ == GameState::kTransportLift || state_ == GameState::kTransporting ||
      state_ == GameState::kOrienting ||
      state_ == GameState::kPlacing || state_ == GameState::kRetracting;
    if (destination_committed) {return;}

    if (order_index_ > authorized_count_) {order_index_ = authorized_count_;}
    // 巻き戻しでスロットが復活したら、終了状態から待機へ戻す
    if (state_ == GameState::kComplete && order_index_ < capacity) {
      state_ = GameState::kWaitingForPick;
    }
  }

  // 操縦者の「これでよい」。/catchrobo/game/confirm (PS4の確定ボタン、VRのサムズアップ)
  // を受けてノードが呼ぶ。**微調整待ちの2状態でのみ有効**で、それ以外では無視する
  // (自動シーケンス中に誤って押しても何も起きない)
  void onConfirm(double now_sec)
  {
    if (state_ == GameState::kAdjustingPick) {
      enterGrasping(now_sec);
    } else if (state_ == GameState::kAdjustingPlace) {
      enterRetracting();
    }
  }

  // motion_generator が 100Hz で出している現在の目標姿勢 (/catchrobo/command/cartesian)。
  // **微調整で操縦者がジョグした結果を含む**ため、掴んだ/離した実際の位置はこれで分かる。
  // 直後の垂直移動 (kTransportLift / kRetracting) の xy にこれを使うことで、
  // 微調整した分だけ横にずれた斜め移動になるのを防ぐ
  void onCurrentPose(const CartesianState & pose) {current_pose_ = pose;}

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
        // ワークの真上に着いた。ここから垂直に pick_z まで降ろす
        state_ = GameState::kApproachDescend;
        pending_goal_ = pick_pose_;
        break;
      case GameState::kApproachDescend:
        // 掴む位置へ着いた。require_manual_confirm なら、ここで止まって
        // 操縦者が微調整して確定 (onConfirm) するのを待つ
        if (config_.require_manual_confirm) {
          state_ = GameState::kAdjustingPick;
        } else {
          enterGrasping(now_sec);
        }
        break;
      case GameState::kTransportLift:
        // 掴んだ位置での上昇が終わった。ここから水平にスロット上空へ運ぶ
        enterTransporting();
        break;
      case GameState::kPlacing:
        // スロットへ降ろし切った。require_manual_confirm なら、離す前に
        // 止まって操縦者の確定を待つ
        if (config_.require_manual_confirm) {
          state_ = GameState::kAdjustingPlace;
        } else {
          enterRetracting();
        }
        break;
      case GameState::kTransporting:
        // スロットの上空へ着いた。降下の前に、その場で縦にする時間を取る。
        // 「置け」の指示は box_count が既に兼ねている (onBoxCount 参照)
        enterOrienting(now_sec);
        break;
      case GameState::kRetracting:
        advanceSlot();
        break;
      case GameState::kInit:
        // 初期位置へ戻り切った。ここから普通に pick_request を受けられる。
        // まだゴールを出していない (動作許可待ち) のに届いた到達は他人のもの
        // (ノード再起動時に latched で届く古い status 等) なので無視する
        if (!init_goal_sent_) {break;}
        init_goal_sent_ = false;
        state_ = GameState::kWaitingForPick;
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
    // kInit でゴールを出す前 (動作許可待ち) の却下・中断は自分のものではない
    // (途絶で中断された直前のゴールの ABORTED が enable の立ち下がりの後に届く等)
    if (state_ == GameState::kInit && !init_goal_sent_) {return;}
    init_goal_sent_ = false;
    if (state_ != GameState::kWaitingForPick && state_ != GameState::kComplete &&
      state_ != GameState::kManualControl)
    {
      state_ = GameState::kWaitingForPick;
    }
  }

  // 経過時間 (grasp dwell / orient dwell) と box_count キューによる遷移を進める。
  // 定期的に (control loop相当で) 呼ぶ。grasp dwell が経過していても**キューが空の
  // 間は kGrasping のまま待つ** (掴んだ位置で宛先の指示待ちになる)
  void tick(double now_sec)
  {
    // kInit: 動作許可が出てから init_delay_sec 待って初期位置へのゴールを出す
    if (state_ == GameState::kInit && init_goal_due_sec_ && now_sec >= *init_goal_due_sec_) {
      init_goal_due_sec_.reset();
      sendInitGoal();
      return;
    }
    if (state_ == GameState::kGrasping && hasQueuedSlot() &&
      now_sec - grasp_start_sec_ >= config_.grasp_dwell_sec)
    {
      // 斜めに持ち上げると缶を引きずるので、まず掴んだ場所で真上に上げる。
      // 横移動 (kTransporting) はそのあと
      state_ = GameState::kTransportLift;
      CartesianState goal = verticalFrom(pick_pose_);
      goal.z = config_.transport_clearance_z;
      pending_goal_ = goal;
      return;
    }
    // 縦にし終わったはずの時間が経ったら降下に入る。
    // ORIENTING 中はゴールを出していないので、アームはスロット上空で静止したまま
    if (state_ == GameState::kOrienting &&
      now_sec - orient_start_sec_ >= config_.orient_dwell_sec)
    {
      enterPlacing();
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
    init_goal_sent_ = false;   // forceState(kInit) では動かない (動かすのは reset)
    init_goal_due_sec_.reset();
    pending_gripper_.reset();
    pending_orient_vertical_.reset();
    pending_clamp_.reset();
    // GRASPING / ORIENTING は経過時間で次へ進むので、基準時刻を入れ直す
    if (state == GameState::kGrasping) {grasp_start_sec_ = now_sec;}
    if (state == GameState::kOrienting) {orient_start_sec_ = now_sec;}
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
      // kInit から自由操作に入っていた場合は kInit へ戻さず待機へ (自由操作で動かした
      // 後に、途中だった初期位置への移動を再開しない)。自由操作は INIT の出口の 1 つ
      state_ = pre_manual_state_ == GameState::kInit ?
        GameState::kWaitingForPick : pre_manual_state_;
    } else {
      pre_manual_state_ = state_;
      state_ = GameState::kManualControl;
    }
    pending_goal_.reset();
    init_goal_sent_ = false;
    init_goal_due_sec_.reset();
    pending_gripper_.reset();
    pending_orient_vertical_.reset();
    WorkspaceClampCommand reset_clamp;
    reset_clamp.reset = true;
    pending_clamp_ = reset_clamp;
    // GRASPING / ORIENTING に戻った場合、経過時間で次へ進むので基準時刻を入れ直す
    if (state_ == GameState::kGrasping) {grasp_start_sec_ = now_sec;}
    if (state_ == GameState::kOrienting) {orient_start_sec_ = now_sec;}
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
  // init_delay_sec 後に初期位置へのゴールを出すよう予約する (tick が出す)
  void scheduleInitGoal(double now_sec)
  {
    init_goal_due_sec_ = now_sec + config_.init_delay_sec;
  }

  // 初期位置へのゴール (直線 1 本) を出す。姿勢は 0 にそろえる
  void sendInitGoal()
  {
    CartesianState goal = config_.init_pose;
    goal.pitch = 0.0;
    goal.yaw = 0.0;
    pending_goal_ = goal;
    init_goal_sent_ = true;
  }

  void enterGrasping(double now_sec)
  {
    state_ = GameState::kGrasping;
    grasp_start_sec_ = now_sec;
    pending_gripper_ = true;          // 閉じる
  }

  // 垂直移動の起点にする xy。微調整で操縦者がずらしていれば実際の位置を、
  // 現在位置が未受信なら nominal (引数) をそのまま使う。
  // これを使わずに nominal へ戻すと、微調整した分だけ横にずれた斜め移動になる
  CartesianState verticalFrom(const CartesianState & nominal) const
  {
    if (!current_pose_) {return nominal;}
    CartesianState from = nominal;
    from.x = current_pose_->x;
    from.y = current_pose_->y;
    return from;
  }

  // 掴んだ位置での上昇が終わった状態から呼ぶ。transport_clearance_z を保ったまま
  // スロットの真上まで水平に運ぶ (高さを変えないので箱・設置済みの缶の上を
  // 一定のクリアランスで通過できる)
  void enterTransporting()
  {
    state_ = GameState::kTransporting;
    const auto id = currentSlotId();
    CartesianState goal = id ? config_.slots.at(*id) : pick_pose_;
    goal.z = config_.transport_clearance_z;
    pending_goal_ = goal;
  }

  // スロット上空に着いた状態から呼ぶ。**ゴールは発行しない**ので、アームは
  // TRANSPORTING の到達点で静止したまま縦にする動作だけを行う。
  //
  // 降下 (kPlacing) と同時に縦にすると、ピッチ機構の回転が終わる前に箱へ入り、
  // 缶が斜めのまま狭いスロット (箱の内寸 138×255mm に6箇所) へ突っ込む。動かない時間を
  // ここで確保することで、kPlacing は「既に縦になった缶をまっすぐ降ろすだけ」になる。
  //
  // 作業領域クランプもここで絞る。降下前から効かせておけば、ORIENTING 中に
  // 人間の手動ゴールが紛れ込んでも遠方へは動けない
  void enterOrienting(double now_sec)
  {
    state_ = GameState::kOrienting;
    orient_start_sec_ = now_sec;
    pending_orient_vertical_ = true;  // 横倒しのワークを縦にする指示

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
  }

  // 既に kOrienting で縦になっている前提。**箱へは降下しない** (2026-09-11 ユーザー決定)。
  // ゴールは TRANSPORTING の到達点と同じ高さなので実質その場で止まり、到達で離す段へ進む
  // (クランプと orient_vertical は enterOrienting() で済ませてある)
  void enterPlacing()
  {
    state_ = GameState::kPlacing;
    const auto id = currentSlotId();
    if (!id) {return;}  // placement_order を使い切っている場合の安全側フォールバック
    CartesianState goal = config_.slots.at(*id);
    goal.z = config_.transport_clearance_z;
    pending_goal_ = goal;
  }

  void enterRetracting()
  {
    state_ = GameState::kRetracting;
    const auto id = currentSlotId();
    CartesianState goal = verticalFrom(id ? config_.slots.at(*id) : CartesianState{});
    goal.z = config_.retract_clearance_z;
    pending_goal_ = goal;
    pending_gripper_ = false;         // 置いたので開く
    // **縦のまま抜く。** ここで横に戻すと箱の中でグリッパを回すことになり、
    // 壁に当たりうる。横へ戻すのは次の kApproaching (空中の長い移動なので安全)

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
  double orient_start_sec_{0.0};   // kOrienting に入った時刻。orient dwell 判定に使う
  // motion_generator の現在の目標姿勢 (微調整のジョグを含む)。未受信なら nullopt
  std::optional<CartesianState> current_pose_;

  // motion_generator_node の動作許可 (未受信なら false)
  bool motion_enabled_{false};
  // kInit で初期位置へのゴールを出したか。出す前に届く到達・却下は無視する
  bool init_goal_sent_{false};
  // kInit でゴールを出す予定時刻 [s] (動作許可が出た時刻 + init_delay_sec)。予約が無ければ nullopt
  std::optional<double> init_goal_due_sec_;

  std::optional<CartesianState> pending_goal_;
  std::optional<bool> pending_gripper_;
  std::optional<bool> pending_orient_vertical_;
  std::optional<WorkspaceClampCommand> pending_clamp_;
};

}  // namespace sharmech_core

#endif  // SHARMECH_CORE__UTILITY__GAME_STATE_MACHINE_HPP_
