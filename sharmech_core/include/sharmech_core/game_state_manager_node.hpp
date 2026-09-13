#ifndef SHARMECH_CORE__GAME_STATE_MANAGER_NODE_HPP_
#define SHARMECH_CORE__GAME_STATE_MANAGER_NODE_HPP_

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>
#include <sharmech_msgs/msg/cartesian_command.hpp>
#include <sharmech_msgs/msg/motion_status.hpp>
#include <sharmech_msgs/msg/workspace_clamp.hpp>
#include <sharmech_msgs/msg/jog_limit.hpp>

#include "sharmech_core/utility/game_state_machine.hpp"

namespace sharmech_core
{

// 「掴む→運ぶ→置く→退避」の自動配置シーケンスと、ゲーム全体の状態を管理する
// 中核ノード。状態遷移の純粋ロジックは utility/game_state_machine.hpp
// (GameStateMachine) に切り出してあり、本ノードは ROS トピックとの橋渡しのみ行う。
//
// 自動シーケンスのゴールは既存の /catchrobo/arm/target_pose にそのまま publish する
// (専用チャンネルは新設しない)。VR/PS4と同じ「調停なし・早い者勝ち」の前提に乗るため、
// 自動シーケンス実行中はVR側UIが人間の手動ゴールを送らないことがクライアント側の
// 契約になる (この契約自体はVRクライアント側の実装で担保する。本ノードは
// PLACING/RETRACTING中だけ作業領域クランプを動的に縮小することで安全側の保険をかける)。
//
// 仕様の正本: sharmech_core/docs/game_state_manager_node.md
//
// Sub: /catchrobo/game/pick_request   (geometry_msgs/PoseStamped) VRで選択したワーク姿勢
// Sub: /catchrobo/arm/status          (sharmech_msgs/MotionStatus) ゴール到達/却下の検知
// Pub: /catchrobo/arm/target_pose     自動シーケンスのゴール
// Pub: /catchrobo/arm/gripper         自動シーケンスのグリッパ指令
// Pub: /catchrobo/arm/orient_vertical PLACING中のみ true
// Pub: /catchrobo/game/workspace_clamp PLACING/RETRACTING前後の作業領域クランプ上書き
// Pub: /catchrobo/game/state          現在のゲームステート (std_msgs/String, latched)
// Sub: /catchrobo/debug/change_state  デバッグ用のステート強制遷移 (std_msgs/String)
// Sub: /catchrobo/game/toggle_manual_control  DualSenseの特定ボタン同時押し
//      (L1+R1+L3+R3) で joy_teleop_node が publish する、自由操作の入/切トグル
// Sub: /catchrobo/game/reset          (std_msgs/Empty) 状態のリセット要求。
//      どの状態からでも INIT へ入り、初期位置 (init_pose_*) へ戻ってから WAITING_FOR_PICK に復帰する
// Sub: /catchrobo/game/finish         (std_msgs/Empty) 競技終了時の終了位置 (finish_pose_*) への
//      移動要求。reset と同じ導線でどの状態からでも FINISH へ入り、着いても FINISH に留まる
// Sub: /catchrobo/game/reset_progress (std_msgs/Empty) 配置の進み具合のリセット。
//      VR の「置き直す」から届く。**アームは動かさない** (次に置くスロットを先頭へ戻すだけ)
// Sub: /catchrobo/command/cartesian   現在の目標姿勢 (微調整の起点) と動作許可 enable。
//      enable の立ち上がりで INIT が初期位置へ動き出し、立ち下がり (MCU 未初期化・
//      フィードバック途絶) でどの状態からでも INIT へ入る
class GameStateManagerNode : public rclcpp::Node
{
public:
  explicit GameStateManagerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void onPickRequest(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void onArmStatus(const sharmech_msgs::msg::MotionStatus::SharedPtr msg);
  // デバッグ用。任意のステートへ強制遷移する (目標姿勢は配信しない)
  void onChangeStateRequest(const std_msgs::msg::String::SharedPtr msg);
  // VRが使えない場合の脱出ハッチ。どの状態からでも自由操作(手動ジョグのみ)へ
  // トグルする。詳細は GameStateMachine::toggleManualControl() のコメント参照
  void onToggleManualControl(const std_msgs::msg::Empty::SharedPtr msg);
  // 状態のリセット要求。どの状態からでも INIT へ入り、初期位置へのゴールを 1 本出す
  // (詳細は GameStateMachine::requestInit() のコメント参照)
  void onResetRequest(const std_msgs::msg::Empty::SharedPtr msg);
  // 競技終了時の終了位置への移動要求。reset と同じくどの状態からでも FINISH へ入り、
  // 終了位置へのゴールを 1 本出す (詳細は GameStateMachine::requestFinish() のコメント参照)
  void onFinishRequest(const std_msgs::msg::Empty::SharedPtr msg);
  // 配置の進み具合のリセット。VR の「置き直す」から届く。アームは動かさない
  // (詳細は GameStateMachine::resetProgress() のコメント参照)
  void onResetProgress(const std_msgs::msg::Empty::SharedPtr msg);
  // 操縦者の確定 (微調整の完了)。ADJUSTING_PICK / ADJUSTING_PLACE でのみ効く
  void onConfirm(const std_msgs::msg::Empty::SharedPtr msg);
  // motion_generator の現在の目標姿勢と動作許可 (enable)。姿勢は微調整でジョグした
  // 結果を追うため、enable は INIT の動き出し / 途絶時の INIT 突入の契機
  void onCommandCartesian(const sharmech_msgs::msg::CartesianCommand::SharedPtr msg);
  void onTimer();

  void publishPendingOutputs();
  void publishState();
  // 現在の状態にふさわしいジョグ速度上限を motion_generator_node へ伝える。
  // **変化した瞬間だけ publish する** (状態は 10Hz で回るので毎回送ると無駄)。
  // 状態を知っているのはROS2側だけ、という役割分担を保つための経路で、
  // これがあるおかげで操縦層 (VR/PS4) はゲーム状態を知らなくてよい
  void publishJogLimitIfChanged();
  // INIT の間だけ motion_generator_node の v_max_z を init_v_max_z に絞り、抜けたら
  // 0 (無効) へ戻す。専用トピックではなく ros2 param set 相当 (パラメータクライアント)
  // で行う (ユーザー判断 2026-09-12)。publishState() から毎回呼ばれ、送るべき値が
  // 変わったときだけ送る。相手のサービスがまだ無ければ次の tick で再試行
  void syncInitSpeedLimit();

  // slot_x_<color_suffix> / slot_y_<color_suffix> / slot_z_<color_suffix>
  // パラメータ (等長の配列) からスロット姿勢の一覧を組み立てる

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pick_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr confirm_sub_;
  rclcpp::Subscription<sharmech_msgs::msg::CartesianCommand>::SharedPtr command_cartesian_sub_;
  rclcpp::Subscription<sharmech_msgs::msg::MotionStatus>::SharedPtr status_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr gripper_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr orient_vertical_pub_;
  rclcpp::Publisher<sharmech_msgs::msg::WorkspaceClamp>::SharedPtr workspace_clamp_pub_;
  rclcpp::Publisher<sharmech_msgs::msg::JogLimit>::SharedPtr jog_limit_pub_;
  // motion_generator_node の v_max_z を実行時に書き換えるためのクライアント (INIT 専用)
  rclcpp::AsyncParametersClient::SharedPtr motion_params_client_;
  // 直近に motion_generator_node へ送った v_max_z。nullopt なら次の publishState() で必ず送る
  // (動作許可の立ち上がりで nullopt に戻し、motion_generator_node が再起動して
  // パラメータが既定に戻っていても入れ直す)
  std::optional<double> sent_v_max_z_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr change_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr toggle_manual_control_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr reset_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr finish_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr reset_progress_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // パラメータの宣言 (既定値のみ)。値の取得は buildConfig() で行う
  void declareParameters(const std::string & color_suffix);
  // 現在値 (overrides があればそちらを優先) から設定を組み立てる。
  // 不整合は std::invalid_argument を投げる
  GameStateMachine::Config buildConfig(const std::vector<rclcpp::Parameter> & overrides) const;
  // 箱の中心と格子ピッチからスロット座標を生成する (箱ごとにX列が外側・Y行が内側)
  static std::vector<CartesianState> generateSlots(
    const std::vector<double> & box_center_x, double box_center_y,
    int cols_x, int rows_y, double pitch_x, double pitch_y, double slot_z);
  // 生成した格子が箱に収まっているかを知らせる (はみ出し・缶同士の干渉を警告)
  void logSlotGeometry(
    const GameStateMachine::Config & config,
    const std::vector<rclcpp::Parameter> & overrides) const;
  // 実行中のパラメータ変更を検証して適用する (再起動なしの現場合わせ)
  rcl_interfaces::msg::SetParametersResult onSetParameters(
    const std::vector<rclcpp::Parameter> & parameters);

  std::string field_color_;
  // 状態ごとのジョグ方針。publishJogLimitIfChanged() が状態から決める
  struct JogPolicy
  {
    bool reset{false};    // 起動時の上限 (jog_v_max) へ戻す
    bool block{false};    // ジョグを完全に無効化する
    double v_max{0.0};    // reset/block でないときの上限 [m/s]
    bool operator==(const JogPolicy & o) const
    {
      return reset == o.reset && block == o.block && v_max == o.v_max;
    }
  };
  // 直近に publish した方針。初期値はどの実方針とも一致しないので必ず1回は送られる
  JogPolicy last_jog_policy_{false, false, -1.0};
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  std::unique_ptr<GameStateMachine> machine_;
  // /catchrobo/arm/status は status_rate で常時流れてくるため、
  // last_result が変化した瞬間だけをイベントとして扱うための直近値
  uint8_t prev_last_result_{sharmech_msgs::msg::MotionStatus::RESULT_NONE};
};

}  // namespace sharmech_core

#endif  // SHARMECH_CORE__GAME_STATE_MANAGER_NODE_HPP_
