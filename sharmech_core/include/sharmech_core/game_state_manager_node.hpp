#ifndef SHARMECH_CORE__GAME_STATE_MANAGER_NODE_HPP_
#define SHARMECH_CORE__GAME_STATE_MANAGER_NODE_HPP_

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <sharmech_msgs/msg/cartesian_command.hpp>
#include <sharmech_msgs/msg/motion_status.hpp>
#include <sharmech_msgs/msg/workspace_clamp.hpp>

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
// Sub: /catchrobo/game/box_count      (std_msgs/Int32) VRの指定箱にワークを離した通算個数。
//      置きに行くべきスロット座標のキューになる (count-1 が最新スロットIDの正本)
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
//      どの状態からでも INIT へ入り、init_pose へ戻ってから WAITING_FOR_PICK に復帰する
class GameStateManagerNode : public rclcpp::Node
{
public:
  explicit GameStateManagerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void onPickRequest(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void onBoxCount(const std_msgs::msg::Int32::SharedPtr msg);
  void onArmStatus(const sharmech_msgs::msg::MotionStatus::SharedPtr msg);
  // デバッグ用。任意のステートへ強制遷移する (目標姿勢は配信しない)
  void onChangeStateRequest(const std_msgs::msg::String::SharedPtr msg);
  // VRが使えない場合の脱出ハッチ。どの状態からでも自由操作(手動ジョグのみ)へ
  // トグルする。詳細は GameStateMachine::toggleManualControl() のコメント参照
  void onToggleManualControl(const std_msgs::msg::Empty::SharedPtr msg);
  // 状態のリセット要求。どの状態からでも INIT へ入り、初期位置へのゴールを1本出す
  // (詳細は GameStateMachine::requestInit() のコメント参照)
  void onResetRequest(const std_msgs::msg::Empty::SharedPtr msg);
  // 操縦者の確定 (微調整の完了)。ADJUSTING_PICK / ADJUSTING_PLACE でのみ効く
  void onConfirm(const std_msgs::msg::Empty::SharedPtr msg);
  // motion_generator の現在の目標姿勢。微調整でジョグした結果を追うために購読する
  void onCommandCartesian(const sharmech_msgs::msg::CartesianCommand::SharedPtr msg);
  void onTimer();

  void publishPendingOutputs();
  void publishState();

  // slot_x_<color_suffix> / slot_y_<color_suffix> / slot_z_<color_suffix>
  // パラメータ (等長の配列) からスロット姿勢の一覧を組み立てる
  std::vector<CartesianState> loadSlots(const std::string & color_suffix);

  // init_pose パラメータ ([x, y, z, pitch, yaw]) を読む。
  // 空なら既定値、要素数が5でなければ起動時に落とす (無言で別の場所へ動かさない)
  CartesianState loadInitPose();

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pick_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr box_count_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr confirm_sub_;
  rclcpp::Subscription<sharmech_msgs::msg::CartesianCommand>::SharedPtr command_cartesian_sub_;
  rclcpp::Subscription<sharmech_msgs::msg::MotionStatus>::SharedPtr status_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr gripper_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr orient_vertical_pub_;
  rclcpp::Publisher<sharmech_msgs::msg::WorkspaceClamp>::SharedPtr workspace_clamp_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr change_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr toggle_manual_control_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr reset_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::unique_ptr<GameStateMachine> machine_;
  // /catchrobo/arm/status は status_rate で常時流れてくるため、
  // last_result が変化した瞬間だけをイベントとして扱うための直近値
  uint8_t prev_last_result_{sharmech_msgs::msg::MotionStatus::RESULT_NONE};
  // 直近の box_count。+1 以外の変化 (飛び・減少・0リセット) を警告するためだけに持つ。
  // 追従自体は GameStateMachine::onBoxCount が受け取った値をそのまま正本として行う
  int prev_box_count_{0};
};

}  // namespace sharmech_core

#endif  // SHARMECH_CORE__GAME_STATE_MANAGER_NODE_HPP_
