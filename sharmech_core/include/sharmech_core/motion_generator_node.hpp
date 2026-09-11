#ifndef SHARMECH_CORE__MOTION_GENERATOR_NODE_HPP_
#define SHARMECH_CORE__MOTION_GENERATOR_NODE_HPP_

#include <optional>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <sharmech_msgs/msg/cartesian_command.hpp>
#include <sharmech_msgs/msg/jog_limit.hpp>
#include <sharmech_msgs/msg/mcu_status.hpp>
#include <sharmech_msgs/msg/motion_status.hpp>
#include <sharmech_msgs/msg/workspace_clamp.hpp>

#include "sharmech_core/utility/trapezoidal_trajectory.hpp"

namespace sharmech_core
{

// ロボット全体のモーションを管理する中核ノード
//
// ゴール指定 (軌道生成 → 経過時間サンプリング) とジョグ (レート制限 → 積分) を
// 1本の Cartesian ストリーム (位置 + 速度) に合流させて下流へ流す。
// 全ての操縦指令がこのノードを通る唯一の調停点。運動学は一切持たない。
//
// 仕様の正本: sharmech_core/docs/motion_generator_node.md
//
// Sub: /catchrobo/arm/target_pose   ゴール入力
// Sub: /catchrobo/arm/cmd_twist     ジョグ入力 (ベース座標系)
// Sub: /catchrobo/arm/gripper       グリッパ指令
// Sub: /catchrobo/arm/orient_vertical  「縦にする」指令 (game_state_manager_node の PLACING 用)
// Sub: /catchrobo/arm/cancel        ゴール中断
// Sub: /catchrobo/arm/current_pose  実姿勢 (起動時の同期・残距離計算に使用)
// Sub: /catchrobo/arm/mcu_status    MCU の status_flags (未初期化の間は同期しない)
// Sub: /catchrobo/game/workspace_clamp  作業領域クランプの動的上書き (game_state_manager_node)
// Sub: /catchrobo/game/jog_limit    ジョグ速度上限の動的上書き (同上)
// Pub: /catchrobo/command/cartesian 位置 + 速度ストリーム (control_rate)。
//                                   動作許可 (enable) も同乗 (実姿勢に同期するまで false)
// Pub: /catchrobo/command/gripper   調停後のグリッパ指令
// Pub: /catchrobo/command/orient_vertical  調停後の「縦にする」指令
// Pub: /catchrobo/arm/status        現在状態 (latched, status_rate。**mode /
//                                   last_result が変わった瞬間にも即時 publish する**)
//
// **初期位置へ動かすのはこのノードの仕事ではない** (2026-09-11〜)。
// game_state_manager_node が INIT で普通のゴール (target_pose) として出してくる。
// このノードは「起動時に実姿勢へ同期するまで enable=false」だけを担当する
class MotionGeneratorNode : public rclcpp::Node
{
public:
  explicit MotionGeneratorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  enum class Mode : uint8_t
  {
    kIdle = sharmech_msgs::msg::MotionStatus::MODE_IDLE,
    kGoal = sharmech_msgs::msg::MotionStatus::MODE_GOAL,
    kJog  = sharmech_msgs::msg::MotionStatus::MODE_JOG,
  };

  // コールバック: 入力の記録と状態遷移の判定のみ。target_ は書き換えない
  void onTargetPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void onCmdTwist(const geometry_msgs::msg::Twist::SharedPtr msg);
  void onGripper(const std_msgs::msg::Bool::SharedPtr msg);
  void onOrientVertical(const std_msgs::msg::Bool::SharedPtr msg);
  void onCancel(const std_msgs::msg::Empty::SharedPtr msg);
  void onCurrentPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void onMcuStatus(const sharmech_msgs::msg::McuStatus::SharedPtr msg);
  void onJogLimit(const sharmech_msgs::msg::JogLimit::SharedPtr msg);
  void onWorkspaceClamp(const sharmech_msgs::msg::WorkspaceClamp::SharedPtr msg);

  // 制御タイマー: target_ を書き換える唯一の場所
  void onControlTimer();
  void onStatusTimer();
  // 現在の状態を1回 publish する (タイマーからも即時通知からも通る唯一の経路)
  void publishStatus();
  // **mode / last_result が変わっていたら即 publish する。**
  // status_rate (10Hz) のタイマーだけだと、所要時間が 100ms 未満のゴール
  // (L 字の短い区間など) で NONE → SUCCEEDED の変化がサンプルの間に埋もれ、
  // last_result の変化を見ている game_state_manager_node が到達を取りこぼす。
  // コンテナは単一スレッドなので排他は要らない
  void publishStatusIfChanged();

  void rejectGoal(const std::string & reason);
  // 直近のフィードバックを CartesianState に直す (無ければ nullopt)
  std::optional<CartesianState> feedbackState() const;
  // MCU が「未初期化・原点未確定」を報告中か (status 未受信なら false)
  bool mcuUninitialized() const;
  bool isInsideWorkspace(double x, double y, double z) const;
  CartesianState clampToWorkspace(const CartesianState & state) const;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_twist_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr gripper_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr orient_vertical_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr cancel_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_sub_;
  rclcpp::Subscription<sharmech_msgs::msg::McuStatus>::SharedPtr mcu_status_sub_;
  rclcpp::Subscription<sharmech_msgs::msg::WorkspaceClamp>::SharedPtr workspace_clamp_sub_;
  rclcpp::Subscription<sharmech_msgs::msg::JogLimit>::SharedPtr jog_limit_sub_;
  rclcpp::Publisher<sharmech_msgs::msg::CartesianCommand>::SharedPtr cartesian_pub_;
  // 可視化用。/catchrobo/command/cartesian の pose だけを PoseStamped で写す
  // (RViz は独自型の CartesianCommand を表示できないため)。制御には使わない
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr command_pose_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr gripper_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr orient_vertical_pub_;
  rclcpp::Publisher<sharmech_msgs::msg::MotionStatus>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;

  // パラメータ
  double control_rate_;
  double status_rate_;
  // パラメータ (overrides 優先) をメンバへ反映する。失敗時は理由を返し何も書き換えない
  rcl_interfaces::msg::SetParametersResult applyParameters(
    const std::vector<rclcpp::Parameter> & overrides);
  // 実行中のパラメータ変更を検証して適用する (再起動なしの現場合わせ)
  rcl_interfaces::msg::SetParametersResult onSetParameters(
    const std::vector<rclcpp::Parameter> & parameters);

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  double v_max_, a_max_;        // 並進 [m/s], [m/s²]。軌道生成とジョグで共用
  double w_max_, alpha_max_;    // 姿勢 [rad/s], [rad/s²]。同上
  // 起動時 (config.yaml) の作業領域。/catchrobo/game/workspace_clamp の
  // reset=true で戻る先であり、上書き値の安全上限としても使う (下記 active_* 参照)
  double workspace_x_min_, workspace_x_max_;
  double workspace_y_min_, workspace_y_max_;
  double workspace_z_min_, workspace_z_max_;
  double twist_timeout_;        // [s] ジョグのウォッチドッグ
  // ジョグ (cmd_twist) の並進速度上限 [m/s]。**操縦層が送ってくる値を信用せず
  // ここで頭打ちにする。** v_max_ と分けてあるのは、ゴールの軌道生成 (v_max_) と
  // 手動ジョグとで妥当な上限が違うため (ジョグは操縦者が随時止められる)。
  // /catchrobo/game/jog_limit の reset=true で戻る先でもある
  double jog_v_max_;
  // 現在有効なジョグ速度上限。通常は jog_v_max_ と同じだが、
  // game_state_manager_node が微調整中 (ADJUSTING_*) に一時的に絞る
  double active_jog_v_max_;
  // ジョグの完全遮断。自動シーケンスの動作中 (ゴールを持たない GRASPING /
  // ORIENTING を含む) に game_state_manager_node が立てる
  bool jog_blocked_{false};
  std::string goal_mode_;       // "goal_priority" / "twist_priority" / "exclusive"

  // 現在有効な作業領域。通常時は workspace_*_min_/max_ と同じだが、
  // game_state_manager_node が PLACING/RETRACTING 中に一時的に絞ることがある。
  // isInsideWorkspace/clampToWorkspace はこちらを見る
  double active_workspace_x_min_, active_workspace_x_max_;
  double active_workspace_y_min_, active_workspace_y_max_;
  double active_workspace_z_min_, active_workspace_z_max_;

  // 内部状態。target_ はこのノードが唯一の所有者
  CartesianState target_{};          // 現在指令中の目標姿勢
  CartesianState target_vel_{};      // 現在指令中の目標速度
  Mode mode_{Mode::kIdle};
  CartesianState commanded_twist_{};             // 操縦層から届いた生の Twist
  std::optional<rclcpp::Time> last_twist_time_;  // ウォッチドッグ判定用
  CartesianState current_twist_{};               // レート制限後の速度
  TrapezoidalTrajectory trajectory_{};
  rclcpp::Time trajectory_start_time_;
  CartesianState goal_{};            // 実行中 / 直近のゴール
  uint8_t last_result_{sharmech_msgs::msg::MotionStatus::RESULT_NONE};
  std::string status_message_;
  bool gripper_state_{false};
  bool orient_vertical_state_{false};
  std::optional<geometry_msgs::msg::PoseStamped> latest_feedback_;
  // 直近の MCU status_flags。未受信 (nullopt) は「MCU 状態が分からない」であり、
  // その場合は current_pose だけで同期する (mcu_status を出さないシミュレータとの後方互換)
  std::optional<uint16_t> latest_mcu_flags_;
  // 目標姿勢を MCU の実姿勢へ同期済みか。**false の間は動作許可 (bit0) を 0 で送る**。
  // 同期前の target_ は原点の仮値なので、これを MCU に追従させると起動しただけで
  // アームが動く。MCU が未初期化 (bit3) を報告したら false に戻す (MCU 再起動)
  bool synced_with_feedback_{false};
  // 直近に publish した status の mode / last_result (即時 publish の判定用)。
  // 初期値は起動直後の実値と同じなので、最初の余計な publish は起きない
  Mode last_published_mode_{Mode::kIdle};
  uint8_t last_published_result_{sharmech_msgs::msg::MotionStatus::RESULT_NONE};
};

}  // namespace sharmech_core

#endif  // SHARMECH_CORE__MOTION_GENERATOR_NODE_HPP_
