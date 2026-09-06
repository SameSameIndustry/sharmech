#ifndef SHARMECH_CORE__MOTION_GENERATOR_NODE_HPP_
#define SHARMECH_CORE__MOTION_GENERATOR_NODE_HPP_

#include <optional>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <sharmech_msgs/msg/cartesian_command.hpp>
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
// Sub: /catchrobo/arm/current_pose  実姿勢 (状態トピックの残距離計算に使用)
// Sub: /catchrobo/game/workspace_clamp  作業領域クランプの動的上書き (game_state_manager_node)
// Pub: /catchrobo/command/cartesian 位置 + 速度ストリーム (control_rate)
// Pub: /catchrobo/command/gripper   調停後のグリッパ指令
// Pub: /catchrobo/command/orient_vertical  調停後の「縦にする」指令
// Pub: /catchrobo/arm/status        現在状態 (latched, status_rate)
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
  void onWorkspaceClamp(const sharmech_msgs::msg::WorkspaceClamp::SharedPtr msg);

  // 制御タイマー: target_ を書き換える唯一の場所
  void onControlTimer();
  void onStatusTimer();

  void rejectGoal(const std::string & reason);
  bool isInsideWorkspace(double x, double y, double z) const;
  CartesianState clampToWorkspace(const CartesianState & state) const;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_twist_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr gripper_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr orient_vertical_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr cancel_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_sub_;
  rclcpp::Subscription<sharmech_msgs::msg::WorkspaceClamp>::SharedPtr workspace_clamp_sub_;
  rclcpp::Publisher<sharmech_msgs::msg::CartesianCommand>::SharedPtr cartesian_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr gripper_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr orient_vertical_pub_;
  rclcpp::Publisher<sharmech_msgs::msg::MotionStatus>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;

  // パラメータ
  double control_rate_;
  double status_rate_;
  double v_max_, a_max_;        // 並進 [m/s], [m/s²]。軌道生成とジョグで共用
  double w_max_, alpha_max_;    // 姿勢 [rad/s], [rad/s²]。同上
  // 起動時 (config.yaml) の作業領域。/catchrobo/game/workspace_clamp の
  // reset=true で戻る先であり、上書き値の安全上限としても使う (下記 active_* 参照)
  double workspace_x_min_, workspace_x_max_;
  double workspace_y_min_, workspace_y_max_;
  double workspace_z_min_, workspace_z_max_;
  double twist_timeout_;        // [s] ジョグのウォッチドッグ
  std::string goal_mode_;       // "twist_priority" / "exclusive"

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
  bool synced_with_feedback_{false};
};

}  // namespace sharmech_core

#endif  // SHARMECH_CORE__MOTION_GENERATOR_NODE_HPP_
