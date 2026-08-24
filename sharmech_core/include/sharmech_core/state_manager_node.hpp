#ifndef SHARMECH_CORE__STATE_MANAGER_NODE_HPP_
#define SHARMECH_CORE__STATE_MANAGER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace sharmech_core
{

enum class TaskState
{
  IDLE,       // 待機中。新しい目標を受け付ける
  PLANNING,   // 軌道生成中 (trajectory_generator → hardware_bridge 処理中)
  EXECUTING,  // 軌道実行中
  DONE,       // 実行完了。次サイクルでIDLEへ遷移
  ERROR,      // エラー発生
};

// ロボット状態管理ノード
//
// [タスク状態管理]  IDLE → PLANNING → EXECUTING → DONE → IDLE
//
// エンドエフェクタの逆運動学 (5節リンク・Z軸・テーブルヨー軸・手首差動機構) は
// MCU側で行うため、本ノードはCartesian姿勢のみを扱う。到達判定は
// /robot/current_pose (hardware_bridge_node からの実姿勢) と
// /cartesian_trajectory の最終ウェイポイント (目標姿勢) の差分で行う。
//
// Sub: /target_pose         (vr_interface_node から)
// Sub: /robot/current_pose  (hardware_bridge_node から)
// Sub: /cartesian_trajectory (trajectory_generator_node から、完了判定用の目標取得)
// Pub: /goal_pose           (trajectory_generator_node へ)
// Srv: /task/cancel         (緊急停止)
class StateManagerNode : public rclcpp::Node
{
public:
  explicit StateManagerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void onTargetPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void onCurrentPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void onCartesianTrajectory(const nav_msgs::msg::Path::SharedPtr msg);
  void onTaskCancel(
    const std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr response);

  void updateState();
  bool isExecutionComplete() const;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr             cartesian_traj_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr    goal_pose_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr               cancel_srv_;
  rclcpp::TimerBase::SharedPtr                                     state_timer_;

  TaskState state_{TaskState::IDLE};

  // 現在姿勢・目標姿勢 (Cartesian)
  geometry_msgs::msg::Pose current_pose_{};
  geometry_msgs::msg::Pose goal_pose_{};
  bool has_goal_pose_{false};

  double pos_tolerance_;  // [m]   到達判定閾値 (位置)
  double rot_tolerance_;  // [rad] 到達判定閾値 (pitch/yaw)
};

}  // namespace sharmech_core

#endif  // SHARMECH_CORE__STATE_MANAGER_NODE_HPP_
