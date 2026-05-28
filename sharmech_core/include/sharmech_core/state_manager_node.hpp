#ifndef SHARMECH_CORE__STATE_MANAGER_NODE_HPP_
#define SHARMECH_CORE__STATE_MANAGER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <std_srvs/srv/trigger.hpp>
#include "sharmech_core/utility/five_bar_kinematics.hpp"

namespace sharmech_core
{

enum class TaskState
{
  IDLE,       // 待機中。新しい目標を受け付ける
  PLANNING,   // 軌道生成中 (trajectory_generator → kinematics 処理中)
  EXECUTING,  // 軌道実行中
  DONE,       // 実行完了。次サイクルでIDLEへ遷移
  ERROR,      // エラー発生
};

// ロボット状態管理ノード
//
// [タスク状態管理]  IDLE → PLANNING → EXECUTING → DONE → IDLE
// [物理状態管理]   /joint_states から現在位置をFK計算して管理
//
// Sub: /target_pose        (vr_interface_node から)
// Sub: /joint_states       (hardware_bridge_node から)
// Sub: /joint_trajectory   (kinematics_node から、完了監視用)
// Pub: /goal_pose          (trajectory_generator_node へ)
// Pub: /robot/current_pose (vr_interface_node へ)
// Srv: /task/cancel        (緊急停止)
class StateManagerNode : public rclcpp::Node
{
public:
  explicit StateManagerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void onTargetPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void onJointStates(const sensor_msgs::msg::JointState::SharedPtr msg);
  void onJointTrajectory(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg);
  void onTaskCancel(
    const std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr response);

  void updateState();
  void publishCurrentPose();
  bool isExecutionComplete() const;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr    target_pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr       joint_states_sub_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_traj_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr       goal_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr       current_pose_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr                  cancel_srv_;
  rclcpp::TimerBase::SharedPtr                                        state_timer_;

  TaskState state_{TaskState::IDLE};

  // 現在の関節角度・位置
  double current_theta1_{0.0};
  double current_theta2_{0.0};
  CartesianPoint current_pos_{0.0, 0.0};

  // 軌道の最終ウェイポイント (完了判定用)
  double goal_theta1_{0.0};
  double goal_theta2_{0.0};
  bool has_goal_joints_{false};

  double joint_tolerance_{0.05};  // [rad] 到達判定閾値

  FiveBarParams kinematics_params_;
};

}  // namespace sharmech_core

#endif  // SHARMECH_CORE__STATE_MANAGER_NODE_HPP_
