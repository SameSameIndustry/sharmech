#ifndef SHARMECH_CORE__TASK_MANAGER_NODE_HPP_
#define SHARMECH_CORE__TASK_MANAGER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace sharmech_core
{

// ピックアンドプレース タスクマネージャーノード
// 5節リンクアームのピックアンドプレース動作を状態機械で管理する
//
// Service (server): /task/start   (std_srvs/Trigger) - タスク開始
// Service (server): /task/abort   (std_srvs/Trigger) - タスク中断
// Subscribe       : /joint_states (sensor_msgs/JointState) - アーム状態フィードバック
// Publish         : /target_pose  (geometry_msgs/Point) - 目標位置指令
// Service (client): /gripper/command (std_srvs/SetBool) - グリッパ制御
enum class TaskState
{
  IDLE,
  MOVE_TO_PICK,    // ピック位置へ移動中
  PICKING,         // 把持動作中
  MOVE_TO_PLACE,   // プレース位置へ移動中
  PLACING,         // 配置動作中
  HOMING,          // ホーム位置へ帰還中
  ABORTED,
};

class TaskManagerNode : public rclcpp::Node
{
public:
  explicit TaskManagerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void onTaskStart(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response);

  void onTaskAbort(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response);

  void onJointStates(const sensor_msgs::msg::JointState::SharedPtr msg);

  void updateStateMachine();
  void publishTargetPose(double x, double y);
  void commandGripper(bool grasp);
  bool isArmAtTarget();

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr task_start_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr task_abort_srv_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr target_pose_pub_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr gripper_client_;
  rclcpp::TimerBase::SharedPtr state_machine_timer_;

  TaskState state_{TaskState::IDLE};

  // タスクパラメータ (config.yaml から読み込み)
  double pick_x_, pick_y_;    // ピック位置
  double place_x_, place_y_;  // プレース位置
  double home_x_, home_y_;    // ホーム位置

  double position_tolerance_{0.005};  // 位置到達判定閾値 [m]

  // 現在のEE位置 (kinematics_nodeからのフィードバック用)
  double current_x_{0.0};
  double current_y_{0.0};
};

}  // namespace sharmech_core

#endif  // SHARMECH_CORE__TASK_MANAGER_NODE_HPP_
