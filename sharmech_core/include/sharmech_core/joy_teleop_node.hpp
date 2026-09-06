#ifndef SHARMECH_CORE__JOY_TELEOP_NODE_HPP_
#define SHARMECH_CORE__JOY_TELEOP_NODE_HPP_

#include <optional>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <sharmech_msgs/msg/motion_status.hpp>

namespace sharmech_core
{

// PS4 コントローラ入力を操縦層の共通インターフェースへ変換するノード
//
// このノードの本質は「割り当て表」であり、軸・ボタン番号はすべて config。
// VR と同じ /catchrobo/arm/* へ publish するため、下流からは区別がつかない。
//
// 仕様の正本: sharmech_core/docs/joy_teleop_node.md
//
// Sub: /joy
// Sub: /catchrobo/arm/status      (ホーム復帰の却下をログに出すため・任意)
// Pub: /catchrobo/arm/cmd_twist   (publish_rate で定期送信)
// Pub: /catchrobo/arm/gripper     (同上)
// Pub: /catchrobo/arm/target_pose (ホームボタンの立ち上がりエッジ)
// Pub: /catchrobo/arm/cancel      (デッドマンの立ち下がり)
// Pub: /catchrobo/game/toggle_manual_control (L1+R1+L3+R3 同時押しの立ち上がりエッジ。
//      VRが使えない場合にDualSenseだけで最低限試合を進められるようにする脱出ハッチ。
//      game_state_manager_node が受けて GameState::kManualControl をトグルする)
class JoyTeleopNode : public rclcpp::Node
{
public:
  explicit JoyTeleopNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // 1自由度分の割り当て。軸とボタン対の両方から駆動でき、合算する。
  // 使わない側は -1。反転は scale の符号で表現する (反転フラグは持たない)
  struct DofMapping
  {
    int axis{-1};
    double scale{0.0};
    int button_pos{-1};
    int button_neg{-1};
  };

  void onJoy(const sensor_msgs::msg::Joy::SharedPtr msg);
  void onStatus(const sharmech_msgs::msg::MotionStatus::SharedPtr msg);
  void onPublishTimer();

  DofMapping declareDofMapping(
    const std::string & name, int default_axis, double default_scale);
  double readDof(const DofMapping & mapping, const sensor_msgs::msg::Joy & joy);
  double readAxis(int index, const sensor_msgs::msg::Joy & joy);
  bool   readButton(int index, const sensor_msgs::msg::Joy & joy);
  void   publishHomeGoal();

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<sharmech_msgs::msg::MotionStatus>::SharedPtr status_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr gripper_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr cancel_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr toggle_manual_control_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  // パラメータ
  double publish_rate_;
  double joy_timeout_;
  double deadzone_;
  bool use_deadman_;
  std::vector<double> home_pose_;  // [x, y, z, pitch, yaw]。空なら無効
  DofMapping vx_map_, vy_map_, vz_map_, pitch_map_, yaw_map_;
  int deadman_button_;
  int gripper_toggle_button_;
  int home_button_;
  // 自由操作トグルの4ボタン同時押し (既定は DualSense/PS4 の L1・R1・L3・R3)
  int manual_toggle_button_l1_;
  int manual_toggle_button_r1_;
  int manual_toggle_button_l_stick_;
  int manual_toggle_button_r_stick_;

  // 内部状態
  std::optional<sensor_msgs::msg::Joy> last_joy_;
  std::optional<rclcpp::Time> last_joy_time_;
  bool gripper_state_{false};   // トグルで反転。起動時は false (開)
  std::vector<int32_t> prev_buttons_;
  bool manual_toggle_combo_was_active_{false};  // 4ボタン同時押しの立ち上がりエッジ検出用
  uint8_t last_seen_result_{sharmech_msgs::msg::MotionStatus::RESULT_NONE};
  bool warned_out_of_range_{false};
  bool warned_no_home_{false};
};

}  // namespace sharmech_core

#endif  // SHARMECH_CORE__JOY_TELEOP_NODE_HPP_
