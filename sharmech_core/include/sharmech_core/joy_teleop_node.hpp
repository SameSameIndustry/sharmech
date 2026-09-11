#ifndef SHARMECH_CORE__JOY_TELEOP_NODE_HPP_
#define SHARMECH_CORE__JOY_TELEOP_NODE_HPP_

#include <optional>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/joy_feedback.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>
#include <sharmech_msgs/msg/motion_status.hpp>

#include "sharmech_core/utility/dualsense_leds.hpp"

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
// Sub: /catchrobo/game/state      (MANUAL_CONTROL の間 DualSense の LED を白で点滅させる。
//      表示のみで、操作の可否はこのノードでは判断しない)
// Pub: /catchrobo/arm/cmd_twist   (ジョグが非ゼロの間 publish_rate で定期送信。
//      ゼロに戻った直後に 1 発ゼロを送り、以後は送らない)
// Pub: /catchrobo/arm/gripper     (トグルボタンの立ち上がりエッジで変化時のみ)
// Pub: /catchrobo/arm/target_pose (ホームボタンの立ち上がりエッジ)
// Pub: /catchrobo/arm/cancel      (デッドマンの立ち下がり)
// Pub: /catchrobo/game/confirm (微調整の確定。ADJUSTING_PICK/ADJUSTING_PLACE で効く)
// Pub: /catchrobo/game/toggle_manual_control (L1+R1+L3+R3 同時押しの立ち上がりエッジ。
//      VRが使えない場合にDualSenseだけで最低限試合を進められるようにする脱出ハッチ。
//      game_state_manager_node が受けて GameState::kManualControl をトグルする)
class JoyTeleopNode : public rclcpp::Node
{
public:
  explicit JoyTeleopNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~JoyTeleopNode() override;   // MANUAL_CONTROL 中に落ちても LED を元へ戻す

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
  void onGameState(const std_msgs::msg::String::SharedPtr msg);
  void onPublishTimer();

  DofMapping declareDofMapping(
    const std::string & name, int default_axis, double default_scale);
  double readDof(const DofMapping & mapping, const sensor_msgs::msg::Joy & joy);
  double readAxis(int index, const sensor_msgs::msg::Joy & joy);
  bool   readButton(int index, const sensor_msgs::msg::Joy & joy);
  void   publishHomeGoal();

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<sharmech_msgs::msg::MotionStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr game_state_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr gripper_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr cancel_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr toggle_manual_control_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr confirm_pub_;
  // コントローラーの振動 (DualSense)。joy_node が /joy/set_feedback を購読して鳴らす
  rclcpp::Publisher<sensor_msgs::msg::JoyFeedback>::SharedPtr feedback_pub_;
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
  // 微調整の確定ボタン。既定は R3 (自由操作トグルの4ボタンと同じ番号なので、
  // 他の3つが押されていないときだけ確定として扱う。下記 onJoy 参照)
  int confirm_button_;
  // 振動の強さ [0,1]。0 で無効。イベントごとに intensity を変えて区別する
  double rumble_intensity_;
  double rumble_duration_sec_;
  bool rumble_enabled_;
  // 振動を止めるための単発タイマー。JoyFeedback に長さのフィールドが無く、
  // joy_node にも時間のパラメータが無いため、こちらから停止指令を出して長さを決める
  rclcpp::TimerBase::SharedPtr rumble_stop_timer_;

  // 操作が受け付けられたことを手に返す。intensity_scale はイベントごとの相対強さ
  void rumble(double intensity_scale);

  // --- MANUAL_CONTROL の LED 表示 (DualSense のライトバー + プレイヤー LED 5 個) ---
  // PS5 だけで操作するときは MANUAL_CONTROL に入っていないと自動シーケンスに
  // 引き戻される/ジョグが遮断されるので、今その状態かどうかを手元で分かるようにする。
  // joy_node は LED を扱えないため sysfs へ直接書く (utility/dualsense_leds.hpp)
  std::string leds_sysfs_dir_;               // "" で無効
  double manual_led_blink_period_sec_;       // 0 以下なら点滅せず常時点灯
  std::vector<int64_t> lightbar_normal_rgb_;  // MANUAL_CONTROL 以外のときのライトバー色
  DualSenseLeds::Devices led_devices_;
  DualSenseLeds::PlayerSnapshot led_player_snapshot_;  // 入る前のプレイヤー LED (復元用)
  rclcpp::TimerBase::SharedPtr led_blink_timer_;
  bool led_manual_active_{false};
  bool led_blink_on_{false};
  bool warned_led_error_{false};

  void setManualLeds(bool manual);   // 入る/出る
  void onLedBlinkTimer();
  bool rediscoverLeds();             // 見つかったら true。スナップショットも取り直す
  void reportLedError(const std::string & error);

  // 内部状態
  std::optional<sensor_msgs::msg::Joy> last_joy_;
  std::optional<rclcpp::Time> last_joy_time_;
  bool gripper_state_{false};   // トグルで反転。起動時は false (開)
  bool twist_was_active_{false};  // 直前に publish した Twist が非ゼロだったか (停止の 1 発用)
  std::vector<int32_t> prev_buttons_;
  bool manual_toggle_combo_was_active_{false};  // 4ボタン同時押しの立ち上がりエッジ検出用
  uint8_t last_seen_result_{sharmech_msgs::msg::MotionStatus::RESULT_NONE};
  bool warned_out_of_range_{false};
  bool warned_no_home_{false};
};

}  // namespace sharmech_core

#endif  // SHARMECH_CORE__JOY_TELEOP_NODE_HPP_
