#include "sharmech_core/joy_teleop_node.hpp"

#include <rclcpp_components/register_node_macro.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <cmath>

namespace sharmech_core
{

JoyTeleopNode::JoyTeleopNode(const rclcpp::NodeOptions & options)
: Node("joy_teleop_node", options)
{
  publish_rate_ = declare_parameter("publish_rate", 50.0);
  joy_timeout_  = declare_parameter("joy_timeout", 0.5);
  deadzone_     = declare_parameter("deadzone", 0.15);
  use_deadman_  = declare_parameter("use_deadman", true);
  home_pose_    = declare_parameter("home_pose", std::vector<double>{});

  // 既定値は目安。実機で `ros2 topic echo /joy` を見て config で合わせること
  vx_map_    = declareDofMapping("vx", 1, 0.10);
  vy_map_    = declareDofMapping("vy", 0, 0.10);
  vz_map_    = declareDofMapping("vz", 7, 0.10);
  pitch_map_ = declareDofMapping("pitch", 4, 0.50);
  yaw_map_   = declareDofMapping("yaw", 3, 0.50);
  deadman_button_        = declare_parameter("deadman_button", 4);
  gripper_toggle_button_ = declare_parameter("gripper_toggle_button", 0);
  home_button_           = declare_parameter("home_button", 2);
  // VRが使えない場合の脱出ハッチ。既定は一般的なLinux上のDualSense/PS4マッピング
  // (L1=4, R1=5, L3=11, R3=12)。実機で `ros2 topic echo /joy` を見て合わせること
  manual_toggle_button_l1_        = declare_parameter("manual_toggle_button_l1", 4);
  manual_toggle_button_r1_        = declare_parameter("manual_toggle_button_r1", 5);
  manual_toggle_button_l_stick_   = declare_parameter("manual_toggle_button_l_stick", 11);
  manual_toggle_button_r_stick_   = declare_parameter("manual_toggle_button_r_stick", 12);

  if (!home_pose_.empty() && home_pose_.size() != 5) {
    RCLCPP_FATAL(get_logger(),
      "home_pose must be [x, y, z, pitch, yaw] (got %zu values)", home_pose_.size());
    throw std::invalid_argument("home_pose must have 5 elements");
  }

  joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
    "/joy", 10, std::bind(&JoyTeleopNode::onJoy, this, std::placeholders::_1));
  status_sub_ = create_subscription<sharmech_msgs::msg::MotionStatus>(
    "/catchrobo/arm/status", rclcpp::QoS(1).transient_local(),
    std::bind(&JoyTeleopNode::onStatus, this, std::placeholders::_1));

  twist_pub_ = create_publisher<geometry_msgs::msg::Twist>(
    "/catchrobo/arm/cmd_twist", 10);
  gripper_pub_ = create_publisher<std_msgs::msg::Bool>(
    "/catchrobo/arm/gripper", 10);
  target_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
    "/catchrobo/arm/target_pose", 10);
  cancel_pub_ = create_publisher<std_msgs::msg::Empty>(
    "/catchrobo/arm/cancel", 10);
  toggle_manual_control_pub_ = create_publisher<std_msgs::msg::Empty>(
    "/catchrobo/game/toggle_manual_control", 10);

  const auto period = std::chrono::duration<double>(1.0 / publish_rate_);
  publish_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&JoyTeleopNode::onPublishTimer, this));

  RCLCPP_INFO(get_logger(), "joy_teleop_node started (%.0f Hz, deadman=%s)",
    publish_rate_, use_deadman_ ? "on" : "off");
}

JoyTeleopNode::DofMapping JoyTeleopNode::declareDofMapping(
  const std::string & name, int default_axis, double default_scale)
{
  DofMapping mapping;
  mapping.axis       = declare_parameter(name + "_axis", default_axis);
  mapping.scale      = declare_parameter(name + "_scale", default_scale);
  mapping.button_pos = declare_parameter(name + "_button_pos", -1);
  mapping.button_neg = declare_parameter(name + "_button_neg", -1);
  return mapping;
}

void JoyTeleopNode::onJoy(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  // エッジ検出 (前回のボタン状態と比較)
  auto pressed_edge = [this, &msg](int index) {
      if (index < 0 || index >= static_cast<int>(msg->buttons.size())) {return false;}
      const bool now_pressed = msg->buttons[index] != 0;
      const bool was_pressed =
        index < static_cast<int>(prev_buttons_.size()) && prev_buttons_[index] != 0;
      return now_pressed && !was_pressed;
    };
  auto released_edge = [this, &msg](int index) {
      if (index < 0 || index >= static_cast<int>(msg->buttons.size())) {return false;}
      const bool now_pressed = msg->buttons[index] != 0;
      const bool was_pressed =
        index < static_cast<int>(prev_buttons_.size()) && prev_buttons_[index] != 0;
      return !now_pressed && was_pressed;
    };

  if (pressed_edge(gripper_toggle_button_)) {
    gripper_state_ = !gripper_state_;
    RCLCPP_INFO(get_logger(), "Gripper toggled → %s",
      gripper_state_ ? "close" : "open");
  }
  if (pressed_edge(home_button_)) {
    publishHomeGoal();
  }
  if (released_edge(deadman_button_)) {
    // 自動移動中にデッドマンを離したら止まる (操作者の能動的指令が消えた)
    cancel_pub_->publish(std_msgs::msg::Empty());
  }

  // 自由操作トグル: L1+R1+L3+R3 が「同時に」揃った瞬間(立ち上がりエッジ)のみ
  // 1回 publish する。押しっぱなしの間に連打しないよう、combo自体の
  // 立ち上がりで判定する (個々のボタンのpressed_edgeではない)
  const bool combo_now =
    readButton(manual_toggle_button_l1_, *msg) &&
    readButton(manual_toggle_button_r1_, *msg) &&
    readButton(manual_toggle_button_l_stick_, *msg) &&
    readButton(manual_toggle_button_r_stick_, *msg);
  if (combo_now && !manual_toggle_combo_was_active_) {
    toggle_manual_control_pub_->publish(std_msgs::msg::Empty());
    RCLCPP_INFO(get_logger(), "Manual control toggle combo detected (L1+R1+L3+R3)");
  }
  manual_toggle_combo_was_active_ = combo_now;

  prev_buttons_.assign(msg->buttons.begin(), msg->buttons.end());
  last_joy_      = *msg;
  last_joy_time_ = now();
}

void JoyTeleopNode::onStatus(const sharmech_msgs::msg::MotionStatus::SharedPtr msg)
{
  using MotionStatus = sharmech_msgs::msg::MotionStatus;
  if (msg->last_result == MotionStatus::RESULT_REJECTED &&
    last_seen_result_ != MotionStatus::RESULT_REJECTED)
  {
    RCLCPP_WARN(get_logger(), "Goal rejected by motion_generator: %s",
      msg->message.c_str());
  }
  last_seen_result_ = msg->last_result;
}

void JoyTeleopNode::onPublishTimer()
{
  geometry_msgs::msg::Twist twist;  // 既定はゼロ

  const bool joy_alive = last_joy_ && last_joy_time_ &&
    (now() - *last_joy_time_).seconds() <= joy_timeout_;

  if (!joy_alive) {
    // コントローラ切断。最後のスティック値を送り続けると暴走するため
    // 全入力をニュートラルとして扱う。publish は止めない
    // (ゼロを明示的に送れば watchdog を待たず即座に減速が始まる)
    if (last_joy_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "/joy not received for %.1f s; sending neutral", joy_timeout_);
    }
  } else if (use_deadman_ && !readButton(deadman_button_, *last_joy_)) {
    // デッドマン非押下。「送信停止」ではなくゼロを送る
  } else {
    twist.linear.x  = readDof(vx_map_, *last_joy_);
    twist.linear.y  = readDof(vy_map_, *last_joy_);
    twist.linear.z  = readDof(vz_map_, *last_joy_);
    twist.angular.y = readDof(pitch_map_, *last_joy_);
    twist.angular.z = readDof(yaw_map_, *last_joy_);
  }

  twist_pub_->publish(twist);

  std_msgs::msg::Bool gripper_msg;
  gripper_msg.data = gripper_state_;
  gripper_pub_->publish(gripper_msg);
}

double JoyTeleopNode::readDof(
  const DofMapping & mapping, const sensor_msgs::msg::Joy & joy)
{
  double raw = readAxis(mapping.axis, joy);
  if (readButton(mapping.button_pos, joy)) {raw += 1.0;}
  if (readButton(mapping.button_neg, joy)) {raw -= 1.0;}
  if (std::abs(raw) < deadzone_) {return 0.0;}
  return raw * mapping.scale;
}

double JoyTeleopNode::readAxis(int index, const sensor_msgs::msg::Joy & joy)
{
  if (index < 0) {return 0.0;}
  if (index >= static_cast<int>(joy.axes.size())) {
    if (!warned_out_of_range_) {
      RCLCPP_ERROR(get_logger(),
        "Configured axis %d exceeds /joy axes size (%zu); treating as 0. "
        "Check the mapping with `ros2 topic echo /joy`",
        index, joy.axes.size());
      warned_out_of_range_ = true;
    }
    return 0.0;
  }
  return joy.axes[index];
}

bool JoyTeleopNode::readButton(int index, const sensor_msgs::msg::Joy & joy)
{
  if (index < 0) {return false;}
  if (index >= static_cast<int>(joy.buttons.size())) {
    if (!warned_out_of_range_) {
      RCLCPP_ERROR(get_logger(),
        "Configured button %d exceeds /joy buttons size (%zu); treating as unpressed. "
        "Check the mapping with `ros2 topic echo /joy`",
        index, joy.buttons.size());
      warned_out_of_range_ = true;
    }
    return false;
  }
  return joy.buttons[index] != 0;
}

void JoyTeleopNode::publishHomeGoal()
{
  if (home_pose_.empty()) {
    if (!warned_no_home_) {
      RCLCPP_WARN(get_logger(), "home_pose is not configured; home button disabled");
      warned_no_home_ = true;
    }
    return;
  }

  geometry_msgs::msg::PoseStamped goal;
  goal.header.stamp    = now();
  goal.header.frame_id = "field";
  goal.pose.position.x = home_pose_[0];
  goal.pose.position.y = home_pose_[1];
  goal.pose.position.z = home_pose_[2];
  tf2::Quaternion q;
  q.setRPY(0.0, home_pose_[3], home_pose_[4]);
  goal.pose.orientation = tf2::toMsg(q);
  target_pose_pub_->publish(goal);
  RCLCPP_INFO(get_logger(), "Home goal published");
}

}  // namespace sharmech_core

RCLCPP_COMPONENTS_REGISTER_NODE(sharmech_core::JoyTeleopNode)
