#include "sharmech_core/game_state_manager_node.hpp"
#include "sharmech_core/utility/orientation_utils.hpp"

#include <rclcpp_components/register_node_macro.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <stdexcept>

namespace sharmech_core
{

namespace
{

geometry_msgs::msg::PoseStamped toPoseStampedMsg(const CartesianState & s)
{
  geometry_msgs::msg::PoseStamped msg;
  msg.header.frame_id = "field";
  msg.pose.position.x = s.x;
  msg.pose.position.y = s.y;
  msg.pose.position.z = s.z;
  tf2::Quaternion q;
  q.setRPY(0.0, s.pitch, s.yaw);
  msg.pose.orientation = tf2::toMsg(q);
  return msg;
}

}  // namespace

GameStateManagerNode::GameStateManagerNode(const rclcpp::NodeOptions & options)
: Node("game_state_manager_node", options)
{
  // フィールドの色によってシューティングボックスのスロット座標が変わるため、
  // 独断で既定値を決めず launch 引数での明示指定を必須にする
  // (config.yaml 側にも既定値を置かない。sharmech.launch.xml の field_color 引数を参照)
  const std::string field_color = declare_parameter("field_color", std::string(""));
  if (field_color != "red" && field_color != "blue") {
    RCLCPP_FATAL(get_logger(),
      "field_color must be \"red\" or \"blue\" (got \"%s\"). "
      "launch引数 field_color:=red|blue で明示してください",
      field_color.c_str());
    throw std::invalid_argument("field_color must be 'red' or 'blue'");
  }

  const auto placement_order_i64 =
    declare_parameter("placement_order", std::vector<int64_t>{});
  std::vector<int> placement_order(placement_order_i64.begin(), placement_order_i64.end());

  GameStateMachine::Config config;
  config.slots = loadSlots(field_color);
  config.placement_order = placement_order;
  config.slot_clamp_margin_m = declare_parameter("slot_clamp_margin_m", 0.03);
  config.transport_clearance_z = declare_parameter("transport_clearance_z", 0.20);
  config.retract_clearance_z = declare_parameter("retract_clearance_z", 0.20);
  config.grasp_dwell_sec = declare_parameter("grasp_dwell_sec", 0.3);

  if (config.placement_order.empty()) {
    RCLCPP_FATAL(get_logger(), "placement_order is empty; check config.yaml");
    throw std::invalid_argument("empty placement_order");
  }
  for (const int id : config.placement_order) {
    if (id < 0 || static_cast<std::size_t>(id) >= config.slots.size()) {
      RCLCPP_FATAL(get_logger(),
        "placement_order contains out-of-range slot id: %d (slots.size()=%zu)",
        id, config.slots.size());
      throw std::invalid_argument("placement_order out of range");
    }
  }

  machine_ = std::make_unique<GameStateMachine>(config);

  pick_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "/catchrobo/game/pick_request", 10,
    std::bind(&GameStateManagerNode::onPickRequest, this, std::placeholders::_1));
  place_sub_ = create_subscription<std_msgs::msg::Empty>(
    "/catchrobo/game/place_request", 10,
    std::bind(&GameStateManagerNode::onPlaceRequest, this, std::placeholders::_1));
  status_sub_ = create_subscription<sharmech_msgs::msg::MotionStatus>(
    "/catchrobo/arm/status", rclcpp::QoS(1).transient_local(),
    std::bind(&GameStateManagerNode::onArmStatus, this, std::placeholders::_1));

  target_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
    "/catchrobo/arm/target_pose", 10);
  gripper_pub_ = create_publisher<std_msgs::msg::Bool>(
    "/catchrobo/arm/gripper", 10);
  orient_vertical_pub_ = create_publisher<std_msgs::msg::Bool>(
    "/catchrobo/arm/orient_vertical", 10);
  workspace_clamp_pub_ = create_publisher<sharmech_msgs::msg::WorkspaceClamp>(
    "/catchrobo/game/workspace_clamp", 10);
  // latched: 後から接続したVRクライアント・観測用ダッシュボードにも現在状態が即座に届く
  state_pub_ = create_publisher<std_msgs::msg::String>(
    "/catchrobo/game/state", rclcpp::QoS(1).transient_local());

  const double state_publish_rate = declare_parameter("state_publish_rate", 10.0);
  const auto period = std::chrono::duration<double>(1.0 / state_publish_rate);
  timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&GameStateManagerNode::onTimer, this));

  RCLCPP_INFO(get_logger(),
    "game_state_manager_node started (field_color=%s, %zu slots, %zu-step placement_order)",
    field_color.c_str(), config.slots.size(), config.placement_order.size());
}

std::vector<CartesianState> GameStateManagerNode::loadSlots(const std::string & color_suffix)
{
  const auto slot_x = declare_parameter("slot_x_" + color_suffix, std::vector<double>{});
  const auto slot_y = declare_parameter("slot_y_" + color_suffix, std::vector<double>{});
  const auto slot_z = declare_parameter("slot_z_" + color_suffix, std::vector<double>{});

  if (slot_x.empty() || slot_x.size() != slot_y.size() || slot_x.size() != slot_z.size()) {
    RCLCPP_FATAL(get_logger(),
      "slot_x_%s / slot_y_%s / slot_z_%s must be non-empty and have equal length "
      "(got %zu / %zu / %zu)",
      color_suffix.c_str(), color_suffix.c_str(), color_suffix.c_str(),
      slot_x.size(), slot_y.size(), slot_z.size());
    throw std::invalid_argument("invalid slot coordinate arrays");
  }

  std::vector<CartesianState> slots(slot_x.size());
  for (std::size_t i = 0; i < slot_x.size(); ++i) {
    slots[i].x = slot_x[i];
    slots[i].y = slot_y[i];
    slots[i].z = slot_z[i];
  }
  return slots;
}

void GameStateManagerNode::onPickRequest(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  const auto pitch_yaw = OrientationUtils::toPitchYaw(msg->pose.orientation);
  CartesianState pose;
  pose.x = msg->pose.position.x;
  pose.y = msg->pose.position.y;
  pose.z = msg->pose.position.z;
  pose.pitch = pitch_yaw.pitch;
  pose.yaw = pitch_yaw.yaw;
  machine_->onPickPoseReceived(pose);
  publishPendingOutputs();
}

void GameStateManagerNode::onPlaceRequest(const std_msgs::msg::Empty::SharedPtr)
{
  machine_->onPlaceRequested();
  publishPendingOutputs();
}

void GameStateManagerNode::onArmStatus(const sharmech_msgs::msg::MotionStatus::SharedPtr msg)
{
  if (msg->last_result == prev_last_result_) {return;}
  prev_last_result_ = msg->last_result;

  if (msg->last_result == sharmech_msgs::msg::MotionStatus::RESULT_SUCCEEDED) {
    machine_->onGoalReached(now().seconds());
  } else if (msg->last_result == sharmech_msgs::msg::MotionStatus::RESULT_REJECTED ||
    msg->last_result == sharmech_msgs::msg::MotionStatus::RESULT_ABORTED)
  {
    RCLCPP_WARN(get_logger(),
      "Automated goal was not accepted (last_result=%u, %s); returning to WAITING_FOR_PICK",
      msg->last_result, msg->message.c_str());
    machine_->onGoalRejectedOrAborted();
  }
  publishPendingOutputs();
}

void GameStateManagerNode::onTimer()
{
  machine_->tick(now().seconds());
  publishPendingOutputs();
  publishState();
}

void GameStateManagerNode::publishPendingOutputs()
{
  // 順序が重要: 作業領域クランプを先に送ってから、その後にゴールを送る。
  // 逆だとPLACINGのゴールが縮小前の(広い)クランプで一瞬評価される可能性がある
  if (machine_->hasPendingWorkspaceClamp()) {
    const auto clamp = machine_->consumePendingWorkspaceClamp();
    sharmech_msgs::msg::WorkspaceClamp msg;
    msg.reset = clamp.reset;
    msg.x_min = clamp.x_min;
    msg.x_max = clamp.x_max;
    msg.y_min = clamp.y_min;
    msg.y_max = clamp.y_max;
    msg.z_min = clamp.z_min;
    msg.z_max = clamp.z_max;
    workspace_clamp_pub_->publish(msg);
  }
  if (machine_->hasPendingGripper()) {
    std_msgs::msg::Bool msg;
    msg.data = machine_->consumePendingGripper();
    gripper_pub_->publish(msg);
  }
  if (machine_->hasPendingOrientVertical()) {
    std_msgs::msg::Bool msg;
    msg.data = machine_->consumePendingOrientVertical();
    orient_vertical_pub_->publish(msg);
  }
  if (machine_->hasPendingGoal()) {
    target_pose_pub_->publish(toPoseStampedMsg(machine_->consumePendingGoal()));
  }
}

void GameStateManagerNode::publishState()
{
  std_msgs::msg::String msg;
  msg.data = toString(machine_->state());
  state_pub_->publish(msg);
}

}  // namespace sharmech_core

RCLCPP_COMPONENTS_REGISTER_NODE(sharmech_core::GameStateManagerNode)
