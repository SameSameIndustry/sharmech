#include "sharmech_core/motion_generator_node.hpp"
#include "sharmech_core/utility/orientation_utils.hpp"

#include <rclcpp_components/register_node_macro.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <algorithm>
#include <cmath>

namespace sharmech_core
{

namespace
{

constexpr double kTwistEpsilon = 1e-6;

// 5成分のいずれかが非ゼロか (ゼロ Twist は「ジョグ入力なし」を意味する)
bool isActive(const CartesianState & twist)
{
  return std::abs(twist.x) > kTwistEpsilon ||
         std::abs(twist.y) > kTwistEpsilon ||
         std::abs(twist.z) > kTwistEpsilon ||
         std::abs(twist.pitch) > kTwistEpsilon ||
         std::abs(twist.yaw) > kTwistEpsilon;
}

// value を target へ、1周期あたり max_step を上限に近づける (レート制限)
double approach(double value, double target, double max_step)
{
  const double diff = target - value;
  return value + std::clamp(diff, -max_step, max_step);
}

geometry_msgs::msg::Pose toPoseMsg(const CartesianState & s)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = s.x;
  pose.position.y = s.y;
  pose.position.z = s.z;
  tf2::Quaternion q;
  q.setRPY(0.0, s.pitch, s.yaw);
  pose.orientation = tf2::toMsg(q);
  return pose;
}

}  // namespace

MotionGeneratorNode::MotionGeneratorNode(const rclcpp::NodeOptions & options)
: Node("motion_generator_node", options),
  trajectory_start_time_(0, 0, RCL_ROS_TIME)
{
  // control_rate / status_rate はタイマー周期を決めるため起動時のみ (実行中は変えない)
  control_rate_ = declare_parameter("control_rate", 100.0);
  status_rate_ = declare_parameter("status_rate", 10.0);

  // 以下は実行中に ros2 param set で変更できる (現場合わせ用。下記 onSetParameters)
  declare_parameter("v_max", 0.10);
  declare_parameter("a_max", 0.20);
  declare_parameter("w_max", 1.0);
  declare_parameter("alpha_max", 2.0);
  declare_parameter("workspace_x_min", -0.20);
  declare_parameter("workspace_x_max", 0.20);
  declare_parameter("workspace_y_min", 0.05);
  declare_parameter("workspace_y_max", 0.30);
  declare_parameter("workspace_z_min", 0.00);
  declare_parameter("workspace_z_max", 0.30);
  declare_parameter("twist_timeout", 0.4);
  // ジョグの並進速度上限 [m/s]。**操縦層が送ってくる cmd_twist の大きさを
  // ここで頭打ちにする。** 従来はクランプが無く、送られた値がそのまま目標速度に
  // なっていた (VRクライアントが自前で上限を持って肩代わりしていた)。
  // 既定 1.0 はWebXRクライアントの通常時の上限と同じで、現状の挙動を変えない。
  // MANUAL_CONTROL 中にVRが送る 10 m/s はここで 1.0 に落ちる
  declare_parameter("jog_v_max", 1.0);
  declare_parameter("goal_mode", std::string("goal_priority"));

  // 本番設置での原点ズレ補正 (sharmech/docs/field_dimensions.md 参照)。既定0.0。
  // 作業領域全体をこの分だけ平行移動する。Zは game_state_manager_node の
  // field_origin_offset_z_m と同じ意味・同じ値を使う想定
  declare_parameter("field_origin_offset_x_m", 0.0);
  declare_parameter("field_origin_offset_y_m", 0.0);
  declare_parameter("field_origin_offset_z_m", 0.0);

  // 起動時は「上書き無し」で反映する。ここで弾かれる値 (未知の goal_mode 等) は
  // 起動を止める。実行中の変更は reason を返して却下するだけで走り続ける
  const auto initial = applyParameters({});
  if (!initial.successful) {
    RCLCPP_FATAL(get_logger(), "%s", initial.reason.c_str());
    throw std::invalid_argument(initial.reason);
  }

  param_callback_handle_ = add_on_set_parameters_callback(
    std::bind(&MotionGeneratorNode::onSetParameters, this, std::placeholders::_1));

  // 起動直後は有効なジョグ上限 = 起動時パラメータ
  active_jog_v_max_ = jog_v_max_;

  // 起動直後は有効な作業領域 = config.yaml のデフォルト
  active_workspace_x_min_ = workspace_x_min_;
  active_workspace_x_max_ = workspace_x_max_;
  active_workspace_y_min_ = workspace_y_min_;
  active_workspace_y_max_ = workspace_y_max_;
  active_workspace_z_min_ = workspace_z_min_;
  active_workspace_z_max_ = workspace_z_max_;

  // 起動直後の目標姿勢は作業領域内に収めておく (MCU フィードバックが届けば同期される)
  target_ = clampToWorkspace(target_);

  target_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "/catchrobo/arm/target_pose", 10,
    std::bind(&MotionGeneratorNode::onTargetPose, this, std::placeholders::_1));
  cmd_twist_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    "/catchrobo/arm/cmd_twist", 10,
    std::bind(&MotionGeneratorNode::onCmdTwist, this, std::placeholders::_1));
  gripper_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/catchrobo/arm/gripper", 10,
    std::bind(&MotionGeneratorNode::onGripper, this, std::placeholders::_1));
  orient_vertical_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/catchrobo/arm/orient_vertical", 10,
    std::bind(&MotionGeneratorNode::onOrientVertical, this, std::placeholders::_1));
  cancel_sub_ = create_subscription<std_msgs::msg::Empty>(
    "/catchrobo/arm/cancel", 10,
    std::bind(&MotionGeneratorNode::onCancel, this, std::placeholders::_1));
  init_request_sub_ = create_subscription<std_msgs::msg::Empty>(
    "/catchrobo/arm/init_request", 10,
    std::bind(&MotionGeneratorNode::onInitRequest, this, std::placeholders::_1));
  current_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "/catchrobo/arm/current_pose", 10,
    std::bind(&MotionGeneratorNode::onCurrentPose, this, std::placeholders::_1));
  // hardware_bridge_node 側が latched なので、こちらも transient_local で受ける
  // (後から起動しても最後の MCU 状態が届く)
  mcu_status_sub_ = create_subscription<sharmech_msgs::msg::McuStatus>(
    "/catchrobo/arm/mcu_status", rclcpp::QoS(1).transient_local(),
    std::bind(&MotionGeneratorNode::onMcuStatus, this, std::placeholders::_1));
  workspace_clamp_sub_ = create_subscription<sharmech_msgs::msg::WorkspaceClamp>(
    "/catchrobo/game/workspace_clamp", 10,
    std::bind(&MotionGeneratorNode::onWorkspaceClamp, this, std::placeholders::_1));
  jog_limit_sub_ = create_subscription<sharmech_msgs::msg::JogLimit>(
    "/catchrobo/game/jog_limit", 10,
    std::bind(&MotionGeneratorNode::onJogLimit, this, std::placeholders::_1));

  cartesian_pub_ = create_publisher<sharmech_msgs::msg::CartesianCommand>(
    "/catchrobo/command/cartesian", 10);
  gripper_pub_ = create_publisher<std_msgs::msg::Bool>(
    "/catchrobo/command/gripper", 10);
  orient_vertical_pub_ = create_publisher<std_msgs::msg::Bool>(
    "/catchrobo/command/orient_vertical", 10);

  // latched: 後から接続した VR クライアントにも現在状態が即座に届く
  status_pub_ = create_publisher<sharmech_msgs::msg::MotionStatus>(
    "/catchrobo/arm/status",
    rclcpp::QoS(1).transient_local());

  const auto control_period = std::chrono::duration<double>(1.0 / control_rate_);
  control_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(control_period),
    std::bind(&MotionGeneratorNode::onControlTimer, this));
  const auto status_period = std::chrono::duration<double>(1.0 / status_rate_);
  status_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(status_period),
    std::bind(&MotionGeneratorNode::onStatusTimer, this));

  RCLCPP_INFO(
    get_logger(), "motion_generator_node started (%.0f Hz, goal_mode=%s)",
    control_rate_, goal_mode_.c_str());
}

void MotionGeneratorNode::onTargetPose(
  const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  const auto pitch_yaw = OrientationUtils::toPitchYaw(msg->pose.orientation);
  CartesianState goal;
  goal.x = msg->pose.position.x;
  goal.y = msg->pose.position.y;
  goal.z = msg->pose.position.z;
  goal.pitch = pitch_yaw.pitch;
  goal.yaw = pitch_yaw.yaw;

  // 作業領域外のゴールはクランプせず却下する (黙って別の場所へ動くより安全)
  if (!isInsideWorkspace(goal.x, goal.y, goal.z)) {
    rejectGoal("goal outside workspace");
    return;
  }
  // 同期前は target_ が原点の仮値なので、ここから軌道を引くと始点が実姿勢と
  // 無関係になる。動作許可も 0 で送っている間なので、受理しても動かない
  if (!synced_with_feedback_) {
    rejectGoal("not synced with MCU feedback yet");
    return;
  }
  if (mode_ == Mode::kInit) {
    rejectGoal("init request in progress");
    return;
  }
  if (mode_ == Mode::kJog) {
    if (goal_mode_ != "goal_priority") {
      rejectGoal("jog active");
      return;
    }
    // goal_priority: ゴールがジョグを横取りする。ジョグの速度指令は捨て、
    // 減速中の速度も 0 にする (残しておくと次にジョグへ戻った瞬間に
    // 古い速度が積分されて飛ぶ)。始点は現在の指令姿勢 target_ なので段差は出ない
    commanded_twist_ = CartesianState{};
    current_twist_ = CartesianState{};
    RCLCPP_INFO(get_logger(), "Jog preempted by goal");
  }

  // 新しいゴールは実行中のゴールを上書きする。
  // 軌道の始点は現在指令中の目標姿勢 (このノードが唯一の所有者)
  trajectory_ = TrapezoidalTrajectory(
    target_, goal, v_max_, a_max_, w_max_, alpha_max_);
  trajectory_start_time_ = now();
  goal_ = goal;
  mode_ = Mode::kGoal;
  last_result_ = sharmech_msgs::msg::MotionStatus::RESULT_NONE;
  status_message_ = "executing";

  RCLCPP_INFO(
    get_logger(),
    "Goal accepted: (%.3f, %.3f, %.3f) pitch=%.2f yaw=%.2f, duration=%.2f s",
    goal.x, goal.y, goal.z, goal.pitch, goal.yaw, trajectory_.duration());
}

void MotionGeneratorNode::onCmdTwist(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  // 自動シーケンスの動作中は操縦層の入力を捨てる。
  // **ゼロも含めて記録しない** —— ここで記録すると mode_ が kJog へ落ちてしまう。
  // 既に動いている分は onControlTimer 側で commanded_twist_ が空のまま
  // レート制限に従って減速する
  if (jog_blocked_) {return;}
  // 同期前・初期位置要求中も同じ理由で捨てる (記録すると mode_ が kJog へ落ちる)
  if (!synced_with_feedback_ || mode_ == Mode::kInit) {return;}

  CartesianState twist;
  twist.x = msg->linear.x;
  twist.y = msg->linear.y;
  twist.z = msg->linear.z;
  twist.pitch = msg->angular.y;
  twist.yaw = msg->angular.z;

  // 並進の「大きさ」を上限で頭打ちにする (向きは変えない)。
  // **軸ごとに切ると斜め入力で合成速度が最大 √3 倍になる**ため、比を保って縮める。
  // 操縦層が上限を守ってくれることを前提にしない ——「誰が publish しても
  // この上限を超えられない」ことがこのノードの責務 (作業領域クランプと同じ)
  const double speed = std::sqrt(twist.x * twist.x + twist.y * twist.y + twist.z * twist.z);
  if (speed > active_jog_v_max_) {
    const double ratio = active_jog_v_max_ / speed;
    twist.x *= ratio;
    twist.y *= ratio;
    twist.z *= ratio;
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "cmd_twist %.2f m/s exceeds jog limit %.2f m/s; scaled down",
      speed, active_jog_v_max_);
  }

  // ゼロでない Twist のみがゴールに干渉する。
  // joy_teleop_node はニュートラルでもゼロ Twist を送り続けるため、
  // 「受信したら abort」と実装するとゴール指定が一切使えなくなる
  const bool active = isActive(twist);

  if ((goal_mode_ == "exclusive" || goal_mode_ == "goal_priority") &&
    mode_ == Mode::kGoal)
  {
    return;  // ゴール実行中は Twist を無視 (記録もしない)
  }

  commanded_twist_ = twist;
  last_twist_time_ = now();

  if (active) {
    if (mode_ == Mode::kGoal) {
      last_result_ = sharmech_msgs::msg::MotionStatus::RESULT_ABORTED;
      status_message_ = "preempted by jog";
      RCLCPP_INFO(get_logger(), "Goal aborted: preempted by jog");
    }
    if (mode_ != Mode::kJog) {
      mode_ = Mode::kJog;
    }
  }
}

void MotionGeneratorNode::onGripper(const std_msgs::msg::Bool::SharedPtr msg)
{
  gripper_state_ = msg->data;  // publish はタイマー内で行う
}

void MotionGeneratorNode::onOrientVertical(const std_msgs::msg::Bool::SharedPtr msg)
{
  orient_vertical_state_ = msg->data;  // publish はタイマー内で行う (グリッパと同じ扱い)
}

void MotionGeneratorNode::onCancel(const std_msgs::msg::Empty::SharedPtr)
{
  if (mode_ == Mode::kGoal) {
    mode_ = Mode::kIdle;  // target_ はその場で保持される
    last_result_ = sharmech_msgs::msg::MotionStatus::RESULT_ABORTED;
    status_message_ = "cancelled";
    RCLCPP_INFO(get_logger(), "Goal cancelled");
  } else if (mode_ == Mode::kInit) {
    // 初期位置要求を取り下げる。target_ は INIT 中ずっと実姿勢を追いかけて
    // いるので、bit2 が落ちた MCU はその場 (≒ 現在位置) に留まる。
    // MCU が到達を返さない (未対応ファーム等) ときの唯一の出口でもある
    mode_ = Mode::kIdle;
    last_result_ = sharmech_msgs::msg::MotionStatus::RESULT_ABORTED;
    status_message_ = "init request cancelled";
    RCLCPP_WARN(get_logger(), "Init request cancelled");
  }
}

// 初期位置要求 (/catchrobo/game/reset の実体)。**初期位置の座標はこのノードも
// game_state_manager_node も知らない。** control_flags bit2 を立てて MCU に
// 「自前の初期関節角へ行け」と頼み、到達 (McuStatus FLAG_AT_INIT_POSE) を待つだけ。
// 実機無しでも sim / mock_mcu が同じ契約でそれぞれの初期位置へ動く
void MotionGeneratorNode::onInitRequest(const std_msgs::msg::Empty::SharedPtr)
{
  if (mode_ == Mode::kGoal) {
    RCLCPP_INFO(get_logger(), "Goal aborted: preempted by init request");
  } else if (mode_ == Mode::kJog) {
    RCLCPP_INFO(get_logger(), "Jog preempted by init request");
  }
  commanded_twist_ = CartesianState{};
  current_twist_ = CartesianState{};
  mode_ = Mode::kInit;
  last_result_ = sharmech_msgs::msg::MotionStatus::RESULT_NONE;
  status_message_ = "moving to MCU init pose";
  RCLCPP_INFO(get_logger(), "Init request accepted (control_flags bit2)");
}

void MotionGeneratorNode::onCurrentPose(
  const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  // 記録のみ。同期の判定は制御タイマー側で行う (mcu_status と揃えて評価するため)
  latest_feedback_ = *msg;
}

void MotionGeneratorNode::onMcuStatus(const sharmech_msgs::msg::McuStatus::SharedPtr msg)
{
  latest_mcu_flags_ = msg->status_flags;

  // MCU が未初期化 (原点未確定) を報告したら同期を取り消す。起動直後のほか、
  // MCU だけが再起動した場合もここを通る。その間の目標姿勢は信用できないので
  // 実行中のゴール/ジョグは中断し、動作許可も 0 に戻る (制御タイマー)
  if ((msg->status_flags & sharmech_msgs::msg::McuStatus::FLAG_UNINITIALIZED) != 0) {
    if (synced_with_feedback_) {
      RCLCPP_WARN(get_logger(), "MCU reports uninitialized; target sync revoked");
    }
    synced_with_feedback_ = false;
    if (mode_ == Mode::kGoal || mode_ == Mode::kJog) {
      mode_ = Mode::kIdle;
      commanded_twist_ = CartesianState{};
      current_twist_ = CartesianState{};
      last_result_ = sharmech_msgs::msg::MotionStatus::RESULT_ABORTED;
      status_message_ = "MCU uninitialized";
    }
  }
}

std::optional<CartesianState> MotionGeneratorNode::feedbackState() const
{
  if (!latest_feedback_) {return std::nullopt;}
  const auto pitch_yaw = OrientationUtils::toPitchYaw(latest_feedback_->pose.orientation);
  CartesianState fb;
  fb.x = latest_feedback_->pose.position.x;
  fb.y = latest_feedback_->pose.position.y;
  fb.z = latest_feedback_->pose.position.z;
  fb.pitch = pitch_yaw.pitch;
  fb.yaw = pitch_yaw.yaw;
  return fb;
}

bool MotionGeneratorNode::mcuUninitialized() const
{
  return latest_mcu_flags_ &&
         (*latest_mcu_flags_ & sharmech_msgs::msg::McuStatus::FLAG_UNINITIALIZED) != 0;
}

// パラメータ (overrides があればそちらを優先) を実際のメンバへ反映する。
// 起動時とパラメータ変更時の両方から呼ぶ。失敗時は理由を返し、メンバは書き換えない
rcl_interfaces::msg::SetParametersResult MotionGeneratorNode::applyParameters(
  const std::vector<rclcpp::Parameter> & overrides)
{
  const auto find = [&overrides](const std::string & name) -> const rclcpp::Parameter * {
      for (const auto & p : overrides) {
        if (p.get_name() == name) {return &p;}
      }
      return nullptr;
    };
  const auto dbl = [&](const std::string & name) {
      const auto * o = find(name);
      return o ? o->as_double() : get_parameter(name).as_double();
    };
  const auto str = [&](const std::string & name) {
      const auto * o = find(name);
      return o ? o->as_string() : get_parameter(name).as_string();
    };

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = false;

  const std::string goal_mode = str("goal_mode");
  if (goal_mode != "twist_priority" && goal_mode != "exclusive" &&
    goal_mode != "goal_priority")
  {
    result.reason = "unknown goal_mode: " + goal_mode +
      " (goal_priority / twist_priority / exclusive のいずれか)";
    return result;
  }

  const double offset_x = dbl("field_origin_offset_x_m");
  const double offset_y = dbl("field_origin_offset_y_m");
  const double offset_z = dbl("field_origin_offset_z_m");
  const double x_min = dbl("workspace_x_min") + offset_x;
  const double x_max = dbl("workspace_x_max") + offset_x;
  const double y_min = dbl("workspace_y_min") + offset_y;
  const double y_max = dbl("workspace_y_max") + offset_y;
  const double z_min = dbl("workspace_z_min") + offset_z;
  const double z_max = dbl("workspace_z_max") + offset_z;
  if (x_min > x_max || y_min > y_max || z_min > z_max) {
    result.reason = "workspace min must not exceed max";
    return result;
  }

  const double v_max = dbl("v_max");
  const double a_max = dbl("a_max");
  const double w_max = dbl("w_max");
  const double alpha_max = dbl("alpha_max");
  const double twist_timeout = dbl("twist_timeout");
  const double jog_v_max = dbl("jog_v_max");
  if (v_max <= 0.0 || a_max <= 0.0 || w_max <= 0.0 || alpha_max <= 0.0 ||
    twist_timeout <= 0.0 || jog_v_max <= 0.0)
  {
    result.reason =
      "v_max / a_max / w_max / alpha_max / twist_timeout / jog_v_max must be positive";
    return result;
  }

  goal_mode_ = goal_mode;
  v_max_ = v_max;
  a_max_ = a_max;
  w_max_ = w_max;
  alpha_max_ = alpha_max;
  twist_timeout_ = twist_timeout;
  // 上限を下げたら、現在有効な上限もそれを超えないように追従させる
  // (作業領域クランプと同じ考え方。上書き中の値が起動時の上限を超えないこと)
  const bool jog_limit_was_default = (active_jog_v_max_ == jog_v_max_);
  jog_v_max_ = jog_v_max;
  active_jog_v_max_ =
    jog_limit_was_default ? jog_v_max_ : std::min(active_jog_v_max_, jog_v_max_);
  workspace_x_min_ = x_min;
  workspace_x_max_ = x_max;
  workspace_y_min_ = y_min;
  workspace_y_max_ = y_max;
  workspace_z_min_ = z_min;
  workspace_z_max_ = z_max;

  // 現在有効な領域を新しい上限下限へ収め直す。
  // **クランプ中 (game_state_manager_node が縮めている最中) でもその狭さを保ったまま、
  // 「active は必ず workspace の内側」という安全条件だけを守る**
  active_workspace_x_min_ = std::clamp(active_workspace_x_min_, x_min, x_max);
  active_workspace_x_max_ = std::clamp(active_workspace_x_max_, x_min, x_max);
  active_workspace_y_min_ = std::clamp(active_workspace_y_min_, y_min, y_max);
  active_workspace_y_max_ = std::clamp(active_workspace_y_max_, y_min, y_max);
  active_workspace_z_min_ = std::clamp(active_workspace_z_min_, z_min, z_max);
  active_workspace_z_max_ = std::clamp(active_workspace_z_max_, z_min, z_max);

  result.successful = true;
  return result;
}

// 実行中のパラメータ変更。弾かれた場合は直前の設定のまま動き続ける
rcl_interfaces::msg::SetParametersResult MotionGeneratorNode::onSetParameters(
  const std::vector<rclcpp::Parameter> & parameters)
{
  const auto result = applyParameters(parameters);
  if (result.successful) {
    RCLCPP_INFO(
      get_logger(),
      "parameters updated at runtime (%zu changed); v_max=%.3f a_max=%.3f "
      "workspace z=[%.3f, %.3f] goal_mode=%s",
      parameters.size(), v_max_, a_max_, workspace_z_min_, workspace_z_max_,
      goal_mode_.c_str());
  } else {
    RCLCPP_WARN(get_logger(), "parameter update rejected: %s", result.reason.c_str());
  }
  return result;
}

// ジョグ速度上限の動的上書き (game_state_manager_node)。
// 微調整中 (ADJUSTING_*) だけ遅くする、といった場面ごとの調整に使う。
// **起動時パラメータ jog_v_max より緩くはできない** (上書きで安全側の設定を
// 壊せないようにする。作業領域クランプと同じ方針)
void MotionGeneratorNode::onJogLimit(const sharmech_msgs::msg::JogLimit::SharedPtr msg)
{
  if (msg->reset) {
    active_jog_v_max_ = jog_v_max_;
    jog_blocked_ = false;
    RCLCPP_INFO(get_logger(), "Jog limit reset to %.3f m/s", active_jog_v_max_);
    return;
  }
  if (msg->block) {
    // 現在のジョグも止める (指令を空にしてレート制限で減速させる)
    jog_blocked_ = true;
    commanded_twist_ = CartesianState{};
    RCLCPP_INFO(get_logger(), "Jog blocked (automated sequence in progress)");
    return;
  }
  jog_blocked_ = false;
  if (!(msg->v_max > 0.0)) {
    // 0 や NaN を通すと「ジョグが一切効かない」状態を無言で作ってしまう
    RCLCPP_WARN(
      get_logger(), "Ignoring jog limit with non-positive v_max (%.3f)", msg->v_max);
    return;
  }
  active_jog_v_max_ = std::min(msg->v_max, jog_v_max_);
  RCLCPP_INFO(get_logger(), "Jog limit overridden to %.3f m/s", active_jog_v_max_);
}

void MotionGeneratorNode::onWorkspaceClamp(
  const sharmech_msgs::msg::WorkspaceClamp::SharedPtr msg)
{
  if (msg->reset) {
    active_workspace_x_min_ = workspace_x_min_;
    active_workspace_x_max_ = workspace_x_max_;
    active_workspace_y_min_ = workspace_y_min_;
    active_workspace_y_max_ = workspace_y_max_;
    active_workspace_z_min_ = workspace_z_min_;
    active_workspace_z_max_ = workspace_z_max_;
    RCLCPP_INFO(get_logger(), "Workspace clamp reset to default");
    return;
  }

  // 上書き値も config.yaml のデフォルト範囲を超えないようにする。
  // game_state_manager_node のバグで安全域が丸ごと外れることを防ぐ
  active_workspace_x_min_ = std::clamp(msg->x_min, workspace_x_min_, workspace_x_max_);
  active_workspace_x_max_ = std::clamp(msg->x_max, workspace_x_min_, workspace_x_max_);
  active_workspace_y_min_ = std::clamp(msg->y_min, workspace_y_min_, workspace_y_max_);
  active_workspace_y_max_ = std::clamp(msg->y_max, workspace_y_min_, workspace_y_max_);
  active_workspace_z_min_ = std::clamp(msg->z_min, workspace_z_min_, workspace_z_max_);
  active_workspace_z_max_ = std::clamp(msg->z_max, workspace_z_min_, workspace_z_max_);
  RCLCPP_INFO(
    get_logger(),
    "Workspace clamp overridden: x=[%.3f, %.3f] y=[%.3f, %.3f] z=[%.3f, %.3f]",
    active_workspace_x_min_, active_workspace_x_max_,
    active_workspace_y_min_, active_workspace_y_max_,
    active_workspace_z_min_, active_workspace_z_max_);
}

void MotionGeneratorNode::onControlTimer()
{
  const double dt = 1.0 / control_rate_;
  const auto current_time = now();

  // 0. 起動時の同期: まだ何も動かしていなければ目標姿勢を実姿勢に合わせる。
  // これが済むまで動作許可 (bit0) は 0 で送るので MCU は動かない。
  // **MCU が未初期化 (bit3) を報告している間は同期しない** —— 原点出し中の
  // 仮の値に同期すると、確定後の実姿勢とずれた目標から動き出すことになる
  if (!synced_with_feedback_ && mode_ == Mode::kIdle && !mcuUninitialized()) {
    if (const auto fb = feedbackState()) {
      target_ = clampToWorkspace(*fb);
      synced_with_feedback_ = true;
      RCLCPP_INFO(
        get_logger(),
        "Target synced to MCU feedback: (%.3f, %.3f, %.3f); motion enabled",
        fb->x, fb->y, fb->z);
    }
  }

  // 1. ウォッチドッグ: Twist が途絶したら速度指令を 0 とみなす
  if (last_twist_time_ &&
    (current_time - *last_twist_time_).seconds() > twist_timeout_)
  {
    if (isActive(commanded_twist_)) {
      RCLCPP_WARN(get_logger(), "Twist watchdog fired; decelerating to stop");
    }
    commanded_twist_ = CartesianState{};
  }

  // 2. レート制限 (加速・減速の両方に適用)
  current_twist_.x = approach(current_twist_.x, commanded_twist_.x, a_max_ * dt);
  current_twist_.y = approach(current_twist_.y, commanded_twist_.y, a_max_ * dt);
  current_twist_.z = approach(current_twist_.z, commanded_twist_.z, a_max_ * dt);
  current_twist_.pitch =
    approach(current_twist_.pitch, commanded_twist_.pitch, alpha_max_ * dt);
  current_twist_.yaw =
    approach(current_twist_.yaw, commanded_twist_.yaw, alpha_max_ * dt);

  // 3. モード別に target_ / target_vel_ を更新 (書き換えるのはここだけ)
  switch (mode_) {
    case Mode::kJog: {
        target_.x += current_twist_.x * dt;
        target_.y += current_twist_.y * dt;
        target_.z += current_twist_.z * dt;
        target_.pitch += current_twist_.pitch * dt;
        target_.yaw += current_twist_.yaw * dt;
        target_vel_ = current_twist_;

        // 作業領域クランプ。クランプが効いた軸は速度も 0 にする。
        // 位置は境界で止まっているのに速度を報告すると MCU が外挿して
        // 領域外へはみ出すため、位置と速度は常に整合させる
        const CartesianState clamped = clampToWorkspace(target_);
        if (clamped.x != target_.x) {target_vel_.x = 0.0; current_twist_.x = 0.0;}
        if (clamped.y != target_.y) {target_vel_.y = 0.0; current_twist_.y = 0.0;}
        if (clamped.z != target_.z) {target_vel_.z = 0.0; current_twist_.z = 0.0;}
        target_ = clamped;

        if (!isActive(current_twist_) && !isActive(commanded_twist_)) {
          mode_ = Mode::kIdle;
        }
        break;
      }
    case Mode::kGoal: {
        // 軌道は経過時間で評価する (インデックスを進めてはならない)
        const double t = (current_time - trajectory_start_time_).seconds();
        trajectory_.sample(t, target_, target_vel_);
        if (t >= trajectory_.duration()) {
          mode_ = Mode::kIdle;
          last_result_ = sharmech_msgs::msg::MotionStatus::RESULT_SUCCEEDED;
          status_message_ = "reached";
          RCLCPP_INFO(get_logger(), "Goal reached");
        }
        break;
      }
    case Mode::kInit: {
        // 動かしているのは MCU。こちらは実姿勢を追いかけて、bit2 が落ちた瞬間に
        // 指令と実姿勢が一致している状態を作る (落とした途端に飛ばないため)。
        // 作業領域クランプは掛けない —— MCU の初期位置が ROS2 側の作業領域の
        // 外にあっても、そこに居る事実は変えられない
        if (const auto fb = feedbackState()) {target_ = *fb;}
        target_vel_ = CartesianState{};
        const bool at_init = latest_mcu_flags_ &&
          (*latest_mcu_flags_ & sharmech_msgs::msg::McuStatus::FLAG_AT_INIT_POSE) != 0;
        if (at_init) {
          mode_ = Mode::kIdle;
          synced_with_feedback_ = true;
          last_result_ = sharmech_msgs::msg::MotionStatus::RESULT_SUCCEEDED;
          status_message_ = "init pose reached";
          RCLCPP_INFO(
            get_logger(), "MCU init pose reached: (%.3f, %.3f, %.3f)",
            target_.x, target_.y, target_.z);
        } else if (!latest_mcu_flags_) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "Init request pending but no /catchrobo/arm/mcu_status received; "
            "cannot detect arrival (cancel with /catchrobo/arm/cancel)");
        }
        break;
      }
    case Mode::kIdle:
      target_vel_ = CartesianState{};
      break;
  }

  // 4. publish
  sharmech_msgs::msg::CartesianCommand cmd;
  cmd.header.stamp = current_time;
  cmd.header.frame_id = "field";
  cmd.pose = toPoseMsg(target_);
  cmd.twist.linear.x = target_vel_.x;
  cmd.twist.linear.y = target_vel_.y;
  cmd.twist.linear.z = target_vel_.z;
  cmd.twist.angular.y = target_vel_.pitch;
  cmd.twist.angular.z = target_vel_.yaw;
  // 動作許可: 同期済みか、初期位置要求中 (MCU が自力で動く必要がある) のみ true。
  // 初期位置要求: INIT モードの間だけ true (レベル。毎周期送る)。
  // 位置と同じメッセージに載せる理由は CartesianCommand.msg のコメント参照
  cmd.enable = synced_with_feedback_ || mode_ == Mode::kInit;
  cmd.init_request = (mode_ == Mode::kInit);
  cartesian_pub_->publish(cmd);

  std_msgs::msg::Bool gripper_msg;
  gripper_msg.data = gripper_state_;
  gripper_pub_->publish(gripper_msg);

  std_msgs::msg::Bool orient_vertical_msg;
  orient_vertical_msg.data = orient_vertical_state_;
  orient_vertical_pub_->publish(orient_vertical_msg);
}

void MotionGeneratorNode::onStatusTimer()
{
  sharmech_msgs::msg::MotionStatus status;
  status.header.stamp = now();
  status.header.frame_id = "field";
  status.mode = static_cast<uint8_t>(mode_);
  status.last_result = last_result_;
  status.message = status_message_;

  if (mode_ == Mode::kGoal) {
    status.goal_pose = toPoseMsg(goal_);
    // 残距離は実姿勢 (フィードバックがあれば) と目標の差。無ければ指令値基準
    double from_x = target_.x, from_y = target_.y, from_z = target_.z;
    if (latest_feedback_) {
      from_x = latest_feedback_->pose.position.x;
      from_y = latest_feedback_->pose.position.y;
      from_z = latest_feedback_->pose.position.z;
    }
    status.distance_remaining = std::sqrt(
      (goal_.x - from_x) * (goal_.x - from_x) +
      (goal_.y - from_y) * (goal_.y - from_y) +
      (goal_.z - from_z) * (goal_.z - from_z));
    const double elapsed = (now() - trajectory_start_time_).seconds();
    status.time_remaining = std::max(0.0, trajectory_.duration() - elapsed);
  }

  status_pub_->publish(status);
}

void MotionGeneratorNode::rejectGoal(const std::string & reason)
{
  // トピック経由のゴールは送信元に直接返せないため、
  // 却下は /catchrobo/arm/status の last_result / message で伝える
  last_result_ = sharmech_msgs::msg::MotionStatus::RESULT_REJECTED;
  status_message_ = reason;
  RCLCPP_WARN(get_logger(), "Goal rejected: %s", reason.c_str());
}

bool MotionGeneratorNode::isInsideWorkspace(double x, double y, double z) const
{
  return x >= active_workspace_x_min_ && x <= active_workspace_x_max_ &&
         y >= active_workspace_y_min_ && y <= active_workspace_y_max_ &&
         z >= active_workspace_z_min_ && z <= active_workspace_z_max_;
}

CartesianState MotionGeneratorNode::clampToWorkspace(const CartesianState & state) const
{
  CartesianState clamped = state;
  clamped.x = std::clamp(state.x, active_workspace_x_min_, active_workspace_x_max_);
  clamped.y = std::clamp(state.y, active_workspace_y_min_, active_workspace_y_max_);
  clamped.z = std::clamp(state.z, active_workspace_z_min_, active_workspace_z_max_);
  return clamped;
}

}  // namespace sharmech_core

RCLCPP_COMPONENTS_REGISTER_NODE(sharmech_core::MotionGeneratorNode)
