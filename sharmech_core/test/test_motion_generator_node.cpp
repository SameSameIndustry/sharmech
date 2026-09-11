// motion_generator_node のウォッチドッグ・作業領域クランプ・入力調停ロジックの検証。
//
// sharmech_core/docs/motion_generator_node.md に列挙されている「罠」
// (ゼロ Twist はゴールを abort しない、クランプ時は速度も0にする、等) は
// コールバックとタイマーに分散した状態遷移なので、ノードとして起動し
// トピック経由で入出力させないと検証できない。

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <thread>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <sharmech_msgs/msg/cartesian_command.hpp>
#include <sharmech_msgs/msg/mcu_status.hpp>
#include <sharmech_msgs/msg/motion_status.hpp>
#include <sharmech_msgs/msg/workspace_clamp.hpp>
#include <sharmech_msgs/msg/jog_limit.hpp>

#include "sharmech_core/motion_generator_node.hpp"

using namespace std::chrono_literals;

namespace
{

geometry_msgs::msg::PoseStamped makePose(double x, double y, double z, double yaw = 0.0)
{
  geometry_msgs::msg::PoseStamped msg;
  msg.header.frame_id = "field";
  msg.pose.position.x = x;
  msg.pose.position.y = y;
  msg.pose.position.z = z;
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  msg.pose.orientation = tf2::toMsg(q);
  return msg;
}

// テストハーネット: motion_generator_node の入力を publish し、出力を購読する
class TestHarness
{
public:
  explicit TestHarness(const std::string & suffix)
  {
    node_ = std::make_shared<rclcpp::Node>("test_harness_" + suffix);

    target_pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/catchrobo/arm/target_pose", 10);
    cmd_twist_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(
      "/catchrobo/arm/cmd_twist", 10);
    workspace_clamp_pub_ = node_->create_publisher<sharmech_msgs::msg::WorkspaceClamp>(
      "/catchrobo/game/workspace_clamp", 10);
    jog_limit_pub_ = node_->create_publisher<sharmech_msgs::msg::JogLimit>(
      "/catchrobo/game/jog_limit", 10);
    // MCU 側 (hardware_bridge_node) の代役: 実姿勢と status_flags
    current_pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/catchrobo/arm/current_pose", 10);
    mcu_status_pub_ = node_->create_publisher<sharmech_msgs::msg::McuStatus>(
      "/catchrobo/arm/mcu_status", rclcpp::QoS(1).transient_local());
    init_request_pub_ = node_->create_publisher<std_msgs::msg::Empty>(
      "/catchrobo/arm/init_request", 10);
    cancel_pub_ = node_->create_publisher<std_msgs::msg::Empty>(
      "/catchrobo/arm/cancel", 10);


    cartesian_sub_ = node_->create_subscription<sharmech_msgs::msg::CartesianCommand>(
      "/catchrobo/command/cartesian", 10,
      [this](sharmech_msgs::msg::CartesianCommand::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        latest_cartesian_ = *msg;
      });
    status_sub_ = node_->create_subscription<sharmech_msgs::msg::MotionStatus>(
      "/catchrobo/arm/status", rclcpp::QoS(1).transient_local(),
      [this](sharmech_msgs::msg::MotionStatus::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        latest_status_ = *msg;
      });
  }

  rclcpp::Node::SharedPtr node() {return node_;}

  void publishTargetPose(double x, double y, double z, double yaw = 0.0)
  {
    target_pose_pub_->publish(makePose(x, y, z, yaw));
  }

  void publishTwist(double vx, double vy, double vz)
  {
    geometry_msgs::msg::Twist msg;
    msg.linear.x = vx;
    msg.linear.y = vy;
    msg.linear.z = vz;
    cmd_twist_pub_->publish(msg);
  }

  void publishWorkspaceClampOverride(
    double x_min, double x_max, double y_min, double y_max,
    double z_min, double z_max)
  {
    sharmech_msgs::msg::WorkspaceClamp msg;
    msg.reset = false;
    msg.x_min = x_min;
    msg.x_max = x_max;
    msg.y_min = y_min;
    msg.y_max = y_max;
    msg.z_min = z_min;
    msg.z_max = z_max;
    workspace_clamp_pub_->publish(msg);
  }

  void publishJogLimit(double v_max)
  {
    sharmech_msgs::msg::JogLimit msg;
    msg.reset = false;
    msg.v_max = v_max;
    jog_limit_pub_->publish(msg);
  }

  void publishJogLimitReset()
  {
    sharmech_msgs::msg::JogLimit msg;
    msg.reset = true;
    jog_limit_pub_->publish(msg);
  }

  void publishJogBlock()
  {
    sharmech_msgs::msg::JogLimit msg;
    msg.reset = false;
    msg.block = true;
    jog_limit_pub_->publish(msg);
  }

  void publishWorkspaceClampReset()
  {
    sharmech_msgs::msg::WorkspaceClamp msg;
    msg.reset = true;
    workspace_clamp_pub_->publish(msg);
  }

  // MCU の代役として実姿勢と status_flags を1組送る
  void publishFeedback(double x, double y, double z, uint16_t status_flags = 0)
  {
    current_pose_pub_->publish(makePose(x, y, z));
    sharmech_msgs::msg::McuStatus status;
    status.connected = true;
    status.status_flags = status_flags;
    mcu_status_pub_->publish(status);
  }

  void publishMcuStatus(uint16_t status_flags)
  {
    sharmech_msgs::msg::McuStatus status;
    status.connected = true;
    status.status_flags = status_flags;
    mcu_status_pub_->publish(status);
  }

  void publishInitRequest() {init_request_pub_->publish(std_msgs::msg::Empty{});}
  void publishCancel() {cancel_pub_->publish(std_msgs::msg::Empty{});}

  // 動作許可 / 初期位置要求は Cartesian ストリームに同乗する
  std::optional<bool> latestEnable()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!latest_cartesian_) {return std::nullopt;}
    return latest_cartesian_->enable;
  }

  std::optional<bool> latestInitRequest()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!latest_cartesian_) {return std::nullopt;}
    return latest_cartesian_->init_request;
  }

  std::optional<sharmech_msgs::msg::MotionStatus> latestStatus()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return latest_status_;
  }

  std::optional<sharmech_msgs::msg::CartesianCommand> latestCartesian()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return latest_cartesian_;
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_twist_pub_;
  rclcpp::Publisher<sharmech_msgs::msg::WorkspaceClamp>::SharedPtr workspace_clamp_pub_;
  rclcpp::Publisher<sharmech_msgs::msg::JogLimit>::SharedPtr jog_limit_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_pub_;
  rclcpp::Publisher<sharmech_msgs::msg::McuStatus>::SharedPtr mcu_status_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr init_request_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr cancel_pub_;
  rclcpp::Subscription<sharmech_msgs::msg::CartesianCommand>::SharedPtr cartesian_sub_;
  rclcpp::Subscription<sharmech_msgs::msg::MotionStatus>::SharedPtr status_sub_;

  std::mutex mutex_;
  std::optional<sharmech_msgs::msg::CartesianCommand> latest_cartesian_;
  std::optional<sharmech_msgs::msg::MotionStatus> latest_status_;
};

// predicate() が真になるまで、両ノードを spin しながら待つ
bool waitUntil(
  rclcpp::executors::SingleThreadedExecutor & executor,
  const std::function<bool()> & predicate, double timeout_sec)
{
  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(timeout_sec));
  while (std::chrono::steady_clock::now() < deadline) {
    executor.spin_some();
    if (predicate()) {return true;}
    std::this_thread::sleep_for(2ms);
  }
  return predicate();
}

// 起動直後の motion_generator_node は MCU フィードバックに同期するまで
// 動作許可 (command/enable) を false で送り、ゴールもジョグも受け付けない。
// 各テストは MCU の代役として実姿勢を1度流し、enable=true になるまで待ってから始める
bool syncWithFeedback(
  TestHarness & harness, rclcpp::executors::SingleThreadedExecutor & executor,
  double x = 0.0, double y = 0.0, double z = 0.0)
{
  return waitUntil(
    executor, [&]() {
      harness.publishFeedback(x, y, z);
      auto enable = harness.latestEnable();
      return enable && *enable;
    }, 2.0);
}

rclcpp::NodeOptions fastTestOptions(
  const std::vector<rclcpp::Parameter> & extra_overrides = {})
{
  std::vector<rclcpp::Parameter> overrides{
    rclcpp::Parameter("control_rate", 500.0),
    rclcpp::Parameter("status_rate", 100.0),
    rclcpp::Parameter("twist_timeout", 0.1),
  };
  overrides.insert(overrides.end(), extra_overrides.begin(), extra_overrides.end());
  rclcpp::NodeOptions options;
  options.parameter_overrides(overrides);
  return options;
}

}  // namespace

TEST(MotionGeneratorNode, GoalOutsideWorkspaceIsRejected)
{
  TestHarness harness("reject");
  auto motion_node = std::make_shared<sharmech_core::MotionGeneratorNode>(fastTestOptions());

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(harness.node());
  executor.add_node(motion_node);
  ASSERT_TRUE(syncWithFeedback(harness, executor));

  // workspace_x_max の既定値は 0.20 m。大きく外れた値は却下される
  harness.publishTargetPose(999.0, 0.1, 0.1);

  const bool got_rejected = waitUntil(
    executor, [&harness]() {
      auto status = harness.latestStatus();
      return status &&
      status->last_result == sharmech_msgs::msg::MotionStatus::RESULT_REJECTED;
    }, 2.0);

  ASSERT_TRUE(got_rejected);
  EXPECT_NE(harness.latestStatus()->message.find("workspace"), std::string::npos);
}

TEST(MotionGeneratorNode, ZeroTwistDoesNotAbortGoalAndGoalSucceeds)
{
  TestHarness harness("zerotwist");
  auto motion_node = std::make_shared<sharmech_core::MotionGeneratorNode>(fastTestOptions());

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(harness.node());
  executor.add_node(motion_node);
  ASSERT_TRUE(syncWithFeedback(harness, executor));

  // 開始姿勢からの距離 0.02m (三角プロファイル、duration ≈ 0.63s)
  harness.publishTargetPose(0.02, 0.05, 0.0);
  ASSERT_TRUE(
    waitUntil(
      executor, [&harness]() {
        auto s = harness.latestStatus();
        return s && s->mode == sharmech_msgs::msg::MotionStatus::MODE_GOAL;
      }, 1.0));

  // joy_teleop_node はニュートラルでもゼロ Twist を送り続ける。
  // これを模擬してゴールが abort されないことを確認する
  for (int i = 0; i < 10; ++i) {
    harness.publishTwist(0.0, 0.0, 0.0);
    executor.spin_some();
    std::this_thread::sleep_for(10ms);
  }
  auto status_after_zero_twist = harness.latestStatus();
  ASSERT_TRUE(status_after_zero_twist.has_value());
  EXPECT_EQ(status_after_zero_twist->mode, sharmech_msgs::msg::MotionStatus::MODE_GOAL);
  EXPECT_NE(status_after_zero_twist->last_result, sharmech_msgs::msg::MotionStatus::RESULT_ABORTED);

  const bool succeeded = waitUntil(
    executor, [&harness]() {
      auto s = harness.latestStatus();
      return s && s->last_result == sharmech_msgs::msg::MotionStatus::RESULT_SUCCEEDED;
    }, 2.0);
  EXPECT_TRUE(succeeded);
}

// twist_priority (旧既定) だけの挙動。既定は goal_priority なので明示する
TEST(MotionGeneratorNode, NonZeroTwistAbortsActiveGoalInTwistPriorityMode)
{
  TestHarness harness("aborttwist");
  auto motion_node = std::make_shared<sharmech_core::MotionGeneratorNode>(
    fastTestOptions({rclcpp::Parameter("goal_mode", "twist_priority")}));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(harness.node());
  executor.add_node(motion_node);
  ASSERT_TRUE(syncWithFeedback(harness, executor));

  harness.publishTargetPose(0.15, 0.2, 0.1);
  ASSERT_TRUE(
    waitUntil(
      executor, [&harness]() {
        auto s = harness.latestStatus();
        return s && s->mode == sharmech_msgs::msg::MotionStatus::MODE_GOAL;
      }, 1.0));

  harness.publishTwist(0.05, 0.0, 0.0);

  const bool aborted = waitUntil(
    executor, [&harness]() {
      auto s = harness.latestStatus();
      return s && s->mode == sharmech_msgs::msg::MotionStatus::MODE_JOG &&
      s->last_result == sharmech_msgs::msg::MotionStatus::RESULT_ABORTED;
    }, 1.0);
  ASSERT_TRUE(aborted);
  EXPECT_NE(harness.latestStatus()->message.find("jog"), std::string::npos);
}

// goal_priority (既定): ゴール実行中の非ゼロ Twist はゴールを abort しない。
// WebXR が毎フレーム流す cmd_twist に左手の微小な上下動が混ざっても、
// game_state_manager_node の接近ゴールが途中で落ちないための回帰
TEST(MotionGeneratorNode, GoalPriorityIgnoresNonZeroTwistDuringGoal)
{
  TestHarness harness("goalpriority_twist");
  auto motion_node = std::make_shared<sharmech_core::MotionGeneratorNode>(fastTestOptions());

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(harness.node());
  executor.add_node(motion_node);
  ASSERT_TRUE(syncWithFeedback(harness, executor));

  harness.publishTargetPose(0.02, 0.05, 0.0);
  ASSERT_TRUE(
    waitUntil(
      executor, [&harness]() {
        auto s = harness.latestStatus();
        return s && s->mode == sharmech_msgs::msg::MotionStatus::MODE_GOAL;
      }, 1.0));

  for (int i = 0; i < 10; ++i) {
    harness.publishTwist(0.05, 0.0, 0.03);
    executor.spin_some();
    std::this_thread::sleep_for(10ms);
  }
  auto status = harness.latestStatus();
  ASSERT_TRUE(status.has_value());
  EXPECT_EQ(status->mode, sharmech_msgs::msg::MotionStatus::MODE_GOAL);
  EXPECT_NE(status->last_result, sharmech_msgs::msg::MotionStatus::RESULT_ABORTED);

  EXPECT_TRUE(
    waitUntil(
      executor, [&harness]() {
        auto s = harness.latestStatus();
        return s && s->last_result == sharmech_msgs::msg::MotionStatus::RESULT_SUCCEEDED;
      }, 2.0));
}

// goal_priority (既定): ジョグ中に来たゴールは却下せず、ジョグを止めて受理する。
// 減速しきる前 (a_max のレート制限で数秒かかる) に pick_request 由来のゴールが
// 来ても "jog active" で落ちないための回帰
TEST(MotionGeneratorNode, GoalPriorityGoalPreemptsActiveJog)
{
  TestHarness harness("goalpriority_jog");
  auto motion_node = std::make_shared<sharmech_core::MotionGeneratorNode>(fastTestOptions());

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(harness.node());
  executor.add_node(motion_node);
  ASSERT_TRUE(syncWithFeedback(harness, executor));

  harness.publishTwist(0.05, 0.0, 0.0);
  ASSERT_TRUE(
    waitUntil(
      executor, [&harness]() {
        auto s = harness.latestStatus();
        return s && s->mode == sharmech_msgs::msg::MotionStatus::MODE_JOG;
      }, 1.0));

  harness.publishTargetPose(0.02, 0.05, 0.0);
  const bool accepted = waitUntil(
    executor, [&harness]() {
      auto s = harness.latestStatus();
      return s && s->mode == sharmech_msgs::msg::MotionStatus::MODE_GOAL;
    }, 1.0);
  ASSERT_TRUE(accepted);
  EXPECT_NE(
    harness.latestStatus()->last_result,
    sharmech_msgs::msg::MotionStatus::RESULT_REJECTED);

  EXPECT_TRUE(
    waitUntil(
      executor, [&harness]() {
        auto s = harness.latestStatus();
        return s && s->last_result == sharmech_msgs::msg::MotionStatus::RESULT_SUCCEEDED;
      }, 2.0));
}

TEST(MotionGeneratorNode, WatchdogDeceleratesJogToIdleWhenTwistStalls)
{
  TestHarness harness("watchdog");
  auto motion_node = std::make_shared<sharmech_core::MotionGeneratorNode>(fastTestOptions());

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(harness.node());
  executor.add_node(motion_node);
  ASSERT_TRUE(syncWithFeedback(harness, executor));

  // 一度だけ Twist を送り、以後は送らない (joy_timeout 相当の途絶を模擬)
  harness.publishTwist(0.05, 0.0, 0.0);
  ASSERT_TRUE(
    waitUntil(
      executor, [&harness]() {
        auto s = harness.latestStatus();
        return s && s->mode == sharmech_msgs::msg::MotionStatus::MODE_JOG;
      }, 1.0));

  // twist_timeout (0.1s) を過ぎればウォッチドッグが作動し、
  // レート制限に従って減速したのち IDLE へ戻る
  const bool idle_again = waitUntil(
    executor, [&harness]() {
      auto s = harness.latestStatus();
      return s && s->mode == sharmech_msgs::msg::MotionStatus::MODE_IDLE;
    }, 2.0);
  ASSERT_TRUE(idle_again);

  auto cmd = harness.latestCartesian();
  ASSERT_TRUE(cmd.has_value());
  EXPECT_NEAR(cmd->twist.linear.x, 0.0, 1e-3);
}

TEST(MotionGeneratorNode, WorkspaceClampZeroesVelocityOnClampedAxis)
{
  // 作業領域をこのテストだけ狭くし、素早く境界に到達させる
  TestHarness harness("clamp");
  auto motion_node = std::make_shared<sharmech_core::MotionGeneratorNode>(
    fastTestOptions(
  {
    rclcpp::Parameter("workspace_x_min", -0.05),
    rclcpp::Parameter("workspace_x_max", 0.05),
    rclcpp::Parameter("a_max", 1.0),
  }));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(harness.node());
  executor.add_node(motion_node);
  ASSERT_TRUE(syncWithFeedback(harness, executor));

  // joy_teleop_node のように高頻度で送り続け、ウォッチドッグを回避しつつ
  // 境界まで押し込む。control_timer (500Hz = 2ms周期) を取りこぼさないよう
  // spin_some を短い間隔で回す (長いスリープを挟むと実時間に対してタイマーの
  // 発火回数が減り、内部の dt 前提とずれて収束が遅くなる)
  const auto deadline = std::chrono::steady_clock::now() + 3000ms;
  bool clamped = false;
  while (std::chrono::steady_clock::now() < deadline) {
    harness.publishTwist(0.1, 0.0, 0.0);
    executor.spin_some();
    std::this_thread::sleep_for(1ms);

    auto cmd = harness.latestCartesian();
    if (cmd && cmd->pose.position.x >= 0.05 - 1e-3 && cmd->twist.linear.x == 0.0) {
      clamped = true;
      break;
    }
  }

  ASSERT_TRUE(clamped);
  auto cmd = harness.latestCartesian();
  ASSERT_TRUE(cmd.has_value());
  EXPECT_NEAR(cmd->pose.position.x, 0.05, 1e-3);
  EXPECT_EQ(cmd->twist.linear.x, 0.0);

  // クランプされても入力は非ゼロのまま送り続けているので JOG は継続する
  auto status = harness.latestStatus();
  ASSERT_TRUE(status.has_value());
  EXPECT_EQ(status->mode, sharmech_msgs::msg::MotionStatus::MODE_JOG);
}

TEST(MotionGeneratorNode, FieldOriginOffsetShiftsDefaultWorkspace)
{
  // workspace_x_max=0.20 に対し offset_x=+0.5 を与えると、実効上限は 0.70 になる。
  // 0.20 を超えるがoffset込みの上限以下のゴールは受理され、それを超えるゴールは却下される
  TestHarness harness("originoffset");
  auto motion_node = std::make_shared<sharmech_core::MotionGeneratorNode>(
    fastTestOptions(
  {
    rclcpp::Parameter("field_origin_offset_x_m", 0.5),
  }));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(harness.node());
  executor.add_node(motion_node);
  ASSERT_TRUE(syncWithFeedback(harness, executor));

  // offset無しなら却下されるはずの 0.30 が、offset込みなら受理される
  harness.publishTargetPose(0.30, 0.1, 0.1);
  ASSERT_TRUE(
    waitUntil(
      executor, [&harness]() {
        auto s = harness.latestStatus();
        return s && s->mode == sharmech_msgs::msg::MotionStatus::MODE_GOAL;
      }, 2.0));

  // offset込みの上限(0.70)を超えるゴールは却下される
  harness.publishTargetPose(0.71, 0.1, 0.1);
  ASSERT_TRUE(
    waitUntil(
      executor, [&harness]() {
        auto s = harness.latestStatus();
        return s && s->last_result == sharmech_msgs::msg::MotionStatus::RESULT_REJECTED;
      }, 2.0));
}

TEST(MotionGeneratorNode, WorkspaceClampOverrideRejectsGoalOutsideOverride)
{
  // config.yaml のデフォルトでは通る目標だが、game_state_manager_node が
  // PLACING 用に絞ったクランプの外側にあるため却下されるはず
  TestHarness harness("clampoverride");
  auto motion_node = std::make_shared<sharmech_core::MotionGeneratorNode>(fastTestOptions());

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(harness.node());
  executor.add_node(motion_node);
  ASSERT_TRUE(syncWithFeedback(harness, executor));

  harness.publishWorkspaceClampOverride(-0.05, 0.05, 0.10, 0.15, 0.0, 0.05);
  ASSERT_TRUE(
    waitUntil(
      executor, [&harness]() {
        // クランプ上書きが効いたことを、既定なら通る目標が却下されることで確認する
        harness.publishTargetPose(0.15, 0.12, 0.02);
        auto s = harness.latestStatus();
        return s && s->last_result == sharmech_msgs::msg::MotionStatus::RESULT_REJECTED;
      }, 2.0));
}

TEST(MotionGeneratorNode, WorkspaceClampResetRestoresDefaultBounds)
{
  TestHarness harness("clampreset");
  auto motion_node = std::make_shared<sharmech_core::MotionGeneratorNode>(fastTestOptions());

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(harness.node());
  executor.add_node(motion_node);
  ASSERT_TRUE(syncWithFeedback(harness, executor));

  // まず狭いクランプを適用し、0.15 が却下されることを確認する
  harness.publishWorkspaceClampOverride(-0.05, 0.05, 0.10, 0.15, 0.0, 0.05);
  ASSERT_TRUE(
    waitUntil(
      executor, [&harness]() {
        harness.publishTargetPose(0.15, 0.12, 0.02);
        auto s = harness.latestStatus();
        return s && s->last_result == sharmech_msgs::msg::MotionStatus::RESULT_REJECTED;
      }, 2.0));

  // reset でデフォルト (workspace_x_max = 0.20) に戻り、同じ目標が通るようになる
  harness.publishWorkspaceClampReset();
  ASSERT_TRUE(
    waitUntil(
      executor, [&harness]() {
        harness.publishTargetPose(0.15, 0.12, 0.02);
        auto s = harness.latestStatus();
        return s && s->mode == sharmech_msgs::msg::MotionStatus::MODE_GOAL;
      }, 2.0));
}


// ---- ジョグ速度上限 (/catchrobo/game/jog_limit) ----
// 従来 motion_generator_node は cmd_twist の大きさを一切クランプせず、操縦層が
// 送った値がそのまま目標速度になっていた (VRクライアントが自前の上限で肩代わり
// していた)。「誰が publish してもこの上限を超えられない」ことを保証する回帰。

// 速度[m/s]を、一定時間ジョグさせたときの移動距離から求める。
// a_max のレート制限があるので、上限に達してからの区間で測る
double measureJogSpeed(
  TestHarness & harness, rclcpp::executors::SingleThreadedExecutor & executor,
  double commanded_vx, double settle_sec, double measure_sec)
{
  const auto deadline = [](double sec) {
      return std::chrono::steady_clock::now() +
             std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(sec));
    };
  // 上限まで加速させる (cmd_twist はウォッチドッグに掛からないよう送り続ける)
  auto until = deadline(settle_sec);
  while (std::chrono::steady_clock::now() < until) {
    harness.publishTwist(commanded_vx, 0.0, 0.0);
    executor.spin_some();
    std::this_thread::sleep_for(2ms);
  }
  const double x0 = harness.latestCartesian()->pose.position.x;
  const auto t0 = std::chrono::steady_clock::now();
  until = deadline(measure_sec);
  while (std::chrono::steady_clock::now() < until) {
    harness.publishTwist(commanded_vx, 0.0, 0.0);
    executor.spin_some();
    std::this_thread::sleep_for(2ms);
  }
  const double x1 = harness.latestCartesian()->pose.position.x;
  const double elapsed =
    std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  return (x1 - x0) / elapsed;
}

TEST(MotionGeneratorNode, JogSpeedIsClampedToJogVMax)
{
  TestHarness harness("jog_clamp");
  // a_max を大きくして上限へすぐ到達させる (テスト時間の短縮)
  auto motion_node = std::make_shared<sharmech_core::MotionGeneratorNode>(
    fastTestOptions(
      {rclcpp::Parameter("jog_v_max", 0.10),
        rclcpp::Parameter("a_max", 5.0)}));
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(harness.node());
  executor.add_node(motion_node);
  ASSERT_TRUE(syncWithFeedback(harness, executor));

  // 上限の10倍を送っても、上限どおりの速度しか出ないこと
  const double speed = measureJogSpeed(harness, executor, 1.0, 0.3, 0.4);
  EXPECT_NEAR(speed, 0.10, 0.02);
}

TEST(MotionGeneratorNode, JogLimitOverrideSlowsDownAndResetRestores)
{
  TestHarness harness("jog_limit_override");
  auto motion_node = std::make_shared<sharmech_core::MotionGeneratorNode>(
    fastTestOptions(
      {rclcpp::Parameter("jog_v_max", 0.10),
        rclcpp::Parameter("a_max", 5.0)}));
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(harness.node());
  executor.add_node(motion_node);
  ASSERT_TRUE(syncWithFeedback(harness, executor));

  // 微調整中 (ADJUSTING_*) を模して上限を絞る
  harness.publishJogLimit(0.02);
  ASSERT_TRUE(waitUntil(executor, []() {return true;}, 0.1));
  EXPECT_NEAR(measureJogSpeed(harness, executor, 1.0, 0.3, 0.4), 0.02, 0.01);

  // 抜けたら起動時の上限へ戻る
  harness.publishJogLimitReset();
  ASSERT_TRUE(waitUntil(executor, []() {return true;}, 0.1));
  EXPECT_NEAR(measureJogSpeed(harness, executor, 1.0, 0.3, 0.4), 0.10, 0.02);
}

// 上書きで起動時の上限より「緩く」はできない (安全側の設定を壊させない)
TEST(MotionGeneratorNode, JogLimitOverrideCannotExceedStartupLimit)
{
  TestHarness harness("jog_limit_cap");
  auto motion_node = std::make_shared<sharmech_core::MotionGeneratorNode>(
    fastTestOptions(
      {rclcpp::Parameter("jog_v_max", 0.10),
        rclcpp::Parameter("a_max", 5.0)}));
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(harness.node());
  executor.add_node(motion_node);
  ASSERT_TRUE(syncWithFeedback(harness, executor));

  harness.publishJogLimit(10.0);        // 緩めようとする
  ASSERT_TRUE(waitUntil(executor, []() {return true;}, 0.1));
  EXPECT_NEAR(measureJogSpeed(harness, executor, 1.0, 0.3, 0.4), 0.10, 0.02);
}

// v_max <= 0 の上書きは無視する ——「ジョグが一切効かない」状態を無言で作らない
TEST(MotionGeneratorNode, JogLimitIgnoresNonPositiveOverride)
{
  TestHarness harness("jog_limit_zero");
  auto motion_node = std::make_shared<sharmech_core::MotionGeneratorNode>(
    fastTestOptions(
      {rclcpp::Parameter("jog_v_max", 0.10),
        rclcpp::Parameter("a_max", 5.0)}));
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(harness.node());
  executor.add_node(motion_node);
  ASSERT_TRUE(syncWithFeedback(harness, executor));

  harness.publishJogLimit(0.0);
  ASSERT_TRUE(waitUntil(executor, []() {return true;}, 0.1));
  EXPECT_NEAR(measureJogSpeed(harness, executor, 1.0, 0.3, 0.4), 0.10, 0.02);
}

// block: 自動シーケンス動作中はジョグを completely 無効化する。
// **ゴールが無い状態 (GRASPING / ORIENTING の待機中) でも止まること**が要点で、
// goal_priority による「ゴール実行中は無視」だけでは塞げない穴を埋める
TEST(MotionGeneratorNode, JogBlockStopsJogEvenWithoutActiveGoal)
{
  TestHarness harness("jog_block");
  auto motion_node = std::make_shared<sharmech_core::MotionGeneratorNode>(
    fastTestOptions(
      {rclcpp::Parameter("jog_v_max", 0.10),
        rclcpp::Parameter("a_max", 5.0)}));
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(harness.node());
  executor.add_node(motion_node);
  ASSERT_TRUE(syncWithFeedback(harness, executor));

  // まずジョグが効くことを確かめる (ゴールは一度も出していない = mode は IDLE/JOG)
  ASSERT_GT(measureJogSpeed(harness, executor, 1.0, 0.3, 0.3), 0.05);

  harness.publishJogBlock();
  ASSERT_TRUE(waitUntil(executor, []() {return true;}, 0.1));
  // 減速し切るまで待ってから測る (block 前の速度がレート制限で残るため)
  measureJogSpeed(harness, executor, 0.0, 0.3, 0.1);
  EXPECT_NEAR(measureJogSpeed(harness, executor, 1.0, 0.3, 0.3), 0.0, 0.005);

  // reset で解除される
  harness.publishJogLimitReset();
  ASSERT_TRUE(waitUntil(executor, []() {return true;}, 0.1));
  EXPECT_NEAR(measureJogSpeed(harness, executor, 1.0, 0.3, 0.4), 0.10, 0.02);
}

// ---- 起動時の同期と動作許可 (control_flags bit0) ----
// 同期前の目標姿勢は原点の仮値。これを MCU に追従させると ROS2 を起動しただけで
// アームが動くため、同期完了まで enable=false を送り、ゴール/ジョグも受け付けない

TEST(MotionGeneratorNode, HoldsWithEnableFalseUntilSyncedWithFeedback)
{
  TestHarness harness("startup_hold");
  auto motion_node = std::make_shared<sharmech_core::MotionGeneratorNode>(fastTestOptions());

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(harness.node());
  executor.add_node(motion_node);

  // フィードバック無し: ストリームは流れるが enable=false
  ASSERT_TRUE(
    waitUntil(
      executor, [&harness]() {
        return harness.latestEnable().has_value() && harness.latestCartesian().has_value();
      }, 1.0));
  EXPECT_FALSE(*harness.latestEnable());

  // 同期前のゴールは却下される
  harness.publishTargetPose(0.05, 0.05, 0.05);
  ASSERT_TRUE(
    waitUntil(
      executor, [&harness]() {
        auto st = harness.latestStatus();
        return st && st->last_result == sharmech_msgs::msg::MotionStatus::RESULT_REJECTED;
      }, 1.0));
  EXPECT_NE(harness.latestStatus()->message.find("synced"), std::string::npos);

  // 実姿勢が届いたら目標がそこへ同期し、enable=true に切り替わる (段差なし)
  ASSERT_TRUE(syncWithFeedback(harness, executor, 0.10, 0.05, 0.12));
  ASSERT_TRUE(
    waitUntil(
      executor, [&harness]() {
        auto c = harness.latestCartesian();
        return c && std::abs(c->pose.position.x - 0.10) < 1e-6;
      }, 1.0));
  const auto cmd = harness.latestCartesian();
  EXPECT_NEAR(cmd->pose.position.y, 0.05, 1e-6);
  EXPECT_NEAR(cmd->pose.position.z, 0.12, 1e-6);
  EXPECT_NEAR(cmd->twist.linear.x, 0.0, 1e-9);
}

TEST(MotionGeneratorNode, DoesNotSyncWhileMcuReportsUninitialized)
{
  TestHarness harness("uninit");
  auto motion_node = std::make_shared<sharmech_core::MotionGeneratorNode>(fastTestOptions());

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(harness.node());
  executor.add_node(motion_node);

  // 原点出し中 (bit3) の仮の姿勢には同期しない
  const auto deadline = std::chrono::steady_clock::now() + 300ms;
  while (std::chrono::steady_clock::now() < deadline) {
    harness.publishFeedback(0.10, 0.05, 0.12, sharmech_msgs::msg::McuStatus::FLAG_UNINITIALIZED);
    executor.spin_some();
    std::this_thread::sleep_for(5ms);
  }
  ASSERT_TRUE(harness.latestEnable().has_value());
  EXPECT_FALSE(*harness.latestEnable());

  // bit3 が落ちたら同期する
  ASSERT_TRUE(syncWithFeedback(harness, executor, 0.10, 0.05, 0.12));
  EXPECT_NEAR(harness.latestCartesian()->pose.position.x, 0.10, 1e-6);
}

// ---- 初期位置要求 (control_flags bit2) ----
// /catchrobo/game/reset の実体。座標は ROS2 側に無く、MCU が自前の初期角へ動く。
// ROS2 は bit2 を立てて実姿勢を追いかけ、McuStatus FLAG_AT_INIT_POSE で完了とみなす

TEST(MotionGeneratorNode, InitRequestRaisesBit2UntilMcuReportsArrival)
{
  TestHarness harness("init");
  auto motion_node = std::make_shared<sharmech_core::MotionGeneratorNode>(fastTestOptions());

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(harness.node());
  executor.add_node(motion_node);
  ASSERT_TRUE(syncWithFeedback(harness, executor, 0.10, 0.05, 0.12));

  harness.publishInitRequest();
  ASSERT_TRUE(
    waitUntil(
      executor, [&harness]() {
        auto st = harness.latestStatus();
        auto ir = harness.latestInitRequest();
        return st && st->mode == sharmech_msgs::msg::MotionStatus::MODE_INIT && ir && *ir;
      }, 1.0));
  EXPECT_TRUE(*harness.latestEnable());  // MCU が動く必要があるので動作許可は 1

  // 要求中のゴールは却下される
  harness.publishTargetPose(0.05, 0.05, 0.05);
  ASSERT_TRUE(
    waitUntil(
      executor, [&harness]() {
        auto st = harness.latestStatus();
        return st && st->last_result == sharmech_msgs::msg::MotionStatus::RESULT_REJECTED;
      }, 1.0));
  EXPECT_EQ(harness.latestStatus()->mode, sharmech_msgs::msg::MotionStatus::MODE_INIT);

  // MCU が初期位置 (0.30, 0.00, 0.15) へ動いた: 目標は実姿勢を追いかける
  ASSERT_TRUE(
    waitUntil(
      executor, [&harness]() {
        harness.publishFeedback(0.30, 0.00, 0.15);
        auto c = harness.latestCartesian();
        return c && std::abs(c->pose.position.x - 0.30) < 1e-6;
      }, 1.0));
  EXPECT_TRUE(*harness.latestInitRequest());  // 到達フラグが来るまで bit2 は立ったまま

  // 到達 (bit5) → IDLE + SUCCEEDED、bit2 は落ちる、目標は初期位置のまま
  ASSERT_TRUE(
    waitUntil(
      executor, [&harness]() {
        harness.publishFeedback(0.30, 0.00, 0.15, sharmech_msgs::msg::McuStatus::FLAG_AT_INIT_POSE);
        auto st = harness.latestStatus();
        auto ir = harness.latestInitRequest();
        return st && st->mode == sharmech_msgs::msg::MotionStatus::MODE_IDLE &&
        st->last_result == sharmech_msgs::msg::MotionStatus::RESULT_SUCCEEDED &&
        ir && !*ir;
      }, 1.0));
  EXPECT_NE(harness.latestStatus()->message.find("init"), std::string::npos);
  EXPECT_NEAR(harness.latestCartesian()->pose.position.x, 0.30, 1e-6);
  EXPECT_NEAR(harness.latestCartesian()->pose.position.z, 0.15, 1e-6);
  EXPECT_TRUE(*harness.latestEnable());
}

TEST(MotionGeneratorNode, InitRequestPreemptsGoalAndCancelAbortsInit)
{
  TestHarness harness("init_cancel");
  auto motion_node = std::make_shared<sharmech_core::MotionGeneratorNode>(fastTestOptions());

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(harness.node());
  executor.add_node(motion_node);
  ASSERT_TRUE(syncWithFeedback(harness, executor));

  harness.publishTargetPose(0.15, 0.15, 0.1);
  ASSERT_TRUE(
    waitUntil(
      executor, [&harness]() {
        auto st = harness.latestStatus();
        return st && st->mode == sharmech_msgs::msg::MotionStatus::MODE_GOAL;
      }, 1.0));

  // ゴール実行中でも初期位置要求が横取りする
  harness.publishInitRequest();
  ASSERT_TRUE(
    waitUntil(
      executor, [&harness]() {
        auto st = harness.latestStatus();
        return st && st->mode == sharmech_msgs::msg::MotionStatus::MODE_INIT;
      }, 1.0));

  // 到達を返さない MCU (未対応ファーム等) からの出口は cancel
  harness.publishCancel();
  ASSERT_TRUE(
    waitUntil(
      executor, [&harness]() {
        auto st = harness.latestStatus();
        auto ir = harness.latestInitRequest();
        return st && st->mode == sharmech_msgs::msg::MotionStatus::MODE_IDLE &&
        st->last_result == sharmech_msgs::msg::MotionStatus::RESULT_ABORTED &&
        ir && !*ir;
      }, 1.0));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  // 同名トピック (/catchrobo/command/cartesian 等) を流す別プロセス (実機 launch や
  // mock_mcu 疎通中のスタック) と DDS 上で混信すると速度サンプルが混ざって落ちるため、
  // テスト専用のドメインに隔離する (2026-09-05 に実際に混信で落ちた)。
  // 101 は既定ポート範囲で使える最大の ROS_DOMAIN_ID
  rclcpp::InitOptions init_options;
  init_options.set_domain_id(101);
  rclcpp::init(argc, argv, init_options);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
