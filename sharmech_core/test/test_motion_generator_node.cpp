// motion_generator_node のウォッチドッグ・作業領域クランプ・入力調停ロジックの検証。
//
// sharmech_core/docs/motion_generator_node.md に列挙されている「罠」
// (ゼロ Twist はゴールを abort しない、クランプ時は速度も0にする、等) は
// コールバックとタイマーに分散した状態遷移なので、ノードとして起動し
// トピック経由で入出力させないと検証できない。

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
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
#include <sharmech_msgs/msg/cartesian_command.hpp>
#include <sharmech_msgs/msg/motion_status.hpp>
#include <sharmech_msgs/msg/workspace_clamp.hpp>

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

  void publishWorkspaceClampReset()
  {
    sharmech_msgs::msg::WorkspaceClamp msg;
    msg.reset = true;
    workspace_clamp_pub_->publish(msg);
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

TEST(MotionGeneratorNode, NonZeroTwistAbortsActiveGoal)
{
  TestHarness harness("aborttwist");
  auto motion_node = std::make_shared<sharmech_core::MotionGeneratorNode>(fastTestOptions());

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(harness.node());
  executor.add_node(motion_node);

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

TEST(MotionGeneratorNode, WatchdogDeceleratesJogToIdleWhenTwistStalls)
{
  TestHarness harness("watchdog");
  auto motion_node = std::make_shared<sharmech_core::MotionGeneratorNode>(fastTestOptions());

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(harness.node());
  executor.add_node(motion_node);

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
