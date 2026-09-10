#ifndef SHARMECH_CORE__HARDWARE_BRIDGE_NODE_HPP_
#define SHARMECH_CORE__HARDWARE_BRIDGE_NODE_HPP_

#include <optional>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sharmech_msgs/msg/cartesian_command.hpp>
#include <sharmech_msgs/msg/mcu_status.hpp>

#include "sharmech_core/utility/udp_protocol.hpp"

#include <netinet/in.h>

namespace sharmech_core
{

// MCU 通信ノード (UDP)
//
// ROS2 トピックと UDP パケットの間の変換と輸送のみを担う。判断はしない。
// 補間は MCU、レート制限・クランプ・ウォッチドッグは motion_generator_node の責務。
//
// 仕様の正本: sharmech_core/docs/hardware_bridge_node.md
//
// Sub: /catchrobo/command/cartesian (command_mode == cartesian)
// Sub: /catchrobo/command/joint     (command_mode == joint。パターンB)
// Sub: /catchrobo/command/gripper
// Sub: /catchrobo/command/orient_vertical  (game_state_manager_node の PLACING 指示)
// Pub: /catchrobo/arm/current_pose  (MCU が FK して返した実姿勢)
// Pub: /joint_states                (実測の関節角)
// Pub: /catchrobo/arm/mcu_status    (MCU の status_flags と疎通状態)
//
// MCU は 2 枚構成 (r/z 基板 = mcu_ip、θ 基板 = mcu_theta_ip)。同じ指令パケットを
// 両方へ送り、両方から届く 0x81 を送信元で見分けて 1 本に合成する
// (utility/feedback_merge.hpp)。mcu_theta_ip が空なら従来どおりの 1 枚構成
class HardwareBridgeNode : public rclcpp::Node
{
public:
  explicit HardwareBridgeNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~HardwareBridgeNode() override;

private:
  // 基板 1 枚ぶんの宛先と受信状態
  struct McuBoard
  {
    std::string name;      // ログ用 ("r/z" / "theta")
    std::string ip;
    int port{0};
    sockaddr_in addr{};
    std::optional<udp_protocol::Feedback> latest;   // 直近に受理したフィードバック
    std::optional<rclcpp::Time> last_time;          // 途絶の検出用
    std::optional<uint32_t> last_seq;               // 順序逆転の検出用 (基板ごと)
    uint32_t out_of_order{0};
  };
  // 送信: サブスクリプション駆動 (タイマーではない)。
  // 上流が止まれば送信も止まり、MCU 側ウォッチドッグが作動して安全側に倒れる
  void onCartesianCommand(const sharmech_msgs::msg::CartesianCommand::SharedPtr msg);
  void onJointCommand(const sensor_msgs::msg::JointState::SharedPtr msg);
  void onGripperCommand(const std_msgs::msg::Bool::SharedPtr msg);
  void onOrientVerticalCommand(const std_msgs::msg::Bool::SharedPtr msg);
  // joint モード (パターンB) では位置は /catchrobo/command/joint から来るが、
  // 動作許可・初期位置要求は Cartesian ストリームにしか無いのでそこからラッチする
  void onCartesianFlagsOnly(const sharmech_msgs::msg::CartesianCommand::SharedPtr msg);
  // ラッチ済みの各フラグから control_flags を組む
  udp_protocol::ControlFlags controlFlags() const;

  // 受信: タイマーでソケットに溜まったデータグラムを読み切る
  void onFeedbackTimer();

  bool openUdpSocket();
  // 全基板へ同じパケットを送る
  void sendToAllBoards(const std::vector<uint8_t> & packet);
  // 送信元アドレスから基板を特定する (IP で一致、複数一致ならポートで区別)。
  // 未知の送信元なら nullptr
  McuBoard * findBoard(const sockaddr_in & from);
  // 合成済みフィードバックを /catchrobo/arm/current_pose と /joint_states へ流す
  void publishFeedback(const udp_protocol::Feedback & fb, const rclcpp::Time & stamp);
  // ターンテーブル軸位置の実行中変更 (ros2 param set)。有限値のみ受理する
  rcl_interfaces::msg::SetParametersResult onSetParameters(
    const std::vector<rclcpp::Parameter> & params);
  // フィードバックの有無に関わらず定期的に疎通状態を配信する
  void publishMcuStatus(bool connected, double silence_sec);

  rclcpp::Subscription<sharmech_msgs::msg::CartesianCommand>::SharedPtr cartesian_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr gripper_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr orient_vertical_sub_;
  rclcpp::Subscription<sharmech_msgs::msg::CartesianCommand>::SharedPtr cartesian_flags_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub_;
  rclcpp::Publisher<sharmech_msgs::msg::McuStatus>::SharedPtr mcu_status_pub_;
  rclcpp::TimerBase::SharedPtr feedback_timer_;

  // パラメータ
  std::string command_mode_;
  std::string mcu_ip_;          // r/z 基板 (必須)
  int mcu_port_;
  std::string mcu_theta_ip_;    // θ 基板。空なら 1 枚構成
  int mcu_theta_port_;          // 負なら mcu_port と同じ
  int local_port_;
  double feedback_poll_rate_;        // [Hz]
  double feedback_timeout_;          // [s]
  std::vector<std::string> joint_names_;
  // ターンテーブル回転軸のベース座標系での位置 [m] = UDP 極座標 (r, θ) の原点。
  // 正本は robot_geometry.yaml の kinematics.turntable_axis_x/y_m (人間が実測して入れる)。
  // 生成物 robot_geometry.generated.yaml 経由で届き、実行中に ros2 param set でも変えられる。
  // 送信 (x,y → r,θ) と受信 (r,θ → x,y) の両方で同じ値を使う
  double turntable_axis_x_{0.0};
  double turntable_axis_y_{0.0};
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  // 内部状態
  int sockfd_{-1};
  // [0] = r/z 基板 (必須)、[1] = θ 基板 (mcu_theta_ip が空なら無い)
  std::vector<McuBoard> boards_;
  bool gripper_state_{false};              // ラッチしたグリッパ状態
  bool orient_vertical_state_{false};         // ラッチした「縦にする」指示
  // 動作許可 (control_flags bit0) と初期位置要求 (bit2)。Cartesian ストリームの
  // 同名フィールドを毎パケット写す。**既定 false** —— 上流から何も届いていない間に
  // 送る理由は無く、届いた瞬間から値は常にストリーム側が決める
  bool enable_state_{false};
  bool init_request_state_{false};
  uint32_t send_seq_{0};
  // 直前に送った θ [rad]。次の θ をこの値の近傍へアンラップして連続化する
  // (±π のまたぎでターンテーブルを逆走させないため)。
  // **ROS2 が駆動していない間 (未送信・動作許可 0・初期位置要求中) は MCU の実 θ で
  // 上書きする。** MCU が自力で θ=+3.0 に居るのにこちらの基準が 0 のままだと、
  // 同期後の最初の指令が -3.28 側の分岐に落ちてターンテーブルが1回転してしまう
  double last_sent_theta_{0.0};
  bool has_sent_command_{false};
  // 直近に合成フィードバックを publish した時刻 (全基板が揃って新鮮だった時刻)
  std::optional<rclcpp::Time> last_feedback_time_;
  bool warned_joint_names_{false};
  bool warned_joint_cmd_names_{false};          // 0x02送信側の名前不一致の警告 (1回のみ)
  bool warned_joint_cmd_velocity_{false};         // 同、velocity欠落の警告 (1回のみ)
  uint16_t last_status_flags_{0};
  bool last_gripper_state_{false};
  uint32_t last_seq_{0};        // McuStatus.seq (r/z 基板の連番)
  uint32_t last_seq_echo_{0};
};

}  // namespace sharmech_core

#endif  // SHARMECH_CORE__HARDWARE_BRIDGE_NODE_HPP_
