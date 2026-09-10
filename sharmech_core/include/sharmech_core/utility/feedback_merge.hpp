#ifndef SHARMECH_CORE__UTILITY__FEEDBACK_MERGE_HPP_
#define SHARMECH_CORE__UTILITY__FEEDBACK_MERGE_HPP_

#include <cstddef>
#include <cstdint>

#include "sharmech_core/utility/udp_protocol.hpp"

namespace sharmech_core
{

// 2 基板構成の MCU フィードバック (0x81) の合成 (Static・ヘッダーオンリー)
//
// MCU は 2 枚あり (2026-09-10 ユーザー確定)、基板同士は通信できない:
//   - r/z 基板   (192.168.1.100): 肩・肘/膝の GIM ×4。r, z と肩・肘/膝の関節角を返す
//   - θ 基板     (192.168.1.101): ターンテーブル + エンドエフェクタ。θ、ターンテーブル
//                の関節角、pitch/yaw、グリッパ状態を返す
// ROS2 は同じ 0x01 を両方へ送り、各基板は **自分の担当フィールドだけ埋め、他は 0** で
// 同じ 8889 番へ 0x81 を返す (契約の正本: sharmech/docs/mcu_spec.md §3.4)。
// 合成は hardware_bridge_node がここで行い、下流には 1 本のフィードバックとして見せる。
//
// status_flags の合成規則:
//   - bit0〜4 (追従誤差・異常・ホールド・未初期化・破棄) は **OR**。片方でも立てば立つ。
//     特に bit3 (未初期化) を OR にしないと、片方の基板だけ原点出しが済んだ時点で
//     motion_generator_node が目標を同期して enable=1 を送り、もう片方が原点出しの
//     途中で動き出す
//   - bit5 (初期位置到達) は **AND**。両方が初期位置に着くまで INIT モードを抜けない
class FeedbackMerge
{
public:
  FeedbackMerge() = delete;

  // kJointOrder 上のターンテーブル関節の位置 (θ 基板が埋めるスロット)
  static constexpr std::size_t kTurntableJointIndex = 2;
  static_assert(
    udp_protocol::kJointOrder[kTurntableJointIndex][0] == 't' &&
    udp_protocol::kJointOrder[kTurntableJointIndex][1] == 'u',
    "kTurntableJointIndex must point at 'turntable' in kJointOrder");

  // AND で合成するビット。それ以外は OR
  static constexpr uint16_t kAndMergedBits = udp_protocol::kStatusAtInitPose;

  static uint16_t mergeStatusFlags(uint16_t arm_flags, uint16_t theta_flags)
  {
    return static_cast<uint16_t>(
      ((arm_flags | theta_flags) & static_cast<uint16_t>(~kAndMergedBits)) |
      (arm_flags & theta_flags & kAndMergedBits));
  }

  // r/z 基板と θ 基板のフィードバックを 1 つに合成する。
  // seq / seq_echo / timestamp_us は r/z 基板のものを採用する (基板ごとの連番管理は
  // 呼び出し側の責務。ここでは値を選ぶだけ)
  static udp_protocol::Feedback merge(
    const udp_protocol::Feedback & arm, const udp_protocol::Feedback & theta)
  {
    udp_protocol::Feedback out = arm;  // r, z, 肩・肘/膝の関節角, seq 系
    out.theta = theta.theta;
    out.pitch = theta.pitch;
    out.yaw = theta.yaw;
    out.gripper_closed = theta.gripper_closed;
    if (out.joint_positions.size() > kTurntableJointIndex &&
      theta.joint_positions.size() > kTurntableJointIndex)
    {
      out.joint_positions[kTurntableJointIndex] =
        theta.joint_positions[kTurntableJointIndex];
    }
    out.status_flags = mergeStatusFlags(arm.status_flags, theta.status_flags);
    return out;
  }
};

}  // namespace sharmech_core

#endif  // SHARMECH_CORE__UTILITY__FEEDBACK_MERGE_HPP_
