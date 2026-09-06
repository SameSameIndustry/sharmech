#ifndef SHARMECH_CORE__UTILITY__TRAPEZOIDAL_TRAJECTORY_HPP_
#define SHARMECH_CORE__UTILITY__TRAPEZOIDAL_TRAJECTORY_HPP_

#include <algorithm>
#include <cmath>

namespace sharmech_core
{

// エンドエフェクタの状態 (x, y, z, pitch, yaw)。
// 速度として使う場合は各成分が [m/s], [rad/s] になる。
// このロボットの手首は pitch/yaw の2自由度のみで roll は存在しない
struct CartesianState
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double pitch{0.0};
  double yaw{0.0};
};

// 始点→終点を直線1本で結ぶ台形速度プロファイル軌道。
//
// 経過時間 t で評価する (配列のウェイポイントは持たない)。
// 「インデックスを1つずつ進める」実装は距離に関係なく一定時間で走り切って
// しまうため禁止 (sharmech_core/docs/motion_generator_node.md 参照)。
//
// 位置 (3D) と姿勢 (pitch/yaw) は同じ正規化進行度 s(t) ∈ [0,1] を共有する。
// 所要時間は「位置の台形プロファイル」と「姿勢の台形プロファイル」の長い方。
// 短い方の軸は制限速度より遅く動くだけなので常に安全側になる
class TrapezoidalTrajectory
{
public:
  TrapezoidalTrajectory() = default;

  TrapezoidalTrajectory(
    const CartesianState & start, const CartesianState & goal,
    double v_max, double a_max, double w_max, double alpha_max)
  : start_(start)
  {
    delta_.x = goal.x - start.x;
    delta_.y = goal.y - start.y;
    delta_.z = goal.z - start.z;
    delta_.pitch = goal.pitch - start.pitch;
    delta_.yaw = goal.yaw - start.yaw;

    const double d_pos = std::sqrt(
      delta_.x * delta_.x + delta_.y * delta_.y + delta_.z * delta_.z);
    const double d_ang = std::max(std::abs(delta_.pitch), std::abs(delta_.yaw));

    // 位置・姿勢それぞれの所要時間を見て、支配的な方でプロファイルを組む
    Profile pos_profile = makeProfile(d_pos, v_max, a_max);
    Profile ang_profile = makeProfile(d_ang, w_max, alpha_max);
    profile_ = (pos_profile.total >= ang_profile.total) ? pos_profile : ang_profile;
  }

  double duration() const {return profile_.total;}

  // 経過時間 t での目標状態と目標速度。t は [0, duration] にクランプされる
  void sample(double t, CartesianState & pose_out, CartesianState & vel_out) const
  {
    double s = 1.0;       // 正規化進行度
    double s_dot = 0.0;   // その時間微分
    if (profile_.distance > 1e-9 && profile_.total > 0.0) {
      const double q = profile_.position(std::clamp(t, 0.0, profile_.total));
      const double q_dot = profile_.velocity(std::clamp(t, 0.0, profile_.total));
      s = q / profile_.distance;
      s_dot = q_dot / profile_.distance;
    }

    pose_out.x = start_.x + s * delta_.x;
    pose_out.y = start_.y + s * delta_.y;
    pose_out.z = start_.z + s * delta_.z;
    pose_out.pitch = start_.pitch + s * delta_.pitch;
    pose_out.yaw = start_.yaw + s * delta_.yaw;

    vel_out.x = s_dot * delta_.x;
    vel_out.y = s_dot * delta_.y;
    vel_out.z = s_dot * delta_.z;
    vel_out.pitch = s_dot * delta_.pitch;
    vel_out.yaw = s_dot * delta_.yaw;
  }

private:
  // スカラー距離に対する台形 (または三角) プロファイル
  struct Profile
  {
    double distance{0.0};
    double accel{0.0};
    double t_acc{0.0};
    double t_flat{0.0};
    double total{0.0};

    double position(double t) const
    {
      if (t <= t_acc) {
        return 0.5 * accel * t * t;
      }
      const double d_acc = 0.5 * accel * t_acc * t_acc;
      const double v_peak = accel * t_acc;
      if (t <= t_acc + t_flat) {
        return d_acc + v_peak * (t - t_acc);
      }
      const double t_remain = total - t;
      return distance - 0.5 * accel * t_remain * t_remain;
    }

    double velocity(double t) const
    {
      if (t <= t_acc) {return accel * t;}
      if (t <= t_acc + t_flat) {return accel * t_acc;}
      return accel * (total - t);
    }
  };

  static Profile makeProfile(double distance, double v_max, double a_max)
  {
    Profile p;
    p.distance = distance;
    if (distance < 1e-9 || v_max <= 0.0 || a_max <= 0.0) {
      return p;  // 移動なし。total = 0
    }
    p.accel = a_max;
    const double t_acc_full = v_max / a_max;
    const double d_acc_full = 0.5 * a_max * t_acc_full * t_acc_full;
    if (2.0 * d_acc_full >= distance) {
      // v_max に届かない三角プロファイル
      p.t_acc = std::sqrt(distance / a_max);
      p.t_flat = 0.0;
    } else {
      p.t_acc = t_acc_full;
      p.t_flat = (distance - 2.0 * d_acc_full) / v_max;
    }
    p.total = 2.0 * p.t_acc + p.t_flat;
    return p;
  }

  CartesianState start_{};
  CartesianState delta_{};
  Profile profile_{};
};

}  // namespace sharmech_core

#endif  // SHARMECH_CORE__UTILITY__TRAPEZOIDAL_TRAJECTORY_HPP_
