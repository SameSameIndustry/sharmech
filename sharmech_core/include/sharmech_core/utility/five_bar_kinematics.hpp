#ifndef SHARMECH_CORE__UTILITY__FIVE_BAR_KINEMATICS_HPP_
#define SHARMECH_CORE__UTILITY__FIVE_BAR_KINEMATICS_HPP_

#include <cmath>
#include <optional>
#include <algorithm>

namespace sharmech_core
{

struct FiveBarParams
{
  double l1;          // 第1リンク長 [m]  (モータ側)
  double l2;          // 第2リンク長 [m]  (エンドエフェクタ側)
  double base_width;  // 2モータ間距離 [m]
};

struct JointAngles
{
  double theta1;  // モータ1 関節角 [rad]
  double theta2;  // モータ2 関節角 [rad]
};

struct CartesianPoint
{
  double x;
  double y;
};

// 5節リンク機構の逆運動学・順運動学
//
//   motor1(-d, 0) ---l1--- elbow_L ---l2---+
//                                          | EE (x, y)
//   motor2(+d, 0) ---l1--- elbow_R ---l2---+
//
//   d = base_width / 2
class FiveBarKinematics
{
public:
  FiveBarKinematics() = delete;

  // 逆運動学: (x, y) → (θ1, θ2)
  // 解が存在しない場合は nullopt を返す
  static std::optional<JointAngles> inverseKinematics(
    const CartesianPoint & target, const FiveBarParams & p)
  {
    const double d = p.base_width / 2.0;

    // モータ1 (-d, 0) からの2リンクIK
    const double dx1 = target.x + d;
    const double dy1 = target.y;
    const double r1  = std::hypot(dx1, dy1);

    if (!isReachableArm(r1, p.l1, p.l2)) {return std::nullopt;}

    const double cos_e1 = (p.l1 * p.l1 + r1 * r1 - p.l2 * p.l2) / (2.0 * p.l1 * r1);
    const double e1     = std::acos(std::clamp(cos_e1, -1.0, 1.0));
    const double theta1 = std::atan2(dy1, dx1) - e1;  // elbow-up

    // モータ2 (+d, 0) からの2リンクIK
    const double dx2 = target.x - d;
    const double dy2 = target.y;
    const double r2  = std::hypot(dx2, dy2);

    if (!isReachableArm(r2, p.l1, p.l2)) {return std::nullopt;}

    const double cos_e2 = (p.l1 * p.l1 + r2 * r2 - p.l2 * p.l2) / (2.0 * p.l1 * r2);
    const double e2     = std::acos(std::clamp(cos_e2, -1.0, 1.0));
    const double theta2 = std::atan2(dy2, dx2) + e2;  // elbow-up (右腕は符号逆)

    return JointAngles{theta1, theta2};
  }

  // 順運動学: (θ1, θ2) → (x, y)
  // 5節リンクの閉ループ解は近似（2アームの肘先位置の中点）
  // 厳密解が必要な場合は2円の交点計算に置き換える
  static CartesianPoint forwardKinematics(
    const JointAngles & joints, const FiveBarParams & p)
  {
    const double d = p.base_width / 2.0;

    const double ex1 = -d + p.l1 * std::cos(joints.theta1);
    const double ey1 =      p.l1 * std::sin(joints.theta1);
    const double ex2 =  d + p.l1 * std::cos(joints.theta2);
    const double ey2 =      p.l1 * std::sin(joints.theta2);

    // 2つの前腕の先端が交わる点 (厳密には2円の交点)
    // TODO: 厳密な交点計算に置き換えること
    return CartesianPoint{(ex1 + ex2) / 2.0 + (p.l2 * std::cos(joints.theta1) + p.l2 * std::cos(joints.theta2)) / 2.0,
                          (ey1 + ey2) / 2.0 + (p.l2 * std::sin(joints.theta1) + p.l2 * std::sin(joints.theta2)) / 2.0};
  }

  // 目標点が作業領域内か確認
  static bool isReachable(const CartesianPoint & target, const FiveBarParams & p)
  {
    const double d = p.base_width / 2.0;
    const double r1 = std::hypot(target.x + d, target.y);
    const double r2 = std::hypot(target.x - d, target.y);
    return isReachableArm(r1, p.l1, p.l2) && isReachableArm(r2, p.l1, p.l2);
  }

private:
  static bool isReachableArm(double r, double l1, double l2)
  {
    return r <= l1 + l2 && r >= std::abs(l1 - l2) && r > 1e-9;
  }
};

}  // namespace sharmech_core

#endif  // SHARMECH_CORE__UTILITY__FIVE_BAR_KINEMATICS_HPP_
