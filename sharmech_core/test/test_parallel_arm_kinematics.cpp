#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "sharmech_core/utility/parallel_arm_kinematics.hpp"

using sharmech_core::EndEffectorPosition;
using sharmech_core::ParallelArmGeometry;
using sharmech_core::ParallelArmJointAngles;
using sharmech_core::ParallelArmJointRates;
using sharmech_core::ParallelArmKinematics;
using sharmech_core::SymmetricLinkage;
using sharmech_core::SymmetricLinkageParams;

namespace
{
constexpr double kTol = 1e-9;
}  // namespace

// a=0, L1=3, L2=5 (3-4-5直角三角形になるよう選んだ手計算しやすい値)。
// theta=0 では knee=(-3,0)、EEはそこから距離5の点 (0,d) 上 → d=4
TEST(SymmetricLinkage, ZeroAngleMatchesHandComputedValue)
{
  SymmetricLinkageParams p;
  p.pivot_half_separation_m = 0.0;
  p.proximal_link_length_m = 3.0;
  p.distal_link_length_m = 5.0;

  const auto d = SymmetricLinkage::computeExtension(0.0, p);
  ASSERT_TRUE(d.has_value());
  EXPECT_NEAR(*d, 4.0, kTol);
}

// 同じリンクで theta=pi/2 (knee=(0,3)) → d = 3 + 5 = 8
TEST(SymmetricLinkage, RightAngleMatchesHandComputedValue)
{
  SymmetricLinkageParams p;
  p.pivot_half_separation_m = 0.0;
  p.proximal_link_length_m = 3.0;
  p.distal_link_length_m = 5.0;

  const auto d = SymmetricLinkage::computeExtension(M_PI_2, p);
  ASSERT_TRUE(d.has_value());
  EXPECT_NEAR(*d, 8.0, kTol);
}

// pivot_half_separation_m を 0 でない値にしても、
// 「肘関節からEEまでの距離が distal_link_length_m に一致する」という
// 幾何拘束そのものが満たされていることを確認する (回帰用の不変条件テスト)
TEST(SymmetricLinkage, ExtensionSatisfiesDistalLinkLengthConstraint)
{
  const std::vector<SymmetricLinkageParams> cases = {
    {0.05, 0.10, 0.15},
    {0.03, 0.08, 0.12},
    {0.10, 0.05, 0.20},
  };
  const std::vector<double> angles = {0.0, 0.3, 0.7, 1.2};

  for (const auto & p : cases) {
    for (const double theta : angles) {
      const auto d = SymmetricLinkage::computeExtension(theta, p);
      ASSERT_TRUE(d.has_value()) << "a=" << p.pivot_half_separation_m << " theta=" << theta;

      const double knee_x = p.pivot_half_separation_m - p.proximal_link_length_m * std::cos(theta);
      const double knee_y = p.proximal_link_length_m * std::sin(theta);
      const double dist_to_ee = std::hypot(knee_x - 0.0, knee_y - *d);
      EXPECT_NEAR(dist_to_ee, p.distal_link_length_m, kTol);
    }
  }
}

// 肘関節がEEに届かない (a - L1cosθ が L2 を超える) 角度では nullopt を返す
TEST(SymmetricLinkage, UnreachableAngleReturnsNullopt)
{
  SymmetricLinkageParams p;
  p.pivot_half_separation_m = 10.0;
  p.proximal_link_length_m = 0.01;
  p.distal_link_length_m = 0.01;

  const auto d = SymmetricLinkage::computeExtension(0.0, p);
  EXPECT_FALSE(d.has_value());
}

// 肩機構(r)・ターンテーブル(θ)・肘/膝機構(z)を組み合わせたエンドエフェクタ座標。
// r=4 (上のZeroAngle相当), z_extension=8 (上のRightAngle相当) を再利用し、
// ターンテーブル角とオフセットが正しく合成されることを確認する
TEST(ParallelArmKinematics, CombinesShoulderTurntableAndKnee)
{
  ParallelArmGeometry geom;
  geom.shoulder = {0.0, 3.0, 5.0};  // theta=0 → r=4
  geom.knee = {0.0, 3.0, 5.0};      // theta=pi/2 → z_extension=8
  geom.turntable_axis_x_m = 0.02;
  geom.turntable_axis_y_m = -0.01;
  geom.knee_base_height_m = 0.1;

  ParallelArmJointAngles joints;
  joints.shoulder_motor_angle_rad = 0.0;
  joints.turntable_angle_rad = M_PI_2;  // rをy軸方向へ向ける
  joints.knee_motor_angle_rad = M_PI_2;

  const auto pos = ParallelArmKinematics::forwardKinematics(joints, geom);
  ASSERT_TRUE(pos.has_value());
  EXPECT_NEAR(pos->x, 0.02, kTol);        // 0.02 + 4*cos(90°)
  EXPECT_NEAR(pos->y, -0.01 + 4.0, kTol);  // -0.01 + 4*sin(90°)
  EXPECT_NEAR(pos->z, 0.1 + 8.0, kTol);
}

// 肩・肘のどちらか一方でも機構が届かない角度なら、全体としてnulloptになる
TEST(ParallelArmKinematics, UnreachableJointMakesWholeResultNullopt)
{
  ParallelArmGeometry geom;
  geom.shoulder = {10.0, 0.01, 0.01};  // 常に届かない
  geom.knee = {0.0, 3.0, 5.0};

  ParallelArmJointAngles joints;
  joints.shoulder_motor_angle_rad = 0.0;
  joints.turntable_angle_rad = 0.0;
  joints.knee_motor_angle_rad = 0.0;

  const auto pos = ParallelArmKinematics::forwardKinematics(joints, geom);
  EXPECT_FALSE(pos.has_value());
}

// computeExtension で使った a=0, L1=3, L2=5 の手計算値 (theta=0→d=4, theta=pi/2→d=8) を
// computeMotorAngle に逆投入し、元の角度に戻ることを確認する
TEST(SymmetricLinkage, ComputeMotorAngleMatchesHandComputedValues)
{
  SymmetricLinkageParams p;
  p.pivot_half_separation_m = 0.0;
  p.proximal_link_length_m = 3.0;
  p.distal_link_length_m = 5.0;

  const auto theta0 = SymmetricLinkage::computeMotorAngle(4.0, p);
  ASSERT_TRUE(theta0.has_value());
  EXPECT_NEAR(*theta0, 0.0, kTol);

  const auto theta1 = SymmetricLinkage::computeMotorAngle(8.0, p);
  ASSERT_TRUE(theta1.has_value());
  EXPECT_NEAR(*theta1, M_PI_2, kTol);
}

// computeExtension → computeMotorAngle が任意の(a, L1, L2, theta)で往復一致することを確認。
//
// 「+sqrt」側の同じ曲線上でも、dが同じになるthetaが2つ存在しうる
// (肘の配置が2通りあるのではなく、曲線自体がthetaについて単調でないため)。
// そのため reference_angle に元のthetaを渡し、連続性優先でその解が選ばれることを確認する
TEST(SymmetricLinkage, ExtensionAndMotorAngleRoundTripWithReference)
{
  const std::vector<SymmetricLinkageParams> cases = {
    {0.0, 3.0, 5.0},
    {0.05, 0.10, 0.15},
    {0.03, 0.08, 0.12},
    {0.10, 0.05, 0.20},
    {1.0, 2.0, 3.0},
  };
  const std::vector<double> angles = {0.0, 0.3, 0.7, 1.2, 1.5};

  for (const auto & p : cases) {
    for (const double theta : angles) {
      const auto d = SymmetricLinkage::computeExtension(theta, p);
      ASSERT_TRUE(d.has_value()) << "a=" << p.pivot_half_separation_m << " theta=" << theta;

      const auto theta_back = SymmetricLinkage::computeMotorAngle(*d, p, theta);
      ASSERT_TRUE(theta_back.has_value())
        << "a=" << p.pivot_half_separation_m << " theta=" << theta << " d=" << *d;
      EXPECT_NEAR(*theta_back, theta, 1e-6)
        << "a=" << p.pivot_half_separation_m << " theta=" << theta;
    }
  }
}

// reference_angle を渡さない場合でも、返ってきた角度をcomputeExtensionへ
// 通せば同じdを再現する (=数学的に有効な解である) ことを確認する。
// (元のthetaと一致するとは限らない。上のテストのコメント参照)
TEST(SymmetricLinkage, ComputeMotorAngleWithoutReferenceIsSelfConsistent)
{
  const std::vector<SymmetricLinkageParams> cases = {
    {0.0, 3.0, 5.0},
    {0.05, 0.10, 0.15},
    {0.03, 0.08, 0.12},
    {0.10, 0.05, 0.20},
    {1.0, 2.0, 3.0},
  };
  const std::vector<double> angles = {0.0, 0.3, 0.7, 1.2, 1.5};

  for (const auto & p : cases) {
    for (const double theta : angles) {
      const auto d = SymmetricLinkage::computeExtension(theta, p);
      ASSERT_TRUE(d.has_value());

      const auto theta_back = SymmetricLinkage::computeMotorAngle(*d, p);
      ASSERT_TRUE(theta_back.has_value());

      const auto d_back = SymmetricLinkage::computeExtension(*theta_back, p);
      ASSERT_TRUE(d_back.has_value());
      EXPECT_NEAR(*d_back, *d, 1e-6);
    }
  }
}

// 三角不等式を満たさない距離 (届かない/近すぎる) では nullopt を返す
TEST(SymmetricLinkage, ComputeMotorAngleUnreachableReturnsNullopt)
{
  SymmetricLinkageParams p;
  p.pivot_half_separation_m = 0.0;
  p.proximal_link_length_m = 3.0;
  p.distal_link_length_m = 5.0;

  EXPECT_FALSE(SymmetricLinkage::computeMotorAngle(100.0, p).has_value());  // 届かない
}

// computeExtensionDerivative が数値微分と一致することを確認
TEST(SymmetricLinkage, ExtensionDerivativeMatchesNumericalDifferentiation)
{
  SymmetricLinkageParams p{0.05, 0.10, 0.15};
  const double theta = 0.4;
  const double h = 1e-6;

  const auto d_plus = SymmetricLinkage::computeExtension(theta + h, p);
  const auto d_minus = SymmetricLinkage::computeExtension(theta - h, p);
  ASSERT_TRUE(d_plus.has_value());
  ASSERT_TRUE(d_minus.has_value());
  const double numerical = (*d_plus - *d_minus) / (2.0 * h);

  const auto analytical = SymmetricLinkage::computeExtensionDerivative(theta, p);
  ASSERT_TRUE(analytical.has_value());
  EXPECT_NEAR(*analytical, numerical, 1e-5);
}

// forwardKinematics → inverseKinematics の往復一致 (turntable_axisオフセットも含む)
TEST(ParallelArmKinematics, ForwardAndInverseKinematicsRoundTrip)
{
  ParallelArmGeometry geom;
  geom.shoulder = {0.05, 0.10, 0.15};
  geom.knee = {0.03, 0.08, 0.12};
  geom.turntable_axis_x_m = 0.02;
  geom.turntable_axis_y_m = -0.01;
  geom.knee_base_height_m = 0.1;

  ParallelArmJointAngles joints;
  joints.shoulder_motor_angle_rad = 0.5;
  joints.turntable_angle_rad = 1.1;
  joints.knee_motor_angle_rad = 0.8;

  const auto pos = ParallelArmKinematics::forwardKinematics(joints, geom);
  ASSERT_TRUE(pos.has_value());

  // 連続性優先の解選択を働かせるため、元の関節角をreferenceとして渡す
  // (SymmetricLinkage::ComputeMotorAngleの2解ある場合があることの回帰テスト参照)
  const auto joints_back = ParallelArmKinematics::inverseKinematics(*pos, geom, joints);
  ASSERT_TRUE(joints_back.has_value());
  EXPECT_NEAR(joints_back->shoulder_motor_angle_rad, joints.shoulder_motor_angle_rad, 1e-6);
  EXPECT_NEAR(joints_back->turntable_angle_rad, joints.turntable_angle_rad, 1e-6);
  EXPECT_NEAR(joints_back->knee_motor_angle_rad, joints.knee_motor_angle_rad, 1e-6);
}

// ターンテーブル軸の直上 (r=0) は特異点なので nullopt
TEST(ParallelArmKinematics, InverseKinematicsSingularAtTurntableAxisReturnsNullopt)
{
  ParallelArmGeometry geom;
  geom.shoulder = {0.05, 0.10, 0.15};
  geom.knee = {0.03, 0.08, 0.12};

  EndEffectorPosition target{0.0, 0.0, 0.05};
  EXPECT_FALSE(ParallelArmKinematics::inverseKinematics(target, geom).has_value());
}

// inverseVelocityが返す関節角速度で微小時間だけ関節角を進めforwardKinematicsすると、
// 指定したエンドエフェクタ速度 (vx,vy,vz) 通りに動くことを確認する
TEST(ParallelArmKinematics, InverseVelocityMatchesForwardFiniteDifference)
{
  ParallelArmGeometry geom;
  geom.shoulder = {0.05, 0.10, 0.15};
  geom.knee = {0.03, 0.08, 0.12};
  geom.turntable_axis_x_m = 0.02;
  geom.turntable_axis_y_m = -0.01;
  geom.knee_base_height_m = 0.1;

  ParallelArmJointAngles joints;
  joints.shoulder_motor_angle_rad = 0.5;
  joints.turntable_angle_rad = 1.1;
  joints.knee_motor_angle_rad = 0.8;

  const double vx = 0.02;
  const double vy = -0.01;
  const double vz = 0.03;

  const auto rates = ParallelArmKinematics::inverseVelocity(joints, vx, vy, vz, geom);
  ASSERT_TRUE(rates.has_value());

  const double h = 1e-6;
  ParallelArmJointAngles joints_plus = joints;
  joints_plus.shoulder_motor_angle_rad += rates->shoulder_motor_angle_rate_rad_s * h;
  joints_plus.turntable_angle_rad += rates->turntable_angle_rate_rad_s * h;
  joints_plus.knee_motor_angle_rad += rates->knee_motor_angle_rate_rad_s * h;

  const auto pos0 = ParallelArmKinematics::forwardKinematics(joints, geom);
  const auto pos1 = ParallelArmKinematics::forwardKinematics(joints_plus, geom);
  ASSERT_TRUE(pos0.has_value());
  ASSERT_TRUE(pos1.has_value());

  EXPECT_NEAR((pos1->x - pos0->x) / h, vx, 1e-4);
  EXPECT_NEAR((pos1->y - pos0->y) / h, vy, 1e-4);
  EXPECT_NEAR((pos1->z - pos0->z) / h, vz, 1e-4);
}
