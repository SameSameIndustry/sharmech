#ifndef SHARMECH_CORE__UTILITY__PARALLEL_ARM_KINEMATICS_HPP_
#define SHARMECH_CORE__UTILITY__PARALLEL_ARM_KINEMATICS_HPP_

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>

namespace sharmech_core
{

// 対称二軸駆動リンク1本分のパラメータ。
// 肩機構 (xy平面のr) と肘/膝機構 (z) はどちらもこの型で表せる
// (2026-08-27 ユーザー確認: 肘機構は「肘の部分にそれぞれモーターがあり、
//  角度は同じにする」= 肩機構と同型の対称二軸駆動)。
//
//   pivot_L(-a, 0) --proximal_link_length_m-- knee_L --distal_link_length_m--+
//                                                                            | EE (0, d)
//   pivot_R(+a, 0) --proximal_link_length_m-- knee_R --distal_link_length_m--+
//
// motor_angle は各ピボットにおいて「もう一方のピボットへ向かう方向」を 0 とし、
// EE 側 (+d方向) へ回転するほど増える向きとする。左右のピボットはミラー対称なので、
// motor_angle が同じ値であれば EE は常に垂直2等分線 (ローカル座標で x=0) 上に来る。
//
// 実機のモータ角ゼロ点・回転方向がこの通りである保証はまだ無い。
// MCU側の実際の角度基準が判明し次第、必要ならオフセット/符号をこの構造体の
// 外側で吸収すること。
//
// 以下の寸法は物理パラメータであり、CAD/実測が無い限りこのヘッダーでは
// 決め打ちしない。呼び出し側 (config.yaml 等) から与えること
struct SymmetricLinkageParams
{
  double pivot_half_separation_m{0.0};  // a: 2つの固定ピボット間距離の半分 [m]
  double proximal_link_length_m{0.0};   // L1: ピボット側リンク長 [m]
  double distal_link_length_m{0.0};     // L2: EE側リンク長 [m]
};

// 対称二軸駆動リンクの順運動学。
// motor_angle → 2つのピボットの中点から見た垂直2等分線上のEE距離 d
class SymmetricLinkage
{
public:
  SymmetricLinkage() = delete;

  // d = L1*sin(theta) + sqrt(L2^2 - (a - L1*cos(theta))^2)
  //
  // 導出: 右ピボット (+a, 0) からの第1関節位置は
  //   knee = (a - L1*cos(theta), L1*sin(theta))
  // であり、EE (0, d) までの距離が L2 になる (elbow-up側の解) という拘束から
  // d について解いたもの。左ピボットは対称なので同じ d を与える。
  //
  // 機構が物理的に届かない角度 (ルート内が負) では nullopt を返す
  static std::optional<double> computeExtension(
    double motor_angle_rad, const SymmetricLinkageParams & p)
  {
    const double knee_x = p.pivot_half_separation_m - p.proximal_link_length_m * std::cos(
      motor_angle_rad);
    const double knee_y = p.proximal_link_length_m * std::sin(motor_angle_rad);

    const double under_sqrt = p.distal_link_length_m * p.distal_link_length_m - knee_x * knee_x;
    if (under_sqrt < 0.0) {return std::nullopt;}

    return knee_y + std::sqrt(under_sqrt);
  }

  // 対称二軸駆動リンクの逆運動学。
  // 中点から見たEE距離 d → motor_angle (computeExtensionの逆関数)
  //
  // 導出: pivot(a,0) から見たEE(0,d)までの距離 r=hypot(a,d) を使い、
  // 2リンクの余弦定理で解く。
  //
  // 幾何的には (β-α, β+α) の2つの角度が、どちらもcomputeExtensionの"+sqrt"側の
  // 同じ曲線上で同じdを与えうる (肘の配置が2通りあるのではなく、この曲線自体が
  // θについて単調でないため。a=0の対称ケースだけで検証すると常にβ-αに見えるが、
  // a>0かつθが大きい領域ではもう一方の解も同じdを与えることをround-tripテストで
  // 確認済み)。そのため解析的にどちらかへ決め打ちせず、両方をcomputeExtensionへ
  // 逆投入して実際に一致する側を選ぶ。両方とも一致する場合は reference_angle_rad
  // (直前の関節角) に近い方を選び、関節角が滑らかに繋がるようにする。
  // reference_angle_rad が省略された場合は絶対値が小さい方を選ぶ
  //
  // 機構が物理的に届かない距離 (三角不等式を満たさない) では nullopt を返す
  static std::optional<double> computeMotorAngle(
    double extension_d, const SymmetricLinkageParams & p,
    std::optional<double> reference_angle_rad = std::nullopt)
  {
    const double l1 = p.proximal_link_length_m;
    const double l2 = p.distal_link_length_m;
    const double r = std::hypot(p.pivot_half_separation_m, extension_d);

    if (r < 1e-9) {return std::nullopt;}
    if (r > l1 + l2 || r < std::abs(l1 - l2)) {return std::nullopt;}

    const double cos_alpha = std::clamp((l1 * l1 + r * r - l2 * l2) / (2.0 * l1 * r), -1.0, 1.0);
    const double alpha = std::acos(cos_alpha);
    const double beta = std::atan2(extension_d, p.pivot_half_separation_m);

    const double candidates[2] = {beta - alpha, beta + alpha};
    std::optional<double> best;
    double best_score = std::numeric_limits<double>::infinity();
    for (const double candidate : candidates) {
      const auto d = computeExtension(candidate, p);
      if (!d || std::abs(*d - extension_d) > 1e-6) {continue;}

      const double score = reference_angle_rad ?
        std::abs(candidate - *reference_angle_rad) :
        std::abs(candidate);
      if (score < best_score) {
        best_score = score;
        best = candidate;
      }
    }
    return best;
  }

  // computeExtension の motor_angle に関する微分 d(extension)/d(motor_angle)。
  // 逆運動学の速度版 (関節角速度 ↔ 直線速度の変換) に使う。
  // 機構が速度特異点にある (根号内が0に近い) 場合は nullopt を返す
  static std::optional<double> computeExtensionDerivative(
    double motor_angle_rad, const SymmetricLinkageParams & p)
  {
    const double l1 = p.proximal_link_length_m;
    const double u = p.pivot_half_separation_m - l1 * std::cos(motor_angle_rad);
    const double under_sqrt = p.distal_link_length_m * p.distal_link_length_m - u * u;

    if (under_sqrt < 1e-9) {return std::nullopt;}

    const double sqrt_term = std::sqrt(under_sqrt);
    return l1 * std::cos(motor_angle_rad) -
           u * l1 * std::sin(motor_angle_rad) / sqrt_term;
  }
};

// 関節角速度 (肩・ターンテーブル・肘/膝、それぞれ代表値1つずつ)
struct ParallelArmJointRates
{
  double shoulder_motor_angle_rate_rad_s{0.0};
  double turntable_angle_rate_rad_s{0.0};
  double knee_motor_angle_rate_rad_s{0.0};
};

// ロボット全軸 (肩2 + ターンテーブル1 + 肘/膝2 = 5軸) の関節角。
// 肩・肘/膝はそれぞれ左右が同一角度で駆動される前提のため、
// 代表値1つずつで表す (左右別角度は現行の駆動方式では発生しない)
struct ParallelArmJointAngles
{
  double shoulder_motor_angle_rad{0.0};  // 肩モータ角 (左右とも同値)
  double turntable_angle_rad{0.0};       // ターンテーブル角 (xy平面内の方向)
  double knee_motor_angle_rad{0.0};      // 肘/膝モータ角 (左右とも同値)
};

// エンドエフェクタの位置 (姿勢は含まない。2026-08-27 時点でユーザーが
// 「まずは座標が一意に定まることを実装する」とスコープを明示したため)
struct EndEffectorPosition
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

// このロボット固有のジオメトリパラメータ一式
struct ParallelArmGeometry
{
  SymmetricLinkageParams shoulder;  // 肩機構 (xy平面のr)
  SymmetricLinkageParams knee;      // 肘/膝機構 (z)
  double turntable_axis_x_m{0.0};   // ターンテーブル回転軸のbase座標系でのx
  double turntable_axis_y_m{0.0};   // ターンテーブル回転軸のbase座標系でのy
  double knee_base_height_m{0.0};   // 肘/膝機構のd=0が対応するbase座標系でのz
};

// パターンB 順運動学: 関節角 → エンドエフェクタ座標。
//
// xy平面とzを独立に計算する (ユーザーが実装方針として明示):
//   xy: 肩機構がr (2ピボット中点からの距離) を、ターンテーブルがθを与える
//       r-θ型アーム
//   z : 肘/膝機構が同じ対称二軸駆動でzを直接与える (肩機構と数式は同型)
//
// 肩・肘/膝のどちらかが物理的に届かない角度の場合は nullopt を返す
class ParallelArmKinematics
{
public:
  ParallelArmKinematics() = delete;

  static std::optional<EndEffectorPosition> forwardKinematics(
    const ParallelArmJointAngles & joints, const ParallelArmGeometry & geom)
  {
    const auto r =
      SymmetricLinkage::computeExtension(joints.shoulder_motor_angle_rad, geom.shoulder);
    if (!r) {return std::nullopt;}

    const auto z_extension = SymmetricLinkage::computeExtension(
      joints.knee_motor_angle_rad,
      geom.knee);
    if (!z_extension) {return std::nullopt;}

    EndEffectorPosition pos;
    pos.x = geom.turntable_axis_x_m + (*r) * std::cos(joints.turntable_angle_rad);
    pos.y = geom.turntable_axis_y_m + (*r) * std::sin(joints.turntable_angle_rad);
    pos.z = geom.knee_base_height_m + (*z_extension);
    return pos;
  }

  // パターンB 逆運動学: エンドエフェクタ座標 → 関節角。
  //
  // xy平面とzを独立に計算する (forwardKinematicsの逆):
  //   xy: ターンテーブル軸からの相対位置の極座標分解で r, θ を得る
  //   z : 肘/膝機構の対称二軸駆動の逆関数で直接 motor_angle を得る
  //
  // SymmetricLinkage::computeMotorAngle と同じ理由で肩・肘/膝それぞれに
  // 2つの解があり得るため、reference (直前の関節角) を渡せば連続性を優先して
  // 解を選ぶ。省略時は絶対値最小の解を選ぶ
  //
  // ターンテーブル軸の直上 (r=0) はθが不定になる特異点、
  // 肩・肘/膝のどちらかが物理的に届かない場合は nullopt を返す
  static std::optional<ParallelArmJointAngles> inverseKinematics(
    const EndEffectorPosition & target, const ParallelArmGeometry & geom,
    std::optional<ParallelArmJointAngles> reference = std::nullopt)
  {
    const double dx = target.x - geom.turntable_axis_x_m;
    const double dy = target.y - geom.turntable_axis_y_m;
    const double r = std::hypot(dx, dy);
    if (r < 1e-9) {return std::nullopt;}

    const std::optional<double> shoulder_ref =
      reference ? std::optional<double>(reference->shoulder_motor_angle_rad) : std::nullopt;
    const auto shoulder_angle = SymmetricLinkage::computeMotorAngle(r, geom.shoulder, shoulder_ref);
    if (!shoulder_angle) {return std::nullopt;}

    const std::optional<double> knee_ref =
      reference ? std::optional<double>(reference->knee_motor_angle_rad) : std::nullopt;
    const double z_extension = target.z - geom.knee_base_height_m;
    const auto knee_angle = SymmetricLinkage::computeMotorAngle(z_extension, geom.knee, knee_ref);
    if (!knee_angle) {return std::nullopt;}

    ParallelArmJointAngles joints;
    joints.shoulder_motor_angle_rad = *shoulder_angle;
    joints.turntable_angle_rad = std::atan2(dy, dx);
    joints.knee_motor_angle_rad = *knee_angle;
    return joints;
  }

  // パターンB 速度逆運動学: 現在の関節角 + エンドエフェクタ速度 (vx,vy,vz) → 関節角速度。
  //
  // 「位置と速度は常に併送する」設計判断 (motor_generator_node/hardware_bridge_node と同様)
  // により、MCUへの関節指令にも角速度が必要となるため、位置IKの解析的な微分として導出する。
  //
  // xy平面は極座標(r,θ)の速度分解、zは肘/膝機構の微分の逆数を使う。
  // ターンテーブル軸直上 (r=0) や、肩・肘/膝が速度特異点にある場合は nullopt を返す
  static std::optional<ParallelArmJointRates> inverseVelocity(
    const ParallelArmJointAngles & joints, double vx, double vy, double vz,
    const ParallelArmGeometry & geom)
  {
    const auto r =
      SymmetricLinkage::computeExtension(joints.shoulder_motor_angle_rad, geom.shoulder);
    if (!r || *r < 1e-9) {return std::nullopt;}

    const double cos_phi = std::cos(joints.turntable_angle_rad);
    const double sin_phi = std::sin(joints.turntable_angle_rad);
    const double r_dot = vx * cos_phi + vy * sin_phi;
    const double phi_dot = (-vx * sin_phi + vy * cos_phi) / (*r);

    const auto d_shoulder =
      SymmetricLinkage::computeExtensionDerivative(joints.shoulder_motor_angle_rad, geom.shoulder);
    if (!d_shoulder || std::abs(*d_shoulder) < 1e-9) {return std::nullopt;}

    const auto d_knee =
      SymmetricLinkage::computeExtensionDerivative(joints.knee_motor_angle_rad, geom.knee);
    if (!d_knee || std::abs(*d_knee) < 1e-9) {return std::nullopt;}

    ParallelArmJointRates rates;
    rates.shoulder_motor_angle_rate_rad_s = r_dot / (*d_shoulder);
    rates.turntable_angle_rate_rad_s = phi_dot;
    rates.knee_motor_angle_rate_rad_s = vz / (*d_knee);
    return rates;
  }
};

}  // namespace sharmech_core

#endif  // SHARMECH_CORE__UTILITY__PARALLEL_ARM_KINEMATICS_HPP_
