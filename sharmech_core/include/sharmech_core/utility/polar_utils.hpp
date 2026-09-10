#ifndef SHARMECH_CORE__UTILITY__POLAR_UTILS_HPP_
#define SHARMECH_CORE__UTILITY__POLAR_UTILS_HPP_

#include <cmath>

namespace sharmech_core
{

// xy平面の直交座標 ⇄ 極座標 (r, θ) 変換
//
// ROS2 層は一貫して base 座標系の直交座標 (x, y, z) で扱うが、この機構の xy 平面は
// 「ターンテーブルで θ を、肩の対称二軸駆動で r を与える」r-θ 型のアームである
// (CLAUDE.md「ロボット構成」)。UDP でマイコンへ送る直前にここで極座標へ直す。
//
// **極座標の原点はターンテーブル回転軸。** ベース座標系原点 (設置エリア中心) と
// 一致する保証は無いので、軸の位置 (origin_x, origin_y) [m] を引数で受ける
// (値は robot_geometry.yaml の kinematics.turntable_axis_x/y_m。人間が実測して入れる)。
// 上流 (VR・motion_generator_node) と下流 (MCU) はこのオフセットを知らない。
// 補正はこの変換の 1 箇所で完結する。
// θ は +x 軸から反時計回り [rad]。
//
// **z・pitch・yaw は変換しない** (z は肘/膝機構が直接与えるため無関係、
// pitch/yaw は手首の姿勢)。

struct Polar
{
  double r;           // [m]   xy平面のターンテーブル軸からの距離
  double theta;       // [rad] +x軸からの角度 (連続化済み。±π を超えうる)
  double r_dot;       // [m/s]   r の時間微分
  double theta_dot;   // [rad/s] θ の時間微分
};

class PolarUtils
{
public:
  PolarUtils() = delete;

  // r がこれ未満のときは θ が定義できない (原点特異点)。
  // このとき θ は直前値を保持し、速度は 0 にして NaN/発散を防ぐ
  static constexpr double kMinRadius = 1e-6;

  // (x, y, vx, vy) → (r, θ, ṙ, θ̇)
  //
  // origin_x/y はターンテーブル軸のベース座標系での位置。変換前に引く
  // (並進のみ。軸の向きは MCU 側のエンコーダ零点で合わせる契約)。
  //
  // theta_ref は直前に送った θ。atan2 の値域 (-π, π] のままだと、-x 軸を
  // またぐ移動 (作業領域は X が -2.045〜+0.941 なのでシューティングボックス上で
  // 実際にまたぐ) で θ が +π ⇄ -π に飛び、ターンテーブルが1回転逆走する。
  // theta_ref に最も近い分岐へアンラップして連続値で返す。
  //
  // 速度は極座標での時間微分そのもの:
  //   ṙ = (x·vx + y·vy) / r
  //   θ̇ = (x·vy - y·vx) / r²
  static Polar toPolar(
    double x, double y, double vx, double vy, double theta_ref,
    double origin_x = 0.0, double origin_y = 0.0)
  {
    x -= origin_x;
    y -= origin_y;
    Polar out{};
    out.r = std::hypot(x, y);

    if (out.r < kMinRadius) {
      out.theta = theta_ref;
      out.r_dot = 0.0;
      out.theta_dot = 0.0;
      return out;
    }

    out.theta = unwrap(std::atan2(y, x), theta_ref);
    out.r_dot = (x * vx + y * vy) / out.r;
    out.theta_dot = (x * vy - y * vx) / (out.r * out.r);
    return out;
  }

  // (r, θ) → (x, y)。フィードバック (0x81) を ROS トピックへ戻すときに使う。
  // toPolar と同じ origin を渡すこと (往復で元の座標に戻る)。
  // θ がアンラップ済み (±π 超え) でも cos/sin は正しく扱える
  static double toX(double r, double theta, double origin_x = 0.0)
  {
    return origin_x + r * std::cos(theta);
  }
  static double toY(double r, double theta, double origin_y = 0.0)
  {
    return origin_y + r * std::sin(theta);
  }

  // theta を、reference から ±π 以内に収まる等価な角度へ移す
  static double unwrap(double theta, double reference)
  {
    constexpr double kTwoPi = 2.0 * M_PI;
    return theta + kTwoPi * std::round((reference - theta) / kTwoPi);
  }
};

}  // namespace sharmech_core

#endif  // SHARMECH_CORE__UTILITY__POLAR_UTILS_HPP_
