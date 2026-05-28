#ifndef SHARMECH_CORE__UTILITY__TRAJECTORY_UTILS_HPP_
#define SHARMECH_CORE__UTILITY__TRAJECTORY_UTILS_HPP_

#include <vector>
#include <cmath>
#include <stdexcept>
#include <geometry_msgs/msg/point.hpp>

namespace sharmech_core
{

// 軌道生成ユーティリティ (カルテシアン空間)
class TrajectoryUtils
{
public:
  TrajectoryUtils() = delete;

  // 始点〜終点を num_points 点で線形補間
  static std::vector<geometry_msgs::msg::Point> interpolateLinear(
    const geometry_msgs::msg::Point & start,
    const geometry_msgs::msg::Point & end,
    int num_points)
  {
    if (num_points < 2) {
      throw std::invalid_argument("num_points must be >= 2");
    }

    std::vector<geometry_msgs::msg::Point> path;
    path.reserve(num_points);

    for (int i = 0; i < num_points; ++i) {
      const double t = static_cast<double>(i) / (num_points - 1);
      geometry_msgs::msg::Point p;
      p.x = start.x + t * (end.x - start.x);
      p.y = start.y + t * (end.y - start.y);
      p.z = 0.0;
      path.push_back(p);
    }
    return path;
  }

  // 台形速度プロファイルで各ウェイポイントの到達時刻 [sec] を計算
  // v_max: 最大速度 [m/s]
  // a_max: 最大加速度 [m/s^2]
  static std::vector<double> trapezoidalTimeStamps(
    const std::vector<geometry_msgs::msg::Point> & waypoints,
    double v_max, double a_max)
  {
    if (waypoints.size() < 2) {
      return std::vector<double>(waypoints.size(), 0.0);
    }

    // 各区間の弧長を計算
    std::vector<double> seg_lengths;
    double total_length = 0.0;
    for (size_t i = 1; i < waypoints.size(); ++i) {
      const double dx = waypoints[i].x - waypoints[i - 1].x;
      const double dy = waypoints[i].y - waypoints[i - 1].y;
      const double len = std::hypot(dx, dy);
      seg_lengths.push_back(len);
      total_length += len;
    }

    if (total_length < 1e-9) {
      return std::vector<double>(waypoints.size(), 0.0);
    }

    // 台形プロファイルの総時間
    const double t_accel = v_max / a_max;
    const double d_accel = 0.5 * a_max * t_accel * t_accel;
    double total_time;

    if (2.0 * d_accel >= total_length) {
      // 三角プロファイル (v_maxに到達しない)
      const double t_peak = std::sqrt(total_length / a_max);
      total_time = 2.0 * t_peak;
    } else {
      total_time = 2.0 * t_accel + (total_length - 2.0 * d_accel) / v_max;
    }

    // 弧長に比例して各ウェイポイントの時刻を割り当て
    std::vector<double> timestamps;
    timestamps.push_back(0.0);
    double cumulative = 0.0;
    for (size_t i = 0; i < seg_lengths.size(); ++i) {
      cumulative += seg_lengths[i];
      timestamps.push_back((cumulative / total_length) * total_time);
    }
    return timestamps;
  }

  // 2点間のユークリッド距離
  static double distance(
    const geometry_msgs::msg::Point & a,
    const geometry_msgs::msg::Point & b)
  {
    return std::hypot(a.x - b.x, a.y - b.y);
  }
};

}  // namespace sharmech_core

#endif  // SHARMECH_CORE__UTILITY__TRAJECTORY_UTILS_HPP_
