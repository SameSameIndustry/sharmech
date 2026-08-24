#ifndef SHARMECH_CORE__UTILITY__ORIENTATION_UTILS_HPP_
#define SHARMECH_CORE__UTILITY__ORIENTATION_UTILS_HPP_

#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace sharmech_core
{

struct PitchYaw
{
  double pitch;  // [rad]
  double yaw;    // [rad]
};

// エンドエフェクタ姿勢(クォータニオン) ↔ pitch/yaw 変換
// 手首はpitch/yawの2自由度(差動2モータ)のみのため、roll成分は無視する
class OrientationUtils
{
public:
  OrientationUtils() = delete;

  static PitchYaw toPitchYaw(const geometry_msgs::msg::Quaternion & orientation)
  {
    tf2::Quaternion q;
    tf2::fromMsg(orientation, q);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    return PitchYaw{pitch, yaw};
  }
};

}  // namespace sharmech_core

#endif  // SHARMECH_CORE__UTILITY__ORIENTATION_UTILS_HPP_
