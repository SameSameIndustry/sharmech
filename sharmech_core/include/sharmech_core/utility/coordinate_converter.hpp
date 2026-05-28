#ifndef SHARMECH_CORE__UTILITY__COORDINATE_CONVERTER_HPP_
#define SHARMECH_CORE__UTILITY__COORDINATE_CONVERTER_HPP_

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

namespace sharmech_core
{

// Unity(左手Y-up) ↔ ROS2(右手Z-up) 座標変換
//
// Unity: X=右, Y=上, Z=奥  (左手系)
// ROS2:  X=前, Y=左, Z=上  (右手系)
//
// 位置変換:
//   ROS.x =  Unity.z
//   ROS.y = -Unity.x
//   ROS.z =  Unity.y
//
// クォータニオン変換:
//   ROS.x = -Unity.z
//   ROS.y =  Unity.x
//   ROS.z = -Unity.y
//   ROS.w =  Unity.w
class CoordinateConverter
{
public:
  CoordinateConverter() = delete;

  static geometry_msgs::msg::PoseStamped unityToRos(
    const geometry_msgs::msg::PoseStamped & unity_pose)
  {
    geometry_msgs::msg::PoseStamped ros_pose;
    ros_pose.header = unity_pose.header;
    ros_pose.header.frame_id = "world";

    const auto & p = unity_pose.pose.position;
    ros_pose.pose.position.x =  p.z;
    ros_pose.pose.position.y = -p.x;
    ros_pose.pose.position.z =  p.y;

    const auto & q = unity_pose.pose.orientation;
    ros_pose.pose.orientation.x = -q.z;
    ros_pose.pose.orientation.y =  q.x;
    ros_pose.pose.orientation.z = -q.y;
    ros_pose.pose.orientation.w =  q.w;

    return ros_pose;
  }

  static geometry_msgs::msg::PoseStamped rosToUnity(
    const geometry_msgs::msg::PoseStamped & ros_pose)
  {
    geometry_msgs::msg::PoseStamped unity_pose;
    unity_pose.header = ros_pose.header;
    unity_pose.header.frame_id = "unity_world";

    const auto & p = ros_pose.pose.position;
    unity_pose.pose.position.x = -p.y;
    unity_pose.pose.position.y =  p.z;
    unity_pose.pose.position.z =  p.x;

    const auto & q = ros_pose.pose.orientation;
    unity_pose.pose.orientation.x =  q.y;
    unity_pose.pose.orientation.y = -q.z;
    unity_pose.pose.orientation.z = -q.x;
    unity_pose.pose.orientation.w =  q.w;

    return unity_pose;
  }

  static geometry_msgs::msg::Point unityPointToRos(
    const geometry_msgs::msg::Point & p)
  {
    geometry_msgs::msg::Point r;
    r.x =  p.z;
    r.y = -p.x;
    r.z =  p.y;
    return r;
  }

  static geometry_msgs::msg::Point rosPointToUnity(
    const geometry_msgs::msg::Point & p)
  {
    geometry_msgs::msg::Point u;
    u.x = -p.y;
    u.y =  p.z;
    u.z =  p.x;
    return u;
  }
};

}  // namespace sharmech_core

#endif  // SHARMECH_CORE__UTILITY__COORDINATE_CONVERTER_HPP_
