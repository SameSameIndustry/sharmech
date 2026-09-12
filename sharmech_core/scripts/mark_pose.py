#!/usr/bin/env python3
"""
PS5 (DualSense) の □ ボタンを押した瞬間のロボット先端位置を CSV に記録する.

用途: WebXR 側の仮想オブジェクト (ワーク・箱など) の配置が実物とずれているので、
実機をジョグで実物の位置へ合わせて □ を押し、その座標を控える。記録後に
「ID N はどのオブジェクトか」を人間が label 列に書き (または Claude に伝え)、
オブジェクト間の既知の距離から WebXR 側の配置を補正する。

使い方 (sharmech.launch.xml と joy_node が動いている状態で、別端末から):
    python3 sharmech_core/scripts/mark_pose.py                 # ./pose_marks_<日時>.csv に書く
    python3 sharmech_core/scripts/mark_pose.py --out marks.csv
    python3 sharmech_core/scripts/mark_pose.py --button 1      # ○ で記録したいとき

記録 1 行 = id, label (空。後で人間が埋める), time, x_m, y_m, z_m, r_m, theta_rad, theta_deg
  - x/y/z は /catchrobo/arm/current_pose そのまま (field = ベース座標系 [m])。
    WebXR の配置はこの直交座標を使う
  - r/θ は UDP と同じ極座標 (mcu_spec.md §3.2/§5): 原点 = ターンテーブル軸
    (--axis-x/--axis-y、既定は robot_geometry.yaml の turntable_axis_x/y_m = 0.0)、
    θ は +X から時計回りが正 (polar_utils.hpp の kThetaSign = -1)。
    θ は (-π, π] に畳んだ値 (アンラップはしない)
  - 記録できたら DualSense を短く振動させる (/joy/set_feedback)

/catchrobo/arm/current_pose が --stale 秒以上届いていない (MCU 途絶中) ときは
古い位置を記録しないよう警告だけ出して記録しない。
"""

import argparse
import csv
import datetime
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import Joy, JoyFeedback

# polar_utils.hpp の kThetaSign と同じ (ワイヤ上の θ は時計回りが正)
THETA_SIGN = -1.0
FIELDS = ["id", "label", "time", "x_m", "y_m", "z_m", "r_m", "theta_rad", "theta_deg"]


class PoseMarker(Node):
    def __init__(self, args):
        super().__init__("pose_marker")
        self.button = args.button
        self.axis_x = args.axis_x
        self.axis_y = args.axis_y
        self.stale_sec = args.stale
        self.pose = None
        self.pose_time = None
        self.prev_pressed = False
        self.next_id = 1

        self.file = open(args.out, "w", newline="")
        self.writer = csv.DictWriter(self.file, fieldnames=FIELDS)
        self.writer.writeheader()
        self.file.flush()

        self.create_subscription(Joy, "/joy", self.on_joy, 10)
        self.create_subscription(PoseStamped, "/catchrobo/arm/current_pose", self.on_pose, 10)
        self.feedback_pub = self.create_publisher(JoyFeedback, "/joy/set_feedback", 10)
        self.rumble_stop_timer = None

        self.get_logger().info(
            f"writing to {args.out}; press button {self.button} to record "
            f"(axis origin = ({self.axis_x}, {self.axis_y}))")

    def on_pose(self, msg):
        self.pose = msg.pose.position
        self.pose_time = self.get_clock().now()

    def on_joy(self, msg):
        pressed = self.button < len(msg.buttons) and msg.buttons[self.button] == 1
        if pressed and not self.prev_pressed:
            self.mark()
        self.prev_pressed = pressed

    def mark(self):
        if self.pose is None:
            self.get_logger().warn("no /catchrobo/arm/current_pose yet; not recorded")
            return
        age = (self.get_clock().now() - self.pose_time).nanoseconds * 1e-9
        if age > self.stale_sec:
            self.get_logger().warn(f"current_pose is {age:.1f}s old (MCU down?); not recorded")
            return

        x, y, z = self.pose.x, self.pose.y, self.pose.z
        dx, dy = x - self.axis_x, y - self.axis_y
        r = math.hypot(dx, dy)
        theta = THETA_SIGN * math.atan2(dy, dx)
        row = {
            "id": self.next_id,
            "label": "",
            "time": datetime.datetime.now().isoformat(timespec="milliseconds"),
            "x_m": f"{x:.4f}", "y_m": f"{y:.4f}", "z_m": f"{z:.4f}",
            "r_m": f"{r:.4f}", "theta_rad": f"{theta:.4f}",
            "theta_deg": f"{math.degrees(theta):.2f}",
        }
        self.writer.writerow(row)
        self.file.flush()
        self.get_logger().info(
            f"#{self.next_id}: x={x:.3f} y={y:.3f} z={z:.3f} | "
            f"r={r:.3f} theta={theta:.3f}rad ({math.degrees(theta):.1f}deg)")
        self.next_id += 1
        self.rumble()

    # joy_node は止める指令を送るまで鳴り続けるので、短く鳴らして止める
    def rumble(self):
        self.feedback_pub.publish(JoyFeedback(type=JoyFeedback.TYPE_RUMBLE, id=0, intensity=0.4))
        if self.rumble_stop_timer is not None:
            self.rumble_stop_timer.cancel()
        self.rumble_stop_timer = self.create_timer(0.06, self.rumble_stop)

    def rumble_stop(self):
        self.rumble_stop_timer.cancel()
        self.feedback_pub.publish(JoyFeedback(type=JoyFeedback.TYPE_RUMBLE, id=0, intensity=0.0))


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument(
        "--out",
        default=datetime.datetime.now().strftime("pose_marks_%Y%m%d_%H%M%S.csv"),
        help="出力 CSV (既定: カレントディレクトリに日時付きで新規作成)")
    parser.add_argument("--button", type=int, default=3, help="/joy のボタン番号 (既定 3 = □。×=0 ○=1 △=2)")
    parser.add_argument("--axis-x", type=float, default=0.0, help="ターンテーブル軸の X [m] (極座標の原点)")
    parser.add_argument("--axis-y", type=float, default=0.0, help="ターンテーブル軸の Y [m]")
    parser.add_argument("--stale", type=float, default=1.0, help="current_pose がこの秒数より古ければ記録しない")
    args = parser.parse_args()

    rclpy.init()
    node = PoseMarker(args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.file.close()
        print(f"{node.next_id - 1} record(s) written to {args.out}")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
