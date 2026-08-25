#!/usr/bin/env python3
"""実機無しで hardware_bridge_node と UDP でやり取りする疑似 MCU。

プロトコル仕様は sharmech_core/docs/hardware_bridge_node.md および
sharmech_core/include/sharmech_core/utility/udp_protocol.hpp が正本。
このスクリプトは ROS2 に依存せず、標準ライブラリのみで動く。

デフォルト動作: Cartesian 指令 (packet_type=0x01) を受信するたびに、
指令された位置・姿勢へ即座に到達したものとしてフィードバック (0x81) を
送り返す (補間はしない)。実際の追従遅れを見たいときは --lag で
一次遅れを、--drop-rate でパケットロスをシミュレートできる。

使い方:
    python3 mock_mcu.py
    python3 mock_mcu.py --listen-port 8888 --drop-rate 0.01 --lag 0.05
"""
import argparse
import math
import random
import socket
import struct
import sys
import time

# CommandHeader (16B): version u8, type u8, payload_len u16, seq u32, ts_us u64
HEADER_FMT = "<BBHIQ"
HEADER_SIZE = struct.calcsize(HEADER_FMT)
assert HEADER_SIZE == 16

# CartesianPayload (44B)
CARTESIAN_PAYLOAD_FMT = "<10fBBH"
CARTESIAN_PAYLOAD_SIZE = struct.calcsize(CARTESIAN_PAYLOAD_FMT)
assert CARTESIAN_PAYLOAD_SIZE == 44

# FeedbackFixedPart の固定部 (ヘッダを除く、28B): seq_echo u32, xyz/pitch/yaw f32*5,
# status_flags u16, gripper_state u8, joint_count u8
FEEDBACK_FIXED_FMT = "<I5fHBB"
FEEDBACK_FIXED_SIZE = struct.calcsize(FEEDBACK_FIXED_FMT)
assert FEEDBACK_FIXED_SIZE == 28

PROTOCOL_VERSION = 1
PACKET_TYPE_CARTESIAN = 0x01
PACKET_TYPE_STATE_FEEDBACK = 0x81


def decode_cartesian(data: bytes):
    if len(data) < HEADER_SIZE + CARTESIAN_PAYLOAD_SIZE:
        return None
    version, ptype, payload_len, seq, ts_us = struct.unpack_from(HEADER_FMT, data, 0)
    if version != PROTOCOL_VERSION or ptype != PACKET_TYPE_CARTESIAN:
        return None
    if payload_len != CARTESIAN_PAYLOAD_SIZE:
        return None
    (x, y, z, pitch, yaw, vx, vy, vz, pitch_rate, yaw_rate,
     gripper, control_flags, _reserved) = struct.unpack_from(
        CARTESIAN_PAYLOAD_FMT, data, HEADER_SIZE)
    return {
        "seq": seq, "timestamp_us": ts_us,
        "x": x, "y": y, "z": z, "pitch": pitch, "yaw": yaw,
        "vx": vx, "vy": vy, "vz": vz,
        "pitch_rate": pitch_rate, "yaw_rate": yaw_rate,
        "gripper": gripper, "control_flags": control_flags,
    }


def encode_feedback(seq, seq_echo, timestamp_us, x, y, z, pitch, yaw,
                     status_flags, gripper_closed, joint_positions):
    payload_len = FEEDBACK_FIXED_SIZE + 4 * len(joint_positions)
    header = struct.pack(
        HEADER_FMT, PROTOCOL_VERSION, PACKET_TYPE_STATE_FEEDBACK,
        payload_len, seq, timestamp_us)
    fixed = struct.pack(
        FEEDBACK_FIXED_FMT, seq_echo, x, y, z, pitch, yaw,
        status_flags, 1 if gripper_closed else 0, len(joint_positions))
    joints = struct.pack("<%df" % len(joint_positions), *joint_positions) \
        if joint_positions else b""
    return header + fixed + joints


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                      formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--listen-port", type=int, default=8888,
                         help="hardware_bridge_node の mcu_port と合わせる (既定: 8888)")
    parser.add_argument("--drop-rate", type=float, default=0.0,
                         help="受信・送信をランダムに破棄する確率 [0,1] (パケットロス試験用)")
    parser.add_argument("--lag", type=float, default=0.0,
                         help="指令位置への一次遅れ時定数 [s]。0 なら即座に到達 (既定)")
    parser.add_argument("--joint-count", type=int, default=0,
                         help="feedback に載せるダミー関節角の数 (既定: 0。joint_names 未確定のため)")
    parser.add_argument("--status-flags", type=lambda v: int(v, 0), default=0,
                         help="常に載せる status_flags (異常系の手動試験用。例: 0x4 = watchdog)")
    parser.add_argument("--quiet", action="store_true", help="受信ログを抑制する")
    args = parser.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", args.listen_port))
    print(f"[mock_mcu] listening on :{args.listen_port} "
          f"(drop_rate={args.drop_rate}, lag={args.lag}s)", file=sys.stderr)

    # 現在の「実位置」。--lag が 0 なら毎回コマンド値に即座に一致させる
    current = {"x": 0.0, "y": 0.0, "z": 0.0, "pitch": 0.0, "yaw": 0.0}
    gripper_closed = False
    feedback_seq = 0
    last_recv_seq = None
    last_recv_time = None
    joints = [0.0] * args.joint_count

    try:
        while True:
            data, addr = sock.recvfrom(2048)
            now = time.monotonic()

            if args.drop_rate > 0.0 and random.random() < args.drop_rate:
                continue  # 受信パケットロスをシミュレート

            cmd = decode_cartesian(data)
            if cmd is None:
                if not args.quiet:
                    print(f"[mock_mcu] discarded unrecognized packet ({len(data)}B) from {addr}",
                          file=sys.stderr)
                continue

            last_recv_seq = cmd["seq"]
            gripper_closed = bool(cmd["gripper"])

            if args.lag <= 0.0:
                current["x"], current["y"], current["z"] = cmd["x"], cmd["y"], cmd["z"]
                current["pitch"], current["yaw"] = cmd["pitch"], cmd["yaw"]
            else:
                # 一次遅れ: alpha = 1 - exp(-dt/lag)。受信間隔 dt は毎回測る
                dt = 0.0 if last_recv_time is None else max(0.0, now - last_recv_time)
                alpha = 1.0 - math.exp(-dt / args.lag)
                for key in ("x", "y", "z", "pitch", "yaw"):
                    current[key] += (cmd[key] - current[key]) * alpha
            last_recv_time = now

            if not args.quiet:
                print(f"[mock_mcu] recv seq={cmd['seq']:6d} "
                      f"pos=({cmd['x']:+.3f},{cmd['y']:+.3f},{cmd['z']:+.3f}) "
                      f"gripper={'closed' if gripper_closed else 'open'}", file=sys.stderr)

            if args.drop_rate > 0.0 and random.random() < args.drop_rate:
                continue  # 送信側のパケットロスをシミュレート

            feedback_seq += 1
            packet = encode_feedback(
                feedback_seq, last_recv_seq, time.monotonic_ns() // 1000,
                current["x"], current["y"], current["z"],
                current["pitch"], current["yaw"],
                args.status_flags, gripper_closed, joints)
            sock.sendto(packet, addr)
    except KeyboardInterrupt:
        print("\n[mock_mcu] stopped", file=sys.stderr)


if __name__ == "__main__":
    main()
