#!/usr/bin/env python3
"""
実機無しで hardware_bridge_node と UDP でやり取りする疑似 MCU (mock).

プロトコル仕様は sharmech_core/docs/hardware_bridge_node.md および
sharmech_core/include/sharmech_core/utility/udp_protocol.hpp が正本。
このスクリプトは ROS2 に依存せず、標準ライブラリのみで動く。

デフォルト動作: 極座標指令 (packet_type=0x01) を受信すると、指令された
位置・姿勢へ即座に到達したものとして内部状態を更新する (補間はしない)。
**xy平面は極座標 (r, θ) で送受信する (protocol_version 2)。** θ は ROS2 側が
アンラップした連続値で届くが、本モックは値をそのまま保持してエコーするだけ
なので ±π の外でも扱いは変わらない。
フィードバック (0x81) は仕様どおり **指令の受信とは独立に一定周期
(--feedback-rate、既定100Hz) で自発送信する** (2026-09-01確定の契約。
実MCUは起動直後から送るが、本モックは送り先アドレスを知らないため
最初の指令パケットを受けてから送り始める)。
実際の追従遅れを見たいときは --lag で一次遅れを、--drop-rate で
パケットロスをシミュレートできる。

関節指令 (packet_type=0x02、パターンB) も受理する。この場合は指令された
関節角をそのままフィードバックの joint_positions にエコーバックする。
ただし本スクリプトは FK を持たないため、0x02 に対する r/θ/z/pitch/yaw の
フィードバックは最後の値のまま更新されない (実機のMCUはFKして返す契約。
hardware_bridge_node 側のプロトコル疎通確認用と割り切ること)。

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

# PolarPayload (44B): r, theta, z, pitch, yaw, r_dot, theta_dot, vz,
# pitch_rate, yaw_rate, gripper, control_flags, reserved
POLAR_PAYLOAD_FMT = "<10fBBH"
POLAR_PAYLOAD_SIZE = struct.calcsize(POLAR_PAYLOAD_FMT)
assert POLAR_PAYLOAD_SIZE == 44

# JointPayload (44B): q[5] + qdot[5] + gripper + control_flags + reserved。
# 並び順は [shoulder_left, shoulder_right, turntable, knee_left, knee_right]
JOINT_COUNT = 5
JOINT_PAYLOAD_FMT = "<10fBBH"
JOINT_PAYLOAD_SIZE = struct.calcsize(JOINT_PAYLOAD_FMT)
assert JOINT_PAYLOAD_SIZE == 44

# FeedbackFixedPart の固定部 (ヘッダを除く、28B): seq_echo u32,
# r/theta/z/pitch/yaw f32*5, status_flags u16, gripper_state u8, joint_count u8
FEEDBACK_FIXED_FMT = "<I5fHBB"
FEEDBACK_FIXED_SIZE = struct.calcsize(FEEDBACK_FIXED_FMT)
assert FEEDBACK_FIXED_SIZE == 28

# 2 = xy平面を極座標 (r, θ) に変更した版 (udp_protocol.hpp の kProtocolVersion)
PROTOCOL_VERSION = 2
PACKET_TYPE_POLAR = 0x01
PACKET_TYPE_JOINT = 0x02
PACKET_TYPE_STATE_FEEDBACK = 0x81

# status_flags のビット定義 (udp_protocol.hpp の kStatus* と一致させること)
FLAG_TRACKING_ERROR = 1 << 0
FLAG_DRIVER_FAULT = 1 << 1
FLAG_WATCHDOG = 1 << 2
FLAG_UNINITIALIZED = 1 << 3
FLAG_COMMAND_REJECTED = 1 << 4


def decode_polar(data: bytes):
    if len(data) < HEADER_SIZE + POLAR_PAYLOAD_SIZE:
        return None
    version, ptype, payload_len, seq, ts_us = struct.unpack_from(HEADER_FMT, data, 0)
    if version != PROTOCOL_VERSION or ptype != PACKET_TYPE_POLAR:
        return None
    if payload_len != POLAR_PAYLOAD_SIZE:
        return None
    (r, theta, z, pitch, yaw, r_dot, theta_dot, vz, pitch_rate, yaw_rate,
     gripper, control_flags, _reserved) = struct.unpack_from(
        POLAR_PAYLOAD_FMT, data, HEADER_SIZE)
    return {
        "seq": seq, "timestamp_us": ts_us,
        "r": r, "theta": theta, "z": z, "pitch": pitch, "yaw": yaw,
        "r_dot": r_dot, "theta_dot": theta_dot, "vz": vz,
        "pitch_rate": pitch_rate, "yaw_rate": yaw_rate,
        "gripper": gripper, "control_flags": control_flags,
    }


def decode_joint(data: bytes):
    if len(data) < HEADER_SIZE + JOINT_PAYLOAD_SIZE:
        return None
    version, ptype, payload_len, seq, ts_us = struct.unpack_from(HEADER_FMT, data, 0)
    if version != PROTOCOL_VERSION or ptype != PACKET_TYPE_JOINT:
        return None
    if payload_len != JOINT_PAYLOAD_SIZE:
        return None
    values = struct.unpack_from(JOINT_PAYLOAD_FMT, data, HEADER_SIZE)
    return {
        "seq": seq, "timestamp_us": ts_us,
        "q": list(values[0:JOINT_COUNT]),
        "qdot": list(values[JOINT_COUNT:2 * JOINT_COUNT]),
        "gripper": values[10], "control_flags": values[11],
    }


def encode_feedback(seq, seq_echo, timestamp_us, r, theta, z, pitch, yaw,
                    status_flags, gripper_closed, joint_positions):
    payload_len = FEEDBACK_FIXED_SIZE + 4 * len(joint_positions)
    header = struct.pack(
        HEADER_FMT, PROTOCOL_VERSION, PACKET_TYPE_STATE_FEEDBACK,
        payload_len, seq, timestamp_us)
    fixed = struct.pack(
        FEEDBACK_FIXED_FMT, seq_echo, r, theta, z, pitch, yaw,
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
                        help="0x01運用時に feedback に載せるダミー関節角の数 (既定: 0)。"
                             "0x02 (関節指令) を受信すると以後は5関節のエコーに切り替わる")
    parser.add_argument("--status-flags", type=lambda v: int(v, 0), default=0,
                        help="常に載せる status_flags (異常系の手動試験用。例: 0x4 = watchdog)")
    parser.add_argument("--tracking-error-limit", type=float, default=0.0,
                        help="追従誤差(r-z平面の距離[m])がこれを超えたら FLAG_TRACKING_ERROR を立てる "
                             "(0 で無効。--lag と併用すると実機に近い立ち方をする)")
    parser.add_argument("--watchdog-timeout", type=float, default=0.05,
                        help="指令がこの秒数途絶したら FLAG_WATCHDOG を立てて外挿を止める "
                             "(既定: 0.05 = 仕様の推奨50ms。0 で無効)")
    parser.add_argument("--feedback-rate", type=float, default=100.0,
                        help="フィードバック(0x81)の自発送信周期 [Hz] (既定: 100。0 で"
                             "旧来のエコー型に戻る)")
    parser.add_argument("--quiet", action="store_true", help="受信ログを抑制する")
    args = parser.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", args.listen_port))
    # フィードバックを指令の受信と独立に送るため、受信はブロックしない
    sock.settimeout(0.002)
    print(f"[mock_mcu] listening on :{args.listen_port} "
          f"(drop_rate={args.drop_rate}, lag={args.lag}s, "
          f"feedback={args.feedback_rate}Hz)", file=sys.stderr)

    # 現在の「実位置」。--lag が 0 なら毎回コマンド値に即座に一致させる
    current = {"r": 0.0, "theta": 0.0, "z": 0.0, "pitch": 0.0, "yaw": 0.0}
    gripper_closed = False
    feedback_seq = 0
    last_recv_seq = None
    last_recv_time = None
    last_cmd_pos = None      # 追従誤差フラグ用の直近の極座標指令位置 (r, θ, z)
    joints = [0.0] * args.joint_count
    client_addr = None       # フィードバックの宛先 (最初に指令をくれた相手)
    rejected_until = 0.0     # seq逆転破棄をbit4で通知しておく期限
    feedback_period = (1.0 / args.feedback_rate) if args.feedback_rate > 0 else None
    last_feedback_send = 0.0

    try:
        while True:
            recv_data = None
            recv_addr = None
            try:
                recv_data, recv_addr = sock.recvfrom(2048)
            except socket.timeout:
                pass
            now = time.monotonic()

            # ---- 受信処理 ----
            processed = False
            if recv_data is not None and not (
                    args.drop_rate > 0.0 and random.random() < args.drop_rate):
                cmd = decode_polar(recv_data)
                joint_cmd = None if cmd is not None else decode_joint(recv_data)
                if cmd is None and joint_cmd is None:
                    if not args.quiet:
                        print(f"[mock_mcu] discarded unrecognized packet "
                              f"({len(recv_data)}B) from {recv_addr}", file=sys.stderr)
                else:
                    client_addr = recv_addr
                    seq = (cmd or joint_cmd)["seq"]
                    if last_recv_seq is not None and seq <= last_recv_seq:
                        # seq逆転 (UDPの順序逆転)。要求事項4どおり破棄し、
                        # FLAG_COMMAND_REJECTED をしばらく立てて通知する
                        rejected_until = now + 0.2
                        if not args.quiet:
                            print(f"[mock_mcu] rejected out-of-order seq={seq} "
                                  f"(last={last_recv_seq})", file=sys.stderr)
                    else:
                        # 一次遅れ係数: alpha = 1 - exp(-dt/lag)
                        dt = 0.0 if last_recv_time is None else max(0.0, now - last_recv_time)
                        alpha = 1.0 if args.lag <= 0.0 else 1.0 - math.exp(-dt / args.lag)
                        last_recv_seq = seq
                        last_recv_time = now
                        processed = True
                        if cmd is not None:
                            gripper_closed = bool(cmd["gripper"])
                            last_cmd_pos = (cmd["r"], cmd["theta"], cmd["z"])
                            for key in ("r", "theta", "z", "pitch", "yaw"):
                                current[key] += (cmd[key] - current[key]) * alpha
                            if not args.quiet:
                                print(f"[mock_mcu] recv seq={seq:6d} "
                                      f"pos=(r={cmd['r']:+.3f},th={cmd['theta']:+.3f},"
                                      f"z={cmd['z']:+.3f}) "
                                      f"gripper={'closed' if gripper_closed else 'open'}",
                                      file=sys.stderr)
                        else:
                            # 0x02 (パターンB): 関節角をエコーバックする。FKを持たないため
                            # r/theta/z/pitch/yaw は更新しない (docstring参照)
                            gripper_closed = bool(joint_cmd["gripper"])
                            if len(joints) != JOINT_COUNT:
                                joints = [0.0] * JOINT_COUNT
                            for i in range(JOINT_COUNT):
                                joints[i] += (joint_cmd["q"][i] - joints[i]) * alpha
                            if not args.quiet:
                                q_text = ",".join(f"{v:+.3f}" for v in joint_cmd["q"])
                                print(f"[mock_mcu] recv seq={seq:6d} joint q=({q_text}) "
                                      f"gripper={'closed' if gripper_closed else 'open'}",
                                      file=sys.stderr)

            # ---- フィードバック送信 (既定: 指令の受信と独立な一定周期) ----
            if feedback_period is not None:
                should_send = client_addr is not None and \
                    (now - last_feedback_send >= feedback_period)
            else:
                should_send = processed  # --feedback-rate 0: 旧来のエコー型
            if not should_send:
                continue
            last_feedback_send = now

            # status_flags を実際の状態から組み立てる。
            # --status-flags で明示指定したビットは常に立てたままにする
            status_flags = args.status_flags
            silence = None if last_recv_time is None else now - last_recv_time
            if args.watchdog_timeout > 0.0 and silence is not None \
                    and silence > args.watchdog_timeout:
                # ウォッチドッグ発動 = 外挿停止・最後の目標位置をホールド (contract)。
                # 本モックは元々外挿しないので、フラグを立てるだけで挙動は同じ
                status_flags |= FLAG_WATCHDOG
            if now < rejected_until:
                status_flags |= FLAG_COMMAND_REJECTED
            if args.tracking_error_limit > 0.0 and last_cmd_pos is not None:
                # 極座標のまま距離を測ると θ [rad] と r/z [m] の単位が混ざるので、
                # 直交座標へ戻してから誤差を測る
                error = math.dist(
                    (current["r"] * math.cos(current["theta"]),
                     current["r"] * math.sin(current["theta"]),
                     current["z"]),
                    (last_cmd_pos[0] * math.cos(last_cmd_pos[1]),
                     last_cmd_pos[0] * math.sin(last_cmd_pos[1]),
                     last_cmd_pos[2]))
                if error > args.tracking_error_limit:
                    status_flags |= FLAG_TRACKING_ERROR

            if args.drop_rate > 0.0 and random.random() < args.drop_rate:
                continue  # 送信側のパケットロスをシミュレート

            feedback_seq += 1
            packet = encode_feedback(
                feedback_seq, last_recv_seq if last_recv_seq is not None else 0,
                time.monotonic_ns() // 1000,
                current["r"], current["theta"], current["z"],
                current["pitch"], current["yaw"],
                status_flags, gripper_closed, joints)
            sock.sendto(packet, client_addr)
    except KeyboardInterrupt:
        print("\n[mock_mcu] stopped", file=sys.stderr)


if __name__ == "__main__":
    main()
