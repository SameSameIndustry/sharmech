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
(--feedback-rate、既定10Hz。2026-09-11 に 100Hz から変更) で自発送信する** (2026-09-01確定の契約。
実MCUは起動直後から送るが、本モックは送り先アドレスを知らないため
最初の指令パケットを受けてから送り始める)。
実際の追従遅れを見たいときは --lag で一次遅れを、--drop-rate で
パケットロスをシミュレートできる。

起動シーケンスと初期位置 (2026-09-10 の契約。mcu_spec.md §4.6):
  - 起動直後は **自前の初期位置 (--init-pose r,theta,z) に居る** ものとし、
    --init-duration 秒の間は FLAG_UNINITIALIZED (bit3) を立てて「原点出し中」を
    再現する。ROS2 側 (motion_generator_node) はこの間同期せず、動作許可 0 を送る
  - control_flags bit0 (動作許可) が 0 のパケットは **位置を更新しない** (ホールド)
  - control_flags bit2 (初期位置要求) が立っている間は r/θ/z を無視して
    --init-pose へ (--lag があれば一次遅れで) 戻り、到達したら
    FLAG_AT_INIT_POSE (bit5) を立てる。bit2 が落ちたら bit5 も落ちる。
    **初期位置の座標は ROS2 側に無い** (実機では MCU が初期関節角として持つ)

関節指令 (packet_type=0x02、パターンB) も受理する。この場合は指令された
関節角をそのままフィードバックの joint_positions にエコーバックする。
ただし本スクリプトは FK を持たないため、0x02 に対する r/θ/z/pitch/yaw の
フィードバックは最後の値のまま更新されない (実機のMCUはFKして返す契約。
hardware_bridge_node 側のプロトコル疎通確認用と割り切ること)。

2 枚構成 (2026-09-10。mcu_spec.md §2.2): --board arm / --board theta で「自分の担当
フィールドだけ埋めて他は 0」を再現する。r/z 基板役と θ 基板役を別ポートで 2 つ立て、
launch の mock_mcu_two_boards:=true (config/mock_mcu_two_boards.yaml) で繋ぐ。

使い方:
    python3 mock_mcu.py
    python3 mock_mcu.py --listen-port 8888 --drop-rate 0.01 --lag 0.05
    python3 mock_mcu.py --listen-port 8888 --board arm     # 2枚構成: r/z 基板役
    python3 mock_mcu.py --listen-port 8890 --board theta   # 2枚構成: θ 基板役
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
TURNTABLE_JOINT_INDEX = 2   # --board theta が埋めるスロット (feedback_merge.hpp と一致)
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
FLAG_AT_INIT_POSE = 1 << 5

# control_flags のビット定義 (udp_protocol.hpp の kControlFlag* と一致させること)
CONTROL_ENABLE = 0x01
CONTROL_ORIENT_VERTICAL = 0x02
CONTROL_INIT_REQUEST = 0x04

# 初期位置「到達」とみなす許容差 (r/z は [m]、θ は [rad] だが仮に同じ閾値)
INIT_ARRIVAL_TOLERANCE = 1e-3


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
    parser.add_argument("--watchdog-timeout", type=float, default=0.5,
                        help="指令がこの秒数途絶したら FLAG_WATCHDOG を立てて外挿を止める "
                             "(既定: 0.5 = 仕様の推奨500ms。指令が 10Hz (100ms 間隔) なので "
                             "50ms だと毎パケット間で発動する。0 で無効)")
    parser.add_argument("--feedback-rate", type=float, default=10.0,
                        help="フィードバック(0x81)の自発送信周期 [Hz] (既定: 10 = 仕様 §3.4。"
                             "2026-09-11 に 100 から変更。0 で旧来のエコー型に戻る)")
    parser.add_argument("--init-pose", type=str, default="0.30,0.0,0.15",
                        help="MCU 側で定義した初期位置 'r,theta,z' (m,rad,m)。起動直後の実位置であり、"
                             "control_flags bit2 (初期位置要求) で戻る先 (既定: 0.30,0.0,0.15)")
    parser.add_argument("--init-duration", type=float, default=0.5,
                        help="起動後この秒数は FLAG_UNINITIALIZED を立てて原点出し中を再現する "
                             "(既定 0.5s。0 で即初期化済み)")
    parser.add_argument("--board", choices=("all", "arm", "theta"), default="all",
                        help="2枚構成の役割。arm = r/z 基板 (r, z, 肩・肘/膝の関節角だけ"
                             "埋めて他は 0)、theta = θ 基板 (θ, ターンテーブル角, pitch/yaw,"
                             " gripper だけ埋める)。all = 1枚構成 (既定、全部埋める)。"
                             "arm/theta では --joint-count 0 を 5 に読み替える")
    parser.add_argument("--quiet", action="store_true", help="受信ログを抑制する")
    args = parser.parse_args()
    if args.board != "all" and args.joint_count == 0:
        args.joint_count = JOINT_COUNT

    try:
        init_r, init_theta, init_z = (float(v) for v in args.init_pose.split(","))
    except ValueError:
        parser.error(f"--init-pose は 'r,theta,z' の3値 (got {args.init_pose!r})")
    init_pose = {"r": init_r, "theta": init_theta, "z": init_z}

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", args.listen_port))
    # フィードバックを指令の受信と独立に送るため、受信はブロックしない
    sock.settimeout(0.002)
    print(f"[mock_mcu] listening on :{args.listen_port} "
          f"(drop_rate={args.drop_rate}, lag={args.lag}s, "
          f"feedback={args.feedback_rate}Hz)", file=sys.stderr)

    # 現在の「実位置」。--lag が 0 なら毎回コマンド値に即座に一致させる。
    # 起動直後は自前の初期位置に居る (実機の電源投入時の原点出し後に相当)
    current = {"r": init_r, "theta": init_theta, "z": init_z, "pitch": 0.0, "yaw": 0.0}
    started_at = time.monotonic()
    init_requested = False   # 直近の指令の control_flags bit2
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
                            flags = cmd["control_flags"]
                            enabled = bool(flags & CONTROL_ENABLE)
                            init_requested = bool(flags & CONTROL_INIT_REQUEST)
                            # グリッパ・手首は bit0/bit2 に関係なく反映する
                            gripper_closed = bool(cmd["gripper"])
                            for key in ("pitch", "yaw"):
                                current[key] += (cmd[key] - current[key]) * alpha
                            if not enabled:
                                # 動作許可 0: 現在位置ホールド (ウォッチドッグ発動時と同じ)。
                                # 追従誤差の判定対象にもしない
                                last_cmd_pos = None
                            elif init_requested:
                                # 初期位置要求: r/θ/z を無視して自前の初期位置へ
                                last_cmd_pos = None
                                for key in ("r", "theta", "z"):
                                    current[key] += (init_pose[key] - current[key]) * alpha
                            else:
                                last_cmd_pos = (cmd["r"], cmd["theta"], cmd["z"])
                                for key in ("r", "theta", "z"):
                                    current[key] += (cmd[key] - current[key]) * alpha
                            if not args.quiet:
                                mode = ("hold" if not enabled else
                                        "init" if init_requested else "track")
                                print(f"[mock_mcu] recv seq={seq:6d} {mode:5s} "
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
            if now - started_at < args.init_duration:
                # 起動直後の原点出し中 (契約: 完了まで bit3 を立てたまま送る)
                status_flags |= FLAG_UNINITIALIZED
            if init_requested and all(
                    abs(current[k] - init_pose[k]) < INIT_ARRIVAL_TOLERANCE
                    for k in ("r", "theta", "z")):
                status_flags |= FLAG_AT_INIT_POSE
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

            # 2枚構成の役割に応じて、担当外のフィールドは 0 で送る
            # (契約: 各基板は自分の担当フィールドだけ埋める。hardware_bridge_node が合成)
            fb_r, fb_theta, fb_z = current["r"], current["theta"], current["z"]
            fb_pitch, fb_yaw, fb_gripper = current["pitch"], current["yaw"], gripper_closed
            fb_joints = list(joints)
            if args.board == "arm":
                fb_theta, fb_pitch, fb_yaw, fb_gripper = 0.0, 0.0, 0.0, False
                if len(fb_joints) > TURNTABLE_JOINT_INDEX:
                    fb_joints[TURNTABLE_JOINT_INDEX] = 0.0
            elif args.board == "theta":
                fb_r, fb_z = 0.0, 0.0
                fb_joints = [v if i == TURNTABLE_JOINT_INDEX else 0.0
                             for i, v in enumerate(fb_joints)]

            feedback_seq += 1
            packet = encode_feedback(
                feedback_seq, last_recv_seq if last_recv_seq is not None else 0,
                time.monotonic_ns() // 1000,
                fb_r, fb_theta, fb_z, fb_pitch, fb_yaw,
                status_flags, fb_gripper, fb_joints)
            sock.sendto(packet, client_addr)
    except KeyboardInterrupt:
        print("\n[mock_mcu] stopped", file=sys.stderr)


if __name__ == "__main__":
    main()
