#!/usr/bin/env python3
"""
実機 MCU が 0x81 フィードバックを自発送信しているかを ROS2 無しで確認する。

UDP 8889 番 (config.yaml の local_port) で数秒間受信し、送信元 IP ごとに
到着レート・ヘッダ・実位置・status_flags を表示する。**こちらからは何も送らない**
(モータは動かない) ので、ROS2 を起動する前の生存確認・ケーブル確認に安全に使える。
パケットレイアウトは sharmech/docs/mcu_spec.md §3.1 / §3.4 / §3.5 が正本。

使い方:
    python3 sharmech_core/scripts/probe_mcu_feedback.py            # 3 秒受信
    python3 sharmech_core/scripts/probe_mcu_feedback.py --seconds 10

hardware_bridge_node が起動中だと 8889 番が塞がっていて bind に失敗する
(その場合は `ros2 topic echo /catchrobo/arm/mcu_status` を使う)。

期待する表示 (2 枚構成、mcu_spec.md §2.2):
  - 192.168.1.100 (r/z 基板) と 192.168.1.101 (θ 基板) の両方から約 10 Hz
  - ver=2 type=0x81 payload_length=48、パケット全体 64 バイト
  - r/z 基板は theta=0、θ 基板は r=z=0 (担当外は 0 で送る契約)
  - status_flags の bit3 (未初期化) が落ちていれば ROS2 起動後すぐ同期できる
"""

import argparse
import socket
import struct
import time

PROTOCOL_VERSION = 2
PACKET_TYPE_FEEDBACK = 0x81
FEEDBACK_LEN = 64
JOINT_NAMES = ["shoulder_left", "shoulder_right", "turntable", "knee_left", "knee_right"]
FLAG_NAMES = {
    0: "追従誤差過大",
    1: "ドライバ異常",
    2: "ウォッチドッグ作動中",
    3: "未初期化",
    4: "指令破棄",
    5: "初期位置到達",
}


def decode(data: bytes) -> dict:
    ver, typ, plen, seq, ts = struct.unpack_from("<BBHIQ", data, 0)
    seq_echo, r, th, z, pitch, yaw, flags, grip, jc = struct.unpack_from("<IfffffHBB", data, 16)
    joints = struct.unpack_from("<5f", data, 44)
    return dict(ver=ver, typ=typ, plen=plen, seq=seq, ts=ts, seq_echo=seq_echo,
                r=r, theta=th, z=z, pitch=pitch, yaw=yaw, flags=flags,
                gripper=grip, joint_count=jc, joints=joints)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", type=int, default=8889, help="受信ポート (config.yaml の local_port)")
    ap.add_argument("--seconds", type=float, default=3.0, help="受信する秒数")
    args = ap.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        sock.bind(("0.0.0.0", args.port))
    except OSError as e:
        print(f"UDP {args.port} 番を bind できない: {e}")
        print("hardware_bridge_node が起動中なら止めるか、ros2 topic echo /catchrobo/arm/mcu_status を使う")
        return 2
    sock.settimeout(0.5)

    per_ip: dict[str, list[bytes]] = {}
    bad: dict[str, int] = {}
    t0 = time.time()
    while time.time() - t0 < args.seconds:
        try:
            data, (ip, _) = sock.recvfrom(2048)
        except socket.timeout:
            continue
        if len(data) != FEEDBACK_LEN or data[0] != PROTOCOL_VERSION or data[1] != PACKET_TYPE_FEEDBACK:
            bad[ip] = bad.get(ip, 0) + 1
            continue
        per_ip.setdefault(ip, []).append(data)

    if not per_ip and not bad:
        print(f"{args.seconds:.0f} 秒間、UDP {args.port} 番に何も届かなかった。")
        print("確認: ping 192.168.1.100 / .101、自分の IP が 192.168.1.2 か (MCU の dest_ip)、"
              "基板の電源、ハブの LED")
        return 1

    for ip in sorted(per_ip):
        pkts = per_ip[ip]
        last = decode(pkts[-1])
        rate = len(pkts) / args.seconds
        seqs = [decode(p)["seq"] for p in pkts]
        lost = (seqs[-1] - seqs[0] + 1) - len(seqs) if len(seqs) > 1 else 0
        bits = [b for b in range(16) if last["flags"] >> b & 1]
        print(f"[{ip}] {len(pkts)} packets / {args.seconds:.0f}s = {rate:.1f} Hz"
              f"  seq {seqs[0]}..{seqs[-1]} (欠落 {lost})  seq_echo={last['seq_echo']}")
        print(f"   ver={last['ver']} type=0x{last['typ']:02x} payload_length={last['plen']} "
              f"joint_count={last['joint_count']}")
        print(f"   r={last['r']:.4f} m  theta={last['theta']:.4f} rad  z={last['z']:.4f} m"
              f"  pitch={last['pitch']:.3f}  yaw={last['yaw']:.3f}  gripper={last['gripper']}")
        flag_desc = ", ".join(f"bit{b}={FLAG_NAMES.get(b, '予約')}" for b in bits) or "なし"
        print(f"   status_flags=0x{last['flags']:04x} ({flag_desc})")
        print("   joints: " + ", ".join(f"{n}={v:.3f}" for n, v in zip(JOINT_NAMES, last["joints"])))
    for ip, n in bad.items():
        print(f"[{ip}] 契約外パケット {n} 個 (長さ≠64 / version≠2 / type≠0x81)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
