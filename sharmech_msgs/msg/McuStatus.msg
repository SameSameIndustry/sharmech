# hardware_bridge_node が MCU から受け取ったフィードバックの、姿勢以外の部分。
#
# これまで status_flags は RCLCPP_WARN_THROTTLE でログに出すだけで、トピックには
# 出ていなかった。そのため可視化クライアント (Three.jsシミュレータ・VR・
# 観測用ダッシュボード) からは MCU の異常もフィードバックの途絶も分からず、
# 「動かないがROS2側は正常に見える」状態の切り分けができなかった。
#
# 仕様の正本: sharmech_core/include/sharmech_core/utility/udp_protocol.hpp
#            sharmech_core/docs/hardware_bridge_node.md

# udp_protocol::kStatus* と同じビット割り当て
uint16 FLAG_TRACKING_ERROR=1   # 追従誤差過大
uint16 FLAG_DRIVER_FAULT=2     # ドライバ異常
uint16 FLAG_WATCHDOG=4         # ウォッチドッグ作動中
uint16 FLAG_UNINITIALIZED=8    # 未初期化・原点未確定
uint16 FLAG_COMMAND_REJECTED=16 # 直近の指令を破棄した (可動域外・seq逆転等)
uint16 FLAG_AT_INIT_POSE=32     # 予約。初期位置要求 (bit2) への到達通知だったが、
                                # 2026-09-11 に初期位置が ROS2 側へ移り ROS2 は bit2 を
                                # 送らないため実機では立たない。互換のため定義のみ残す

std_msgs/Header header

bool     connected          # feedback_timeout 以内にフィードバックが届いているか
uint16   status_flags       # FLAG_*。0 なら異常なし
bool     gripper_closed     # MCU が報告するグリッパの実状態
uint32   seq                # MCU が付けたフィードバックの連番
uint32   seq_echo           # MCU が最後に受信した指令の seq
float64  silence_sec        # 最後にフィードバックが届いてからの経過時間 [s]
uint32   out_of_order_count # 連番が逆転した回数 (UDPの順序入れ替え・喪失の指標)
