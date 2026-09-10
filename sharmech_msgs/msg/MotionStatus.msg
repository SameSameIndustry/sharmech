# motion_generator_node の現在状態。latched (transient_local) で配信され、
# 後から接続したクライアント(VR の再接続を含む)にも現在状態が即座に届く。
#
# 「実行中」はイベントではなく状態。Action を使わない設計の中核
# (詳細は sharmech_core/docs/motion_generator_node.md の「Action を使わない」)。

uint8 MODE_IDLE=0
uint8 MODE_GOAL=1            # ゴールへの軌道を実行中
uint8 MODE_JOG=2             # ジョグ(速度指令)で動作中
uint8 MODE_INIT=3            # 初期位置要求中 (MCU が自前の初期関節角へ移動中。到達で IDLE + SUCCEEDED)

uint8 RESULT_NONE=0
uint8 RESULT_SUCCEEDED=1
uint8 RESULT_ABORTED=2       # cancel された、またはジョグに割り込まれた
uint8 RESULT_REJECTED=3      # 作業領域外・ジョグ動作中などで受理されなかった

std_msgs/Header header
uint8 mode                   # MODE_*
geometry_msgs/Pose goal_pose # mode==MODE_GOAL のときの目標
float64 distance_remaining   # 目標までの残距離 [m]
float64 time_remaining       # 到達までの残時間 [s]
uint8 last_result            # RESULT_*。直近のゴールの結果
string message               # 却下理由など人間向けの説明
