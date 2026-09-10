# motion_generator_node のジョグ速度上限を実行時に上書きする。
#
# game_state_manager_node が「今どういう場面か」に応じて publish する
# (WorkspaceClamp と同じ考え方。状態遷移の節目で1回ずつ飛ばすだけで、
# 上書き中に送り続ける必要は無い)。
#
# 用途: 微調整 (ADJUSTING_PICK / ADJUSTING_PLACE) の間だけジョグを遅くする。
# 掴む/離す直前は缶に一番近く、行き過ぎると破損 (競技で-1点) に直結するため。
#
# reset=true のときは v_max を無視し、motion_generator_node の起動時パラメータ
# jog_v_max に戻す。
#
# **この上書きは「速度の上限」であって、操縦層が送る値そのものではない。**
# 操縦層 (VR / PS4) は今まで通り自分の都合で cmd_twist を送ってよく、
# motion_generator_node が受信直後にこの上限で頭打ちにする。そのため
# 操縦層は ROS2 のゲーム状態を知らなくてよい (状態を知っているのはROS2側だけ、
# という役割分担を保つ)。
#
# 仕様の正本: sharmech_core/docs/motion_generator_node.md
#            sharmech_core/docs/game_state_manager_node.md

bool reset

# ジョグを完全に無効化する。自動シーケンスが動作中で操縦者が触ってはいけない場面
# (APPROACHING〜RETRACTING、INIT) に立てる。
#
# **ゴール実行中は goal_mode=goal_priority が既に Twist を無視するが、それだけでは
# 足りない。** GRASPING / ORIENTING はゴールを持たない待機状態 (グリッパを閉じる/
# 缶を縦にする時間を待つだけ) なので、ゴールによる保護が効かず、ジョグがそのまま
# 通ってしまう。ここを塞ぐのが block の役割。
#
# reset=true のときは block も解除される (reset が優先)。
bool block

# 並進ジョグの速度上限 [m/s]。0 以下は不正として無視される
# (「動かせない」を無言で作らないため。止めたいときは block を使う)。
# 姿勢 (pitch/yaw) は対象外
float64 v_max
