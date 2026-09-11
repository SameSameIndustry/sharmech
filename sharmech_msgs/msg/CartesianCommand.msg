# エンドエフェクタの目標位置と目標速度のセット。
# motion_generator_node が control_rate で publish するストリームの1サンプル。
#
# 位置と速度を同じメッセージで運ぶのは、両者が同じ UDP パケットに載るため。
# 別トピックにすると「サイクル N の位置」と「サイクル N-1 の速度」が
# 組み合わさる危険がある。
#
# time_from_start に相当するフィールドは持たない。ストリーミング方式のため
# 「届いた瞬間がその点の時刻」であり、時刻は header.stamp が担う。

std_msgs/Header header       # stamp = この指令の時刻
geometry_msgs/Pose pose      # 目標位置 + 姿勢(クォータニオン)
geometry_msgs/Twist twist    # 目標速度(並進 + 角速度)。ベース座標系基準

# UDP control_flags に載る「状態 (レベル)」。位置と同じメッセージで運ぶのは、
# 別トピックにすると DDS の発見遅れで「位置は届くがフラグはまだ既定値」という
# 数百 ms の窓が開き、起動直後に enable=1 のまま原点の仮目標へ動き出したため
# (2026-09-10 mock_mcu で実測)。
bool enable                  # 動作許可 (bit0)。false なら MCU は現在位置をホールド。
                             # motion_generator_node が実姿勢に同期し終えるまで false。
                             # **この false → true が game_state_manager_node への
                             # 「動かしてよい」の合図でもある** (起動時と
                             # /catchrobo/game/reset の INIT がこれを待って動き出す)
# init_request (bit2) は 2026-09-11 に廃止。初期位置は ROS2 側
# (game_state_manager_node の init_pose_*) が持ち、普通のゴールとして出す
