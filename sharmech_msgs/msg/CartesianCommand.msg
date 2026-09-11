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
                             # motion_generator_node が実姿勢に同期し終えるまで false
bool init_request            # 初期位置要求 (bit2)。true の間 MCU は pose/twist を無視して
                             # 自前の初期関節角へ移動する (/catchrobo/game/reset の実体)
