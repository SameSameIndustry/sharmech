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
