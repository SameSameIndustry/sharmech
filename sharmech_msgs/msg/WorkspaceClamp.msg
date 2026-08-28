# motion_generator_node の作業領域クランプを実行時に上書きする。
# game_state_manager_node が PLACING/RETRACTING 中、シューティングボックスの
# スロット周辺だけに一時的に作業領域を絞るために publish する (latched ではない。
# 上書き中は継続的に送り続ける想定は無く、状態遷移の節目で1回ずつ飛ばす)。
#
# reset=true のときは他のフィールドを無視し、config.yaml 起動時の
# workspace_x/y/z_min/max (デフォルト値) に戻す。
#
# 仕様の正本: sharmech_core/docs/motion_generator_node.md
#            sharmech_core/docs/game_state_manager_node.md

bool reset

float64 x_min
float64 x_max
float64 y_min
float64 y_max
float64 z_min
float64 z_max
