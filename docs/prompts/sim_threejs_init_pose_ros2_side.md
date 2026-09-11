# catchrobo_sim_threejs 向け修正プロンプト: 初期位置の ROS2 側移管 (2026-09-11)

ROS2 側 (`~/catchrobo_ros2_ws/src/sharmech`) で **初期位置の正本を MCU 側から ROS2 側へ戻した**。
シミュレータは MCU の代役なので、これに追従する修正が要る。以下をそのまま
`~/catchrobo_sim_threejs` で動かす Claude Code に渡せる。

---

`~/catchrobo_sim_threejs` を修正してください。ROS2 側 (sharmech) が 2026-09-11 に
初期位置の扱いを変えました。契約の正本は
`~/catchrobo_ros2_ws/src/sharmech/docs/mcu_spec.md` §3.2 / §3.5 / §4.6 と
`~/catchrobo_ros2_ws/src/sharmech/sharmech_core/docs/game_state_manager_node.md`
「起動時と INIT」です。まずそこを読んでください。

**変わったこと (ROS2 側):**

1. `sharmech_msgs/CartesianCommand` から `init_request` フィールドが**無くなった**
   (`header` / `pose` / `twist` / `enable` のみ)。UDP `control_flags` bit2 と
   `status_flags` bit5 (`FLAG_AT_INIT_POSE`) は予約になり、ROS2 は bit2 を送らず bit5 を見ない
2. 初期位置へ動かすのは ROS2 側 (`game_state_manager_node`) の仕事になった。起動時と
   `/catchrobo/game/reset` で `INIT` 状態に入り、`CartesianCommand.enable` が false → true に
   なった (= 実姿勢へ同期できた) 時点から、**普通の `target_pose` ゴール** (L 字 3 本) で
   `robot_geometry.yaml` の `init_pose` へ動く。シム側は「電源投入位置に居てホールドしている」
   だけでよい
3. `sharmech_msgs/MotionStatus` の `MODE_INIT=3` は廃止 (INIT 中も mode は `GOAL`/`IDLE`)

**やってほしいこと:**

- `src/sim/ActuatorModel.js`: `init_request` (bit2) の分岐と `FLAG_AT_INIT_POSE` (bit5) を立てる
  処理を削除する。`enable=false` のホールドと起動直後の `initDurationS` の間の bit3 は**そのまま**
- `src/config.js` の `actuator.initPosePolar` は残してよいが、意味を「電源投入直後に居る実位置
  (ROS2 の初期位置とは別。ROS2 が同期後にそこから `init_pose` へ動かす)」に改め、コメントを直す。
  名前を `powerOnPosePolar` に変えるなら参照箇所も全部変える
- `src/main.js` の `actuator.onCommand(msg.pose, {enable: msg.enable, initRequest: msg.init_request})`
  から `initRequest` を外す
- `src/ros/topicRegistry.js` / `src/ui/ControlPanel.js` の `init_request` 表示・`'init'` モードの表示・
  `MotionStatus` mode 3 の表示を削除する。`GAME_STATES` の `INIT` は**残す** (状態名は今も使う。
  起動直後に `INIT` が表示されるのが正しい)
- `src/generated/robotParams.js` は ROS2 側の `generate.py` が既に書き換えている (git diff で
  `init_pose` 関連の STATUS 行が増えているはず)。**編集せず**そのままコミットに含める
- 2026-09-11 の別の変更も未追従なら同時に: MCU 代役のウォッチドッグ 50ms → **500ms**、
  `mcu_status` / `current_pose` の publish 周期 → **10Hz** (ROS2 側の `control_rate` が 10Hz になった)

**確認:**

```bash
# ROS2 側 (ROS_DOMAIN_ID=16、hardware_bridge:=false でシムを MCU 代役にする)
ros2 launch sharmech_bringup sharmech.launch.xml field_color:=red hardware_bridge:=false joy:=false
ros2 launch sharmech_bringup rosbridge.launch.xml
ros2 topic echo /catchrobo/game/state   # INIT → (シムが同期・enable=true) → 3 本のゴールで移動 → WAITING_FOR_PICK
```

シムのアームが起動直後に `initPosePolar` (電源投入位置) から ROS2 の `init_pose`
(既定 r=0.30, θ=0.0, z=0.15 → base (0.30, 0.0, 0.15)) へ「上げる → 水平 → 下ろす」で動き、
その後 `WAITING_FOR_PICK` で静止すれば OK。メニューのステートリセットでも同じ動きになること。

README / コメントに `init_request` `bit2` `bit5` `MODE_INIT` が残っていないか grep して直してください。
`git commit` はしないでください (私が確認してからコミットします)。
