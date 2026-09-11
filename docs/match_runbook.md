# 試合当日の起動手順 (実機・コマンド一覧)

2026-09-11 作成。実機 (MCU 2 枚) で試合をするときに叩くコマンドを、順番どおりに並べたもの。
各コマンドの背景は `CLAUDE.md`・`docs/parameter_tuning.md`・`docs/mcu_spec.md` を参照。

前提: ROS2 側 PC の IP は **192.168.1.2** (MCU のフィードバック宛先として固定)。
MCU は r/z 基板 192.168.1.100 と θ 基板 192.168.1.101 に `mcu_spec.md` §10 のファームが
書き込まれていること。

## 0. 端末ごとに必ず最初に

```bash
source /opt/ros/humble/setup.bash
cd ~/catchrobo_ros2_ws
source install/setup.bash
```

## 1. 実測値の反映 (会場で寸法を測ったら。測っていなければ飛ばす)

```bash
# 正本を編集 (status を measured に、value を実測値に)
vim src/sharmech/params/robot_geometry.yaml

python3 src/sharmech/params/generate.py            # 未実測の一覧が出る
python3 src/sharmech/params/generate.py --check    # 3 ファイルとも "up to date" になること
```

MCU 側のリンク寸法 (`user_config.h`) も同じ値を人間が入れる (`mcu_spec.md` §10.2)。

## 2. ビルド (コードを変えたときだけ。launch / config / yaml の変更は不要)

```bash
colcon build --symlink-install --packages-select sharmech_msgs sharmech_core sharmech_bringup sharmech_description
source install/setup.bash
```

## 2.5 PS5 コントローラの LED (初回のみ。入れてあれば飛ばす)

`MANUAL_CONTROL` の間 DualSense のライトバーとプレイヤー LED を白で点滅させる
(`docs/joy_teleop_node.md`「MANUAL_CONTROL の LED 表示」)。sysfs への書き込み権限が要るので
1 回だけ udev ルールを入れる:

```bash
sudo cp ~/catchrobo_ros2_ws/src/sharmech/sharmech_bringup/udev/90-dualsense-leds.rules /etc/udev/rules.d/
sudo udevadm control --reload
sudo udevadm trigger --action=add --subsystem-match=leds
ls -l /sys/class/leds/input*:rgb:indicator/brightness   # -rw-rw-rw- なら OK (コントローラ接続中に)
```

## 3. ネットワーク確認

構成: PC (USB-Ethernet アダプタ) ↔ スイッチングハブ ↔ r/z 基板 (.100) / θ 基板 (.101)。
PC 側の 192.168.1.2 は NetworkManager プロファイル `mcu-wired` (2026-09-11 作成) が
リンク検出時に自動で付ける。

```bash
nmcli device status | grep enx        # 有線アダプタが connected / mcu-wired になっていること
ip addr show | grep 192.168.1.        # 自分が 192.168.1.2 であること
ping -c 3 192.168.1.100               # r/z 基板
ping -c 3 192.168.1.101               # θ 基板

# ROS2 を起動する前に、基板が 0x81 を 10Hz で自発送信しているか (受信だけ、何も送らない)
python3 src/sharmech/sharmech_core/scripts/probe_mcu_feedback.py
#   → .100 と .101 の両方から約 10 Hz、status_flags に bit3 (未初期化) が無ければ OK
```

うまくいかないとき:

| 症状 | 見るところ |
|---|---|
| `nmcli device status` で `unavailable` (NO-CARRIER) | **PC↔ハブ間のケーブルとハブの電源** (基板の電源は関係ない。アダプタは Realtek / ASIX どちらも正常と確認済み) |
| リンクはあるが IP が 192.168.1.2 でない (DHCP の「Wired connection N」を掴んだ) | `nmcli connection up mcu-wired`。プロファイルが無ければ下で再作成 |
| ping が通らない | 基板の電源、ハブの基板側ポート LED、`ip neigh` に `00:08:dc:…` (W5500) が出るか |
| ping は通るが 0x81 が来ない | 基板ファームの `dest_ip` が 192.168.1.2 か (`mcu_spec.md` §10)。`hardware_bridge_node` 起動中は 8889 が塞がるので `ros2 topic echo /catchrobo/arm/mcu_status` で見る |

`mcu-wired` の再作成 (別 PC・再インストール時。sudo 不要):

```bash
nmcli connection add type ethernet con-name mcu-wired \
  ipv4.method manual ipv4.addresses 192.168.1.2/24 ipv4.never-default yes \
  ipv6.method disabled connection.autoconnect yes connection.autoconnect-priority 100 \
  match.interface-name "enxc84d44203420 enxf8e43b02ab70"   # 手持ちの USB-Ethernet アダプタ名。ip -br link で確認
```

## 4. 起動 (端末を 3 つ)

> ⚠ **端末 A を上げると、MCU と同期した瞬間にアームが初期位置へ動く** (2026-09-11〜。
> `game_state_manager_node` が `INIT` から始まり、`robot_geometry.yaml` の `init_pose` へ
> **直線 1 本**で行く。L 字ではないので斜めに動く)。**電源を入れる前に人間がアームを
> おおよその初期位置に置き、今の位置〜初期位置の間を空けること。** 動かしたくないときは
> `config.yaml` の `game_state_manager_node.init_on_startup: false` (起動時にだけ読む)。

```bash
# 端末 A: ROS2 本体 (motion_generator / hardware_bridge / game_state_manager / joy)
#   field_color は必須。PS4 を使わないなら joy:=false
#   ★ 上げた直後に初期位置へ動く (上の注意)
ros2 launch sharmech_bringup sharmech.launch.xml field_color:=red
ros2 launch sharmech_bringup sharmech.launch.xml field_color:=blue

# 端末 B: rosbridge (VR クライアント用。port 9090)
ros2 launch sharmech_bringup rosbridge.launch.xml

# 端末 C: カメラ (パターンは catchrobo_perception/README.md の対応表で選ぶ。下は USB カメラ・色・ホモグラフィ)
ros2 launch catchrobo_perception cylinder_detector.launch.xml \
    camera_type:=usb detector_mode:=color mapper_mode:=homography \
    homography_path:=$HOME/catchrobo_ros2_ws/calib/homography.yaml \
    intrinsics_path:=$HOME/catchrobo_ros2_ws/calib/intrinsics.yaml
```

VR 側 PC (別マシン):

```bash
cd ~/catchrobo_webxr_controller
npm run play -- --ros=ws://192.168.1.2:9090     # 表示された URL を Quest のブラウザで開く
```

## 5. 起動確認 (動かす前に必ず)

**起動直後の流れ (2026-09-11〜):** `game/state` は `INIT` で始まる → MCU の 0x81 が届き
bit3 (未初期化) が落ちると `motion_generator_node` が実姿勢へ同期して動作許可を出す →
**その瞬間にアームが初期位置へ動き出す** (直線 1 本。`INIT` のまま) → 着いたら `WAITING_FOR_PICK`。
**試合中に 0x81 が 500ms 途絶する (ケーブル抜け・MCU 再起動) と、どの状態からでも `INIT` に
戻り、復帰した瞬間に初期位置へ動く** (グリッパは開く)。
**⚠ 「動かす前に」といっても、この初期位置への移動だけは launch した時点で勝手に始まる。**
§4 の注意どおり、起動前にアームの周囲を空けておくこと (同期のタイミングは MCU 次第で、
MCU が既に原点出し済みなら launch の数秒後に動く)。
`INIT` のまま動かないなら「まだ同期していない」(MCU が 0x81 を返していない / bit3 が
立ったまま / 片方の基板しか返していない) なので、下の `mcu_status` を見る。

```bash
ros2 topic echo --once /catchrobo/arm/mcu_status    # connected: true / status_flags: 0 になるまで待つ
                                                    #   8 (bit3) = MCU が原点出し中。0 になるまで動かない
ros2 topic echo --once /catchrobo/arm/current_pose  # 両基板が返して初めて出る
ros2 topic echo /catchrobo/game/state               # INIT (初期位置へ移動中) → WAITING_FOR_PICK になるまで待つ
ros2 topic hz /catchrobo/field/cylinders            # カメラが検出を流しているか
ros2 node list
```

端末 A のログにも出る: 起動時に `INIT on startup: the arm will move to the init pose as
soon as motion is enabled` (WARN)、同期した瞬間に `INIT: motion enabled; moving to init pose
(x, y, z) via L-shaped goals`、その後 `Goal accepted:` が区間ごとに最大 3 回。
初期位置の座標は `init pose (red): r=… theta=… z=… -> base (…)` の行で確認できる
(値の決め方は `parameter_tuning.md` §4.5)。

## 6. 試合中の操作 (VR / PS4 の代わりに端末から出す場合)

```bash
# 初期位置へ戻す (robot_geometry.yaml の init_pose。直線 1 本で戻り、
#   ゲーム状態も INIT → WAITING_FOR_PICK。どの状態からでも可。掴んでいたワークは離す。
#   箱の中など周囲に当たりそうな位置からは、先に MANUAL_CONTROL のジョグで抜いてから)
ros2 topic pub --once /catchrobo/game/reset std_msgs/msg/Empty '{}'

# 微調整 (ADJUSTING_PICK / ADJUSTING_PLACE) の確定
ros2 topic pub --once /catchrobo/game/confirm std_msgs/msg/Empty '{}'

# 自動シーケンスを止めて自由操作へ (もう一度で復帰)
ros2 topic pub --once /catchrobo/game/toggle_manual_control std_msgs/msg/Empty '{}'

# 実行中のゴールを中止 (その場で停止)
ros2 topic pub --once /catchrobo/arm/cancel std_msgs/msg/Empty '{}'

# 手動でゴールを与える (field 座標 [m])
ros2 topic pub --once /catchrobo/arm/target_pose geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: field}, pose: {position: {x: 0.40, y: 0.10, z: 0.10}, orientation: {w: 1.0}}}'

# グリッパ (true = 閉)
ros2 topic pub --once /catchrobo/arm/gripper std_msgs/msg/Bool '{data: true}'

# 自動シーケンスを手で起動: 掴みに行く (缶の姿勢を渡す) / 置きに行く (指定箱に離した通算個数)
ros2 topic pub --once /catchrobo/game/pick_request geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: field}, pose: {position: {x: 0.50, y: 0.20, z: 0.05}, orientation: {w: 1.0}}}'
ros2 topic pub --once /catchrobo/game/box_count std_msgs/msg/Int32 '{data: 1}'
```

PS4 の既定ボタン: × = グリッパ開閉、△ = ホーム、R3 = 確定、L1 = デッドマン (押しながらスティック)、
L1+R1+L3+R3 同時 = 自由操作トグル (`docs/joy_teleop_node.md`)。
**PS5 だけで操作するときは最初に自由操作トグルで `MANUAL_CONTROL` に入る**
(`pick_request` / `box_count` / `reset` は VR にしか無く、PS5 単独では自動シーケンスを
始められない。`WAITING_FOR_PICK` のままだと VR の `pick_request` でジョグが遮断される)。
入っている間は **コントローラのライトバーとプレイヤー LED が白で点滅**する (2.5 の udev ルールが前提)。

## 7. 当日の調整 (再起動なしで即反映。正本にも同じ値を書くこと)

```bash
# 箱の上面高さ (スロットの高さはすべてここからの相対)
ros2 param set /game_state_manager_node box_top_z_m 0.156
ros2 param get /game_state_manager_node box_top_z_m

# 缶の隙間
ros2 param set /game_state_manager_node slot_gap_y_m 0.014

# ターンテーブル軸のベース原点からのずれ (アームを止めてから変えること)。
#   game_state_manager_node も init_pose の極座標の原点として同じ値を持つので両方へ
ros2 param set /hardware_bridge_node turntable_axis_x_m 0.0
ros2 param set /hardware_bridge_node turntable_axis_y_m 0.0
ros2 param set /game_state_manager_node turntable_axis_x_m 0.0
ros2 param set /game_state_manager_node turntable_axis_y_m 0.0

# 初期位置 (極座標 r [m] / θ [rad、時計回り正] / z [m]。次の reset から効く。決め方は parameter_tuning.md §4.5)
ros2 param set /game_state_manager_node init_pose_r_red 0.30
ros2 param set /game_state_manager_node init_pose_theta_red 0.0
ros2 param set /game_state_manager_node init_pose_z_red 0.15
ros2 topic pub --once /catchrobo/game/reset std_msgs/msg/Empty '{}'    # 新しい値で戻してみる
```

`ros2 param set` は起動中の値だけ変わる。**同じ値を `robot_geometry.yaml` に書いて `generate.py`
を回さないと次回起動で元に戻る** (`docs/parameter_tuning.md`)。

## 8. 監視 (別端末で流しておくと切り分けが早い)

```bash
ros2 topic echo /catchrobo/arm/mcu_status      # status_flags: 1=追従誤差 2=ドライバ異常 4=ウォッチドッグ
                                               #   8=未初期化 16=指令破棄 (ビットの OR)。32 (bit5) は予約で ROS2 は見ない
ros2 topic echo /catchrobo/arm/status          # mode / message ('executing' / 'reached' / 却下理由)
ros2 topic echo /catchrobo/game/state
```

`connected: false` は片方の基板でも 0.5 秒途絶したことを示す。`hardware_bridge_node` のログに
どちらの基板かが出る。

RViz で見るなら別端末で:

```bash
ros2 launch sharmech_bringup rviz.launch.xml
```

現在位置 (座標軸)・ストリーム目標 (橙)・ゴール (赤)・ワーク検出 (黄)・VR の共有映像が出る。
共有映像は VR 側で `?spectator=1` かメニュー「表示」で入にしたときだけ流れる (既定は切)。

## 9. 終了

各端末で `Ctrl+C`。ROS2 を止めても MCU は最後の目標位置をホールドする (脱力しない)。
動力を切るのはハードウェアの非常停止 / 電源スイッチ (ソフトからは切らない)。
**ROS2 だけを上げ直すと、その位置から初期位置へまた動く** (§4 の注意。MCU を再起動しない
限り同期はすぐ済むので、`launch` の直後に動き出す)。
