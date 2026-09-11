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

```bash
# 端末 A: ROS2 本体 (motion_generator / hardware_bridge / game_state_manager / joy)
#   field_color は必須。PS4 を使わないなら joy:=false
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

```bash
ros2 topic echo --once /catchrobo/arm/mcu_status    # connected: true / status_flags: 0 になるまで待つ
                                                    #   8 (bit3) = MCU が原点出し中。0 になるまで動かない
ros2 topic echo --once /catchrobo/arm/current_pose  # 両基板が返して初めて出る
ros2 topic echo --once /catchrobo/game/state        # WAITING_FOR_PICK
ros2 topic hz /catchrobo/field/cylinders            # カメラが検出を流しているか
ros2 node list
```

## 6. 試合中の操作 (VR / PS4 の代わりに端末から出す場合)

```bash
# 初期位置へ戻す (MCU 側の初期関節角。ゲーム状態も INIT → WAITING_FOR_PICK)
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

# ターンテーブル軸のベース原点からのずれ (アームを止めてから変えること)
ros2 param set /hardware_bridge_node turntable_axis_x_m 0.0
ros2 param set /hardware_bridge_node turntable_axis_y_m 0.0
```

`ros2 param set` は起動中の値だけ変わる。**同じ値を `robot_geometry.yaml` に書いて `generate.py`
を回さないと次回起動で元に戻る** (`docs/parameter_tuning.md`)。

## 8. 監視 (別端末で流しておくと切り分けが早い)

```bash
ros2 topic echo /catchrobo/arm/mcu_status      # status_flags: 1=追従誤差 2=ドライバ異常 4=ウォッチドッグ
                                               #   8=未初期化 16=指令破棄 32=初期位置到達 (ビットの OR)
ros2 topic echo /catchrobo/arm/status          # mode / message ('executing' / 'reached' / 却下理由)
ros2 topic echo /catchrobo/game/state
```

`connected: false` は片方の基板でも 0.5 秒途絶したことを示す。`hardware_bridge_node` のログに
どちらの基板かが出る。

## 9. 終了

各端末で `Ctrl+C`。ROS2 を止めても MCU は最後の目標位置をホールドする (脱力しない)。
動力を切るのはハードウェアの非常停止 / 電源スイッチ (ソフトからは切らない)。
