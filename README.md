# sharmech

5節リンク機構を用いたピックアンドプレースロボットの ROS2 パッケージ群。

## 環境

- ROS2 Humble / Ubuntu 22.04

## パッケージ構成

```
sharmech/
├── sharmech_bringup/      # launchファイル・パラメータ設定
├── sharmech_description/  # URDF・メッシュ・RViz設定
└── sharmech_core/         # ノード実装（Component）
```

## セットアップ

```bash
cd ~/catchrobo_ros2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## 起動

```bash
ros2 launch sharmech_bringup sharmech.launch.xml
```

RViz付き:

```bash
ros2 launch sharmech_bringup sharmech.launch.xml rviz:=true
```

## ノード構成

| ノード | 役割 |
|--------|------|
| `kinematics_node` | 5節リンクIK/FK（目標位置 → 関節角度） |
| `arm_controller_node` | モータドライバ通信（関節角度指令・エンコーダ読み取り） |
| `gripper_node` | エンドエフェクタ制御（把持・開放） |
| `task_manager_node` | ピックアンドプレース状態機械 |

全ノードは単一の `component_container` プロセス内で動作する。

## タスク実行

```bash
# ピックアンドプレース開始
ros2 service call /task/start std_srvs/srv/Trigger

# タスク中断
ros2 service call /task/abort std_srvs/srv/Trigger

# グリッパ手動操作（true: 把持 / false: 開放）
ros2 service call /gripper/command std_srvs/srv/SetBool "{data: true}"
```

## パラメータ

`sharmech_bringup/config/config.yaml` で以下を設定する。

| パラメータ | 説明 |
|-----------|------|
| `l1`, `l2` | 5節リンクのリンク長 [m] |
| `base_width` | 2モータ間の距離 [m] |
| `pick_x/y`, `place_x/y` | ピック・プレース座標 [m] |
