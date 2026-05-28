# sharmech

5節リンク機構を用いたピックアンドプレースロボットの ROS2 パッケージ群。
Unity + Meta Quest 3 による VR コントローラで操作する。

## 環境

- ROS2 Humble / Ubuntu 22.04
- Unity + Meta Quest 3 (ROS2-For-Unity)

## アーキテクチャ

```mermaid
flowchart TB
    Unity["Unity + Meta Quest 3<br/>(ROS2-For-Unity)"]
    MCU["MCU<br/>(シリアル / CAN)"]

    subgraph container ["sharmech_container (component_container)"]
        VR["vr_interface_node<br/>──────────────<br/>座標系変換<br/>入力バリデーション"]
        SM["state_manager_node<br/>──────────────<br/>ステートマシン<br/>物理状態管理 · FK計算"]
        TG["trajectory_generator_node<br/>──────────────<br/>カルテシアン軌道生成<br/>速度プロファイル付与"]
        KIN["kinematics_node<br/>──────────────<br/>5節リンク IK<br/>関節角度列生成"]
        HB["hardware_bridge_node<br/>──────────────<br/>制御レート送信<br/>エンコーダ読み取り"]
    end

    subgraph util ["utility/ (Static classes)"]
        direction LR
        CC["coordinate_converter.hpp"]
        FK["five_bar_kinematics.hpp"]
        TU["trajectory_utils.hpp"]
    end

    Unity   <-->|"/vr/target_pose<br/>/vr/current_pose"| VR
    VR       -->|"/target_pose"|                         SM
    SM       -->|"/robot/current_pose"|                  VR
    SM       -->|"/goal_pose"|                           TG
    TG       -->|"/cartesian_trajectory"|                KIN
    KIN      -->|"/joint_trajectory"|                    HB
    KIN      -->|"/joint_trajectory"|                    SM
    HB       -->|"/joint_states"|                        SM
    HB      <-->|"シリアル / CAN"|                       MCU

    VR  -.->|uses| CC
    SM  -.->|uses| FK
    TG  -.->|uses| TU
    KIN -.->|uses| FK
```

## パッケージ構成

```
sharmech/
├── sharmech_bringup/        # launchファイル・パラメータ設定
├── sharmech_description/    # URDF・メッシュ・RViz設定
└── sharmech_core/           # ノード実装 (Component)
    ├── include/sharmech_core/
    │   ├── utility/
    │   │   ├── coordinate_converter.hpp   # Unity↔ROS2 座標変換
    │   │   ├── five_bar_kinematics.hpp    # IK/FK 計算
    │   │   └── trajectory_utils.hpp       # 補間・速度プロファイル
    │   ├── vr_interface_node.hpp
    │   ├── state_manager_node.hpp
    │   ├── trajectory_generator_node.hpp
    │   ├── kinematics_node.hpp
    │   └── hardware_bridge_node.hpp
    └── src/  (各ノードの実装)
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
# 通常起動
ros2 launch sharmech_bringup sharmech.launch.xml

# RViz 付き
ros2 launch sharmech_bringup sharmech.launch.xml rviz:=true
```

## ノード一覧

| ノード | 役割 |
|--------|------|
| `vr_interface_node` | Unity↔ROS2 座標変換・入力バリデーション |
| `state_manager_node` | ステートマシン (IDLE/PLANNING/EXECUTING/DONE)・FK・物理状態管理 |
| `trajectory_generator_node` | カルテシアン空間での軌道生成・速度プロファイル付与 |
| `kinematics_node` | 5節リンク IK・関節角度の時系列生成 |
| `hardware_bridge_node` | 制御レート (100Hz) でMCUへ角度指令送信・エンコーダ読み取り |

## トピック一覧

| トピック | 型 | 説明 |
|---------|-----|------|
| `/vr/target_pose` | `geometry_msgs/PoseStamped` | Unity → VR IF (Unity座標系) |
| `/vr/current_pose` | `geometry_msgs/PoseStamped` | VR IF → Unity (Unity座標系) |
| `/target_pose` | `geometry_msgs/PoseStamped` | VR IF → State Manager (ROS2座標系) |
| `/robot/current_pose` | `geometry_msgs/PoseStamped` | State Manager → VR IF (ROS2座標系) |
| `/goal_pose` | `geometry_msgs/PoseStamped` | State Manager → Traj Generator |
| `/cartesian_trajectory` | `nav_msgs/Path` | Traj Generator → Kinematics |
| `/joint_trajectory` | `trajectory_msgs/JointTrajectory` | Kinematics → HW Bridge / State Manager |
| `/joint_states` | `sensor_msgs/JointState` | HW Bridge → State Manager |
| `/gripper/command` | `std_msgs/Bool` | State Manager → HW Bridge |

## サービス

| サービス | 型 | 説明 |
|---------|-----|------|
| `/task/cancel` | `std_srvs/Trigger` | 実行中タスクを中断しIDLEへ戻す |

## パラメータ

`sharmech_bringup/config/config.yaml` で設定。

| パラメータ | デフォルト | 説明 |
|-----------|----------|------|
| `l1` | 0.15 | 第1リンク長 [m] |
| `l2` | 0.15 | 第2リンク長 [m] |
| `base_width` | 0.10 | 2モータ間距離 [m] |
| `v_max` | 0.10 | 最大速度 [m/s] |
| `a_max` | 0.20 | 最大加速度 [m/s²] |
| `control_rate` | 100.0 | 制御周期 [Hz] |
