# 実測チェックリスト (機構パラメータ)

**目的: ロボットを最低限動かすための唯一の前提条件を潰す。**

2026-09-05 時点で、ROS2層は完成しビルドも通るが、**機構の幾何パラメータが
全て仮値 (0.0) のまま**であり、これが無いとIKが解けないため実機を動かせない。
`mcu_spec.md` §8 も「これが無いとIKが解けず動かせない。最優先」としている。

このファイルは**アームの現物を前にして順に埋めていくためのシート**である。
埋めた値の入れ先は各項目に明記してある。

> **2026-09-08 以降、入れ先はすべて `sharmech/params/robot_geometry.yaml` (正本) である。**
> `config.yaml` や シミュレータ/VR の設定に直接書かない。値を入れたら
> `python3 sharmech/params/generate.py` で各リポジトリ向けの設定を生成する。
> 手順の全体は [`parameter_tuning.md`](parameter_tuning.md)。下表の「入れ先」は
> `robot_geometry.yaml` 内のセクション名で読むこと。

- 単位は全て **[m] / [rad]** (ROS標準)。ノギスの mm は 1000 で割ること
- 座標系は `field` = ロボットのベース座標系 (原点・軸の向きは
  [`field_dimensions.md`](field_dimensions.md) が正本)
- 幾何モデルの実装は
  `sharmech_core/include/sharmech_core/utility/parallel_arm_kinematics.hpp`

---

## 0. 測る前に確認すること (モデルの前提。ここが崩れると測っても合わない)

現在の実装は以下を**仮定している**。現物を見て違っていたら、測る前に報告すること
(モデル自体の作り直しになるため、測り直しが発生する)。

| # | 仮定 | 確認方法 |
|---|---|---|
| A | **肩の2ピボットの中点が、ターンテーブルの回転軸上にある** | FKは `x = turntable_axis_x + r·cos(θ)` と計算する。`r` は肩2ピボットの中点からの距離。中点と回転軸がずれていると、そのオフセット分だけ実機と合わない |
| B | **xy と z が完全に独立。** 肘/膝が伸縮しても水平距離 `r` は変わらず、肩が動いても高さ `z` は変わらない | 肘/膝だけを動かしてEEが真上に動くか、肩だけを動かしてEEが水平に動くかを目視で確認 |
| C | **肩・肘/膝とも、左右2モータは常に絶対値が同じ角度で対称駆動される** | 機構的に強制されているか (リンクで拘束) / ソフトで揃えているだけか |
| D | **肘/膝機構は肩機構と数式上同型** (2ピボット + 近位リンク + 遠位リンクの対称二軸駆動) | 現物の形が下図と同じ構成か |

---

## 1. 対称二軸駆動リンクの寸法 (肩・肘/膝で共通の測り方)

肩機構と肘/膝機構は**同じ形**なので、同じ3つの値をそれぞれについて測る。

```
        pivot_L(-a, 0) ----- L1 ----- knee_L ----- L2 -----+
              |                                            |
       (2a = ピボット間距離)                              EE (0, d)
              |                                            |
        pivot_R(+a, 0) ----- L1 ----- knee_R ----- L2 -----+

   a  = pivot_half_separation_m : 2つの固定ピボット間距離の「半分」  ★半分であることに注意
   L1 = proximal_link_length_m  : ピボット中心 〜 中間関節(knee)中心
   L2 = distal_link_length_m    : 中間関節(knee)中心 〜 EE合流点中心
   d  = この機構が生み出す出力距離 (肩なら水平距離 r、肘/膝なら高さ方向)
```

**測り方の原則: 全て「回転軸の中心から中心まで」を測る。** リンク部材の
外形長さではない。軸穴の中心間距離。

### 1.1 肩機構 (水平距離 r を与える)

| 記号 | 入れ先 (`robot_geometry.yaml` の `kinematics`) | 実測値 [m] | 備考 |
|---|---|---|---|
| a | `shoulder_pivot_half_separation_m` | ______ | **ピボット間距離を測って÷2** |
| L1 | `shoulder_proximal_link_length_m` | ______ | |
| L2 | `shoulder_distal_link_length_m` | ______ | |

### 1.2 肘/膝機構 (高さ z を与える)

| 記号 | 入れ先 (`robot_geometry.yaml` の `kinematics`) | 実測値 [m] | 備考 |
|---|---|---|---|
| a | `knee_pivot_half_separation_m` | ______ | **÷2を忘れない** |
| L1 | `knee_proximal_link_length_m` | ______ | |
| L2 | `knee_distal_link_length_m` | ______ | |

### 1.3 妥当性の即席チェック (測った直後にその場でやる)

三角不等式 `|L1 - L2| ≤ hypot(a, d) ≤ L1 + L2` を満たす `d` の範囲が、
その機構の可動範囲になる。**この範囲が実機の可動範囲と大きく食い違っていたら
測り間違い** (よくあるのは `a` を半分にし忘れ、L1/L2 を外形長で測る)。

到達可能な最大出力は概ね `d_max ≈ sqrt((L1+L2)² - a²)`。
肩なら「アームを最も伸ばしたときの水平到達距離」と一致するはず。

---

## 2. 取り付け位置 (base座標系での基準)

| パラメータ | 入れ先 | 実測値 | 定義 |
|---|---|---|---|
| `turntable_axis_x_m` | `robot_geometry.yaml` の `kinematics` | ______ | ターンテーブル回転軸の、ロボット設置エリア中心 (=`field`原点) から見たx。**UDP で送る極座標 (r, θ) の原点でもある** (`mcu_spec.md` §5)。値を入れれば `hardware_bridge_node` が送受信の両方で自動的に引く/足す (2026-09-10 実装済み。実行中は `ros2 param set /hardware_bridge_node turntable_axis_x_m <値>` でも反映可) |
| `turntable_axis_y_m` | 同上 | ______ | 同y。設置エリアの左右中央に据えるなら 0.0 |
| `knee_base_height_m` | 同上 | ______ | **肘/膝機構の d=0 が対応する床からの高さ z。** d はこの高さからの相対量として足される |

`knee_base_height_m` は「肘/膝機構の2ピボットを結ぶ線の高さ」に相当する。
d=0 は幾何的には EE がその線上にある状態だが、実機では到達できない姿勢のことが多い。
その場合は**到達できる姿勢で z を実測し、そのときの d を逆算して差を取る**
(例: EEを床から0.35mの位置に置き、そのとき機構が出している d が 0.28m なら
`knee_base_height_m = 0.35 - 0.28 = 0.07`)。

---

## 3. Z方向の作業領域・高さ (現在すべて仮値。実測すると安全側が決まる)

Z は 2026-08-30 のフィールド実測が上面図のみだったため、**X/Yと違って一切
確定していない。** ここが仮値のままだと「Zに動かすと全部クランプで却下される」
「箱に当たる」のどちらかになる。

| パラメータ (`robot_geometry.yaml`) | 生成先 (ROS2 側の名前) | 現在値 | 実測値 | 定義 |
|---|---|---|---|---|
| `workspace.z_min_m` | `motion_generator_node.workspace_z_min` | 0.00 (仮) | ______ | EEが下がってよい下限。床・治具に当たらない高さ |
| `workspace.z_max_m` | `motion_generator_node.workspace_z_max` | 0.30 (仮) | ______ | 上限。機構の可動上限か、上空の禁止区域の低い方 |
| **`shooting_box.top_z_m`** | `game_state_manager_node.box_top_z_m` | 0.156 (CAD) | ______ | **★高さの基準面 (箱の上端)。2026-09-06 以降、Z方向で実測が要るのは実質これ1個。** 下の相対値がすべてこれに追従する |
| `shooting_box.release_below_top_m` | `slot_release_below_box_top_m` | 0.106 (仮) | ______ | 缶を離す高さ。基準面から何m下か。グリッパ形状が決まらないと定まらない |
| `shooting_box.transport_clearance_above_top_m` | `transport_clearance_above_box_top_m` | 0.044 (仮) | ______ | 運搬中に上げる高さ。基準面から何m上か。**缶を縦に持つと下端が更に142mm下がる点に注意** |
| `shooting_box.retract_clearance_above_top_m` | `retract_clearance_above_box_top_m` | 0.044 (仮) | ______ | 設置後に退避する高さ。同上 |
| `shooting_box.approach_clearance_above_top_m` | `approach_clearance_above_box_top_m` | 0.044 (仮) | ______ | 接近時に水平移動する高さ。**retract と同じ値にする** |
| `work_placement.first_z_m` | `field_geometry.first_work_z_m` | 0.0 (仮) | ______ | 治具上のワーク中心の高さ。土台厚2.3mmは確定済みだが、土台が乗る面の絶対Zが未較正 |

**併せて測るもの: シューティングボックスの上端高さ** (箱の縁の床からの高さ)。
`transport_clearance_above_box_top_m` / `retract_clearance_above_box_top_m` は 正の値にしておけば基準面より上になる。

> **2026-09-06 以降、これらは実行中に `ros2 param set` で変えられる。**
> 測ってから再起動する必要はない。ジョグで正しい高さまで動かし、
> `ros2 topic echo /catchrobo/arm/current_pose --once` で z を読み、
> `ros2 param set /game_state_manager_node box_top_z_m <値>` で入れて、そのまま試せる。
> **ただし `ros2 param set` は再起動で消える。** 決まった値は必ず `robot_geometry.yaml` に
> 書いて `generate.py` を走らせること ([`parameter_tuning.md`](parameter_tuning.md) 手順B)。

箱上端高さ = ______ [m]

---

## 4. モータ側の規約 (MCU担当者と突き合わせる項目)

ROS2側の `parallel_arm_kinematics.hpp` は「各ピボットで**もう一方のピボットへ
向かう方向を0**とし、EE側 (+d方向) へ回すほど増える」という角度の向きを仮定している。
**実機のモータのゼロ点・回転方向がこの通りである保証は無い。**

パターンA (通常運用) ではIKはMCU側なのでROS2は関知しないが、
**フィードバック 0x81 の `joint_positions[]` はこの規約で返してもらう必要がある**
(ROS2側で角度を解釈するのはここだけ)。担当者と以下を合わせること。

| 項目 | 決めるのは | 内容 |
|---|---|---|
| 各モータのゼロ点 | MCU側 | 上記の規約に合わせる or オフセットで吸収 |
| 各モータの正回転方向 | MCU側 | 同上。符号が逆なら反転 |
| ギア比・エンコーダカウント→rad | MCU側 | |
| 関節可動域 (joint limits) | MCU側 | `status_flags` bit4 (可動域外破棄) 用。実測後にROS2側の作業領域クランプと整合を取る |
| 原点出し (ホーミング) 手順 | MCU側 | 完了まで `status_flags` bit3 を立てる |

並び順の契約 `[shoulder_left, shoulder_right, turntable, knee_left, knee_right]`
は確定済み (`udp_protocol.hpp` の `kJointOrder` が正本)。

---

## 5. 実測後の検算手順

値を `robot_geometry.yaml` に入れ (`status` を `measured` に、`date` を当日に)、
生成してから、実機を動かす前に必ず往復確認する。

```bash
cd ~/catchrobo_ros2_ws
python3 src/sharmech/params/generate.py          # 生成 (未実測が残っていれば一覧が出る)
python3 src/sharmech/params/generate.py --check  # 生成物が正本と一致していること
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select sharmech_core
colcon test --packages-select sharmech_core   # FK/IK往復テストが通ること
```

続いてパターンBを有効にして、IKが実際に解けるかを見る
(パターンAでもFK/IKの妥当性確認として有用)。

```bash
ros2 launch sharmech_bringup sharmech.launch.xml field_color:=red pattern_b:=true hardware_bridge:=false

# 別端末で、作業領域内の適当な点を指令して関節角が出るか見る
ros2 topic echo /catchrobo/command/joint
ros2 topic pub --once /catchrobo/arm/target_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: field}, pose: {position: {x: 0.3, y: 0.0, z: 0.1}, orientation: {w: 1.0}}}"
```

`/catchrobo/command/joint` に5関節ぶんの角度が出れば、その点はIKが解けている。
**作業領域の四隅と中心・上下限で試し、どこかで解けない (到達不可) 点があれば、
`workspace_*` をその範囲に狭めること** (ROS2側で先に弾いた方が安全)。

---

## 6. 埋めたあとにすること

1. `robot_geometry.yaml` の該当項目の `status` を `unmeasured` → `measured` (または
   `fitted`) に変え、`source` に測り方、`date` に実測日を書く。`generate.py` を走らせ、
   生成物 (ROS2 / sim / webxr の3つ) をそれぞれのリポジトリでコミットする
2. [`../../CLAUDE.md`](../../CLAUDE.md) の「勝手に決めてはいけない未決定事項」から
   該当項目 (4番・5番・6番) を消すか、確定済みに書き換える
3. `mcu_spec.md` §8 の #1 を「実測済み」に更新し、値をMCU担当者へ渡す
   (MCU側もIK/FKに同じ値が要る)
