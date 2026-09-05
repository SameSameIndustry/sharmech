# kinematics_node

パターンB (ROS2側でIK) 用のノード。**実装済み** (`src/kinematics_node.cpp`)。
`hardware_bridge_node` の `command_mode: "joint"` も 2026-09-01 に実装済みで、
このノードの出力は `packet_type = 0x02` としてMCUまで届く経路が繋がった
(mock_mcu でエンドツーエンド疎通確認済み。ただしリンク長は仮値のままなので
実機で動かせる状態ではない)。詳細は
[`sharmech/README.md`](../../README.md) の「パターンBを将来追加するための備え」、
robot構成の確認内容は `CLAUDE.md` の「ロボット構成 (5軸パラレルリンク)」を参照。

## 役割

`/catchrobo/command/cartesian` (`motion_generator_node` が配信する位置+速度ストリーム)
を購読し、5軸パラレルリンク機構の逆運動学を適用して、5モータ個別の関節角+角速度を
`/catchrobo/command/joint` に publish する。

このロボットは**座標 (x,y,z) のみ**を実現でき、姿勢 (pitch/yaw) の自由度を持たない。
Cartesian指令の姿勢成分 (`pose.orientation`) は無視する。単位クォータニオンでない
指令が来た場合は、ログをスパムしないよう初回のみ警告する。

### ロボット構成のおさらい

- **肩 (2モータ)**: 2つの固定ピボットに絶対値が同じ角度で対称駆動するモータ。
  対称性によりEEは常に2ピボットを結ぶ直線の垂直2等分線上にあり、中点からの距離`r`が変化する
- **ターンテーブル (1モータ)**: 肩機構全体を回転させ、xy平面内の方向`θ`を与える (r-θ型アーム)
- **肘/膝 (2モータ)**: 肩機構と数式上同型の対称二軸駆動。高さ`z`を直接与える

xyとzを独立に計算する。数式の導出・幾何拘束は
`sharmech_core/include/sharmech_core/utility/parallel_arm_kinematics.hpp` のコメントと
`test/test_parallel_arm_kinematics.cpp` を参照。このノード自身は IK の入出力の
配線 (トピック購読・publish・パラメータ読み込み・警告ログ) のみを担い、
運動学の数式そのものは持たない (utility ヘッダーに委譲)。

### やらないこと

| やらないこと | 担当 |
|---|---|
| 軌道生成・速度積分・ウォッチドッグ・作業領域クランプ | `motion_generator_node` |
| UDPパケット組立・MCUとの送受信 | `hardware_bridge_node` |
| 姿勢 (pitch/yaw) の実現 | 不可能 (このロボットに自由度が無い) |

## インターフェース

### Subscribe

| トピック | 型 |
|---|---|
| `/catchrobo/command/cartesian` | `sharmech_msgs/CartesianCommand` |

### Publish

| トピック | 型 | 内容 |
|---|---|---|
| `/catchrobo/command/joint` | `sensor_msgs/JointState` | `name = [shoulder_left, shoulder_right, turntable, knee_left, knee_right]`。`position`・`velocity` を同時に埋める (「位置と速度は常に併送する」設計判断に合わせている) |

IKが失敗した場合 (到達不可能・速度特異点) はそのサンプルを**送信せず**破棄し、
5秒間隔でスロットルした警告を出す (100Hzでのログスパムを避けるため)。

## パラメータ (`config.yaml` の `kinematics_node`)

肩・肘/膝それぞれのリンク長・ピボット間距離、ターンテーブル軸位置、肘/膝機構の基準高さ。
**すべて仮値 (0.0) のまま。** CAD/実測が無いため独断で数値を決めていない。
`sharmech_bringup/config/config.yaml` の `kinematics_node:` セクションを参照。
実測値が入るまで `launch` の `pattern_b` 引数を `false` (既定) にしてある。

## 内部状態

| 変数 | 役割 |
|---|---|
| `geometry_` | パラメータから読み込んだ `ParallelArmGeometry` (リンク長等) |
| `last_joints_` | 直前に解けた関節角。IKの2解のうち連続性が高い方を選ぶための reference |
| `warned_orientation_ignored_` | 姿勢無視の警告を初回のみに抑えるフラグ (`hardware_bridge_node` の `warned_joint_names_` と同じパターン) |

## 設計判断

### IKの2解問題と reference による連続性優先

`SymmetricLinkage::computeMotorAngle` (肩・肘/膝共通の対称二軸駆動の逆運動学) は、
同じ距離 `d` に対して数学的に2つの関節角解を持ちうる。これは肘の配置が2通りある
という一般的な2リンクIKの話ではなく、`computeExtension` が選ぶ"+sqrt"側の**同じ曲線**が
`θ` について単調でないために起こる (round-tripテストで実際に確認した)。

どちらの解も数学的には正しいが、指令のたびに解が飛び移ると関節が不連続に動いてしまう。
そのため `kinematics_node` は直前に解けた関節角を `reference` として
`ParallelArmKinematics::inverseKinematics` に渡し、そちらに近い解を選ばせている。
初回 (reference が無い) は絶対値最小の解が選ばれる。

### IK失敗時はサンプルを破棄する (ゼロ埋めしない)

速度IKが失敗した (特異点) 場合に位置だけ送って速度をゼロ埋めすると、特異点であることが
MCU側から見えなくなり、`hardware_bridge_node`/`motion_generator_node` と同様の
「異常を隠さず表面化させる」方針に反する。届かない・特異点のサンプルは送らず、
スロットルした警告に留めている。

## 未決定事項

- 各リンク長・ピボット間距離・ターンテーブル軸位置・肘/膝機構の基準高さ (要実測)
- モータ角のゼロ点・回転方向が `parallel_arm_kinematics.hpp` の想定
  (「もう一方のピボットへ向かう方向を0、EE側へ回転するほど増加」) と一致するか (MCU側確認待ち)
- 姿勢 (pitch/yaw) をこの5軸でどう扱うか (現状は無視するのみ)

~~`hardware_bridge_node` の `command_mode: "joint"` が無いため出力が未接続~~ は
**2026-09-01 解消** (`UdpProtocol::encodeJoint` と `packet_type = 0x02` の送信経路を実装。
並び順の契約は `udp_protocol.hpp` の `kJointOrder`)。
