# game_state_manager_node

「掴む→運ぶ→置く→退避」の自動配置シーケンスと、ゲーム全体の状態を管理する中核ノード。
**実装済み** (`src/game_state_manager_node.cpp`)。状態遷移の純粋ロジックは
`include/sharmech_core/utility/game_state_machine.hpp` (`GameStateMachine`) に切り出してあり、
本ノードはそこへ ROS トピックの入出力を橋渡しするだけ。

全体アーキテクチャは [`sharmech/README.md`](../../README.md) を参照。

## 背景・やりたいこと

VR の仮想フィールドでオペレータがワークを「掴んで」「置きたい場所(仮想の置き場フィールド)」
へ置くと、実機は以下を自動で行う。

1. 選択されたワークの実姿勢へ接近する
2. グリッパを閉じて把持する
3. 安全な高さまで持ち上げて運搬する
4. オペレータの「置け」指示を受けたら、シューティングボックスの決まったスロットへ
   直線1本で降下し、必要なら「縦にする」指示を MCU へ送りながら設置する
5. 箱にぶつからない高さまで退避する
6. 次のスロットへ進み、1に戻る (24箇所すべて埋まったら完了)

シューティングボックスは4箱×6箇所=24箇所。**どの位置に何番目に置くか**を
`placement_order` (スロットIDの配列) で持ち、スロットIDごとの座標は
`slot_x/y/z_<color>` で持つ。フィールドの色 (赤/blue) によってスロット座標が
異なるため、`field_color` は起動時に launch 引数で明示指定する
(詳細は下記「なぜ launch 引数を必須にするか」)。

## 役割

| 役割 | 詳細 |
|---|---|
| 状態管理 | `GameState` enum で管理し、`/catchrobo/game/state` (string, latched) に配信する |
| 自動シーケンス | 各状態1本の直線ゴールを `/catchrobo/arm/target_pose` へ順に publish する |
| 安全性 | PLACING/RETRACTING 中だけ作業領域クランプをスロット周辺に一時的に絞る |

### やらないこと

| やらないこと | 担当 |
|---|---|
| 軌道生成・速度積分・作業領域クランプの実行・ウォッチドッグ | `motion_generator_node` (このノードは`/catchrobo/game/workspace_clamp`でクランプの範囲を指示するだけ) |
| UDP 送受信・MCU との通信 | `hardware_bridge_node` |
| ワークの検出・スキャン | `catchrobo_perception` (`cylinder_detector_node`)。本ノードはpick_requestで完成済みの姿勢を受け取るだけ |
| VR の仮想フィールド描画・オブジェクトの掴む/置く操作・向き変更用の「置き場フィールド」UI | **WebXR クライアント側**(別リポジトリ、実装は本ノードのスコープ外)。本ノードはそこから届くトピックの契約(下記)を満たすのみ |

## インターフェース

### Subscribe

| トピック | 型 | 説明 |
|---|---|---|
| `/catchrobo/game/pick_request` | `geometry_msgs/PoseStamped` | VR の仮想フィールドで選択したワークの姿勢。**`kWaitingForPick` 状態でのみ有効** |
| `/catchrobo/game/place_request` | `std_msgs/Empty` | VR の「置け」指示。**`kTransporting` 状態でのみ有効** |
| `/catchrobo/arm/status` | `sharmech_msgs/MotionStatus` | `motion_generator_node` の状態。`last_result` の変化でゴール到達/却下を検知する |

`pick_request` の型を `PoseStamped` にしたのは、VR は既に `/catchrobo/field/cylinders`
(`cylinder_detector_node` の出力) から選んだ姿勢をそのまま持っているため。インデックス指定
方式にすると本ノードが検出結果を保持する必要が生じ、責務が増える。

### Publish

| トピック | 型 | 説明 |
|---|---|---|
| `/catchrobo/arm/target_pose` | `geometry_msgs/PoseStamped` | 自動シーケンスのゴール。**既存の VR/PS4 と同じトピックに直接 publish する**(下記) |
| `/catchrobo/arm/gripper` | `std_msgs/Bool` | 自動シーケンスのグリッパ指令 |
| `/catchrobo/arm/orient_vertical` | `std_msgs/Bool` | PLACING 中のみ `true` |
| `/catchrobo/game/workspace_clamp` | `sharmech_msgs/WorkspaceClamp` | PLACING/RETRACTING 前後の作業領域クランプ上書き。詳細は [`motion_generator_node.md`](motion_generator_node.md) |
| `/catchrobo/game/state` | `std_msgs/String` | 現在のゲームステート。**latched (transient_local)**、`state_publish_rate` (既定10Hz) |

### パラメータ

| パラメータ | 既定値 | 説明 |
|---|---|---|
| `field_color` | (既定値なし。**必須**) | `"red"` / `"blue"`。launch 引数 `field_color:=` で指定。他の値ならノード起動失敗 |
| `slot_x_red` / `slot_y_red` / `slot_z_red` | (仮値。要CAD実測) | 赤フィールドのスロット座標。等長配列、インデックス=スロットID |
| `slot_x_blue` / `slot_y_blue` / `slot_z_blue` | (仮値。要CAD実測) | 青フィールドのスロット座標。red の単純なミラーと決め打ちしていない(下記) |
| `placement_order` | `[0..23]` (箱単位で埋める順) | 配置する順番のスロットID列。「ちょうど6個」ボーナスを狙うなら箱単位が既定として妥当 |
| `slot_clamp_margin_m` | 0.03 | PLACING/RETRACTING中の作業領域クランプの片側マージン [m] |
| `transport_clearance_z` | 0.20 | TRANSPORTING中に上げる高さ [m] |
| `retract_clearance_z` | 0.20 | 設置後に上げる高さ [m]。箱に当たらない値にすること |
| `grasp_dwell_sec` | 0.3 | GRASPING状態でグリッパを閉じてから待つ時間 [s] (グリッパの実フィードバックが無いための暫定措置。下記) |
| `state_publish_rate` | 10.0 | `/catchrobo/game/state` の配信周期 [Hz] |

### なぜ `field_color` の launch 引数を必須にするか

CLAUDE.md にあるとおり、フィールド寸法・スロット座標は CAD 実測がまだ揃っていない
(現状は等間隔の仮値)。**加えて、シューティングボックスのスロット座標は赤/青で
単純な左右ミラーになるとは限らない**(フィールド上の設備配置が対称とは限らないため)。
どちらの色で起動しているか取り違えると、実機が全く違う座標へ動こうとする重大な事故に
つながるため、**config.yaml にも launch にもデフォルト値を置かず**、起動のたびに
`ros2 launch sharmech_bringup sharmech.launch.xml field_color:=red` のように明示させる。

## ゲームステートと各状態での動作

```
kWaitingForPick --pick_request--> kApproaching --到達--> kGrasping
                                                              |
                                                         dwell経過
                                                              v
kWaitingForPick <--(次のスロット)-- kRetracting <--到達-- kPlacing <--place_request-- kTransporting
       ^                                                                                    |
       +--------------------------(24箇所埋まったらkComplete)---------------------------------+
```

| 状態 | 動作 |
|---|---|
| `kWaitingForPick` | 次に運ぶワークの選択待ち。`pick_request` を受理する |
| `kApproaching` | グリッパを開いたまま、選択されたワークの姿勢へ直線1本で接近中 |
| `kGrasping` | 到達直後にグリッパを閉じ、`grasp_dwell_sec` だけ待つ(下記「grasp判定が時間待ちである理由」) |
| `kTransporting` | 現在のスロットのxy・`transport_clearance_z` の高さまで運搬中。`place_request` を待つ |
| `kPlacing` | 作業領域クランプをスロット周辺 (`slot_clamp_margin_m`) に一時的に絞り、スロット姿勢まで直線で降下。`orient_vertical` を `true` にする |
| `kRetracting` | グリッパを開き、同じ xy で `retract_clearance_z` まで直線で退避。作業領域クランプをデフォルトに戻す |
| `kComplete` | `placement_order` を使い切った。以降 `pick_request` / `place_request` は無視される(実質的な終了状態) |

**すべての状態遷移のゴールは直線1本のみ。** 経由点を持つ軌道は作らない、という
`sharmech/README.md` の既存方針をこの自動シーケンスにもそのまま適用している。
「一旦上に上げてから横に動かす」ではなく、各状態の到達点への直線移動を状態ごとに
複数回積み重ねる形にした(現在の状態 → 次の状態の目標、を1本ずつ)。

### `kTransporting` の目標が「スロットのxy」である理由

`kGrasping` の直後は、まだ掴んだ場所の低い高さのまま。ここでいきなり `kPlacing` の
スロット姿勢へ直線移動すると、経由点無しの制約上、低い高さのまま横移動する区間が
生じうる。そこで `kTransporting` の目標を「次に置くスロットの xy、高さは
`transport_clearance_z`」にすることで、実質的に「持ち上げながら目的地の上空へ運ぶ」
という1本の直線になる。`kPlacing` はその状態から同じ xy のままスロット姿勢まで
まっすぐ降下するだけでよい。

## grasp 判定が時間待ちである理由

MCU からのフィードバックパケット (`packet_type = 0x81`) には `gripper_state` フィールドが
既に存在するが、`hardware_bridge_node` は現状これをどの ROS2 トピックにも publish していない
(`current_pose` / `joint_states` のみ)。実際にグリッパが閉じたことを確認する手段が無いため、
**`grasp_dwell_sec` の固定時間待ちという暫定実装にしてある。** グリッパの実フィードバックが
ROS2 側で使えるようになったら、時間待ちを「実際に閉じたことの確認」に置き換えるべき。

## 自動シーケンスと `/catchrobo/arm/target_pose` の関係

自動シーケンスのゴールは、**専用チャンネルを新設せず既存の `/catchrobo/arm/target_pose`
にそのまま publish する。** VR/PS4 と同じ「調停なし・早い者勝ち」の前提に乗る形になる。

これは検討の末の選択で、代替案として「`motion_generator_node` に自動シーケンス専用の
優先入力チャンネルを追加する」も検討したが、以下の理由で見送った。

- `motion_generator_node` に手を入れずに済み、パターンA/Bのどちらでも変更不要という
  既存の設計方針([`motion_generator_node.md`](motion_generator_node.md) 参照)を維持できる
- 自動シーケンス実行中に人間が手動ゴールを送らないことは、**VR側UIの責務として担保する**
  前提にした(掴む/置く操作の間はVR側が手動ジョグ・ゴールのUIを出さない、という契約。
  この契約自体はVRクライアント側の実装で満たすものであり、本ノードのスコープ外)

ただし「UI の契約だけに頼る」のは事故時の保険として弱いため、**PLACING/RETRACTING 中だけ
作業領域クランプをスロット周辺に動的に絞る**ことで、仮に人間の手動ゴールが紛れ込んでも
遠方へは物理的に動けないようにしている(詳細は
[`motion_generator_node.md`](motion_generator_node.md) の「作業領域クランプの動的上書き」)。

## 内部状態 (`GameStateMachine`)

ROS に依存しない純粋ロジックとして `utility/game_state_machine.hpp` に実装し、
`test/test_game_state_machine.cpp` (gtest) で検証している。

| 状態変数 | 説明 |
|---|---|
| `state_` | 現在の `GameState` |
| `order_index_` | `placement_order` の何番目を処理中か |
| `pick_pose_` | 直近の `onPickPoseReceived` で受け取った姿勢 |
| `grasp_start_sec_` | `kGrasping` に入った時刻。dwell判定に使う |
| `pending_*` | 状態遷移の直後にのみセットされる「まだ publish していない指令」。ノード側が読んで publish したら消費(consume)する (edge-triggered) |

`pending_*` を edge-triggered にしているのは、**同じゴールを毎周期 publish すると
`motion_generator_node` 側で軌道が再生成され続けて永久に到達しなくなる**ため
(`onTargetPose` は新しいゴールを受理するたびに現在地から軌道を作り直す)。

## 異常系

| 事象 | 挙動 |
|---|---|
| `kWaitingForPick` 以外で `pick_request` を受信 | 無視 |
| `kTransporting` 以外で `place_request` を受信 | 無視 |
| 自動シーケンスのゴールが却下・中断された (`RESULT_REJECTED` / `RESULT_ABORTED`) | 警告ログを出し、`kWaitingForPick` へ戻る(下記「既知の未対応」) |
| `field_color` が `red`/`blue` 以外、または未指定 | 起動時に例外を投げてノード起動失敗 (fail-fast) |
| `slot_x/y/z_<color>` が空または長さ不一致 | 同上 |
| `placement_order` が空、または範囲外のIDを含む | 同上 |

### 既知の未対応

ゴールが却下・中断された場合、現在 `kPlacing` や `kRetracting` の途中でも一律
`kWaitingForPick` へ戻す。**グリッパを開き直す処理をしていない**ため、たとえば
`kPlacing` 中に却下されると「ワークを掴んだまま次の `pick_request` を待つ」状態に
なりうる。作業領域クランプを絞ったことによる意図しない却下は起きにくい設計だが、
発生した場合はオペレータが目視で気づいて手動介入する前提。将来グリッパの実フィードバックが
使えるようになったら、状態ごとの復帰処理(グリッパを開く、等)を追加すべき。

## 設計判断

全体に関わる判断は [`sharmech/README.md`](../../README.md) を参照。ここでは本ノード固有のものを記す。

### Action を使わない

`sharmech/README.md` および `motion_generator_node.md` に記載の「Action を使わない」方針を
そのまま踏襲した。自動シーケンスの実行中/完了/却下は `/catchrobo/arm/status.last_result` の
変化で検知でき、Action の feedback/result に相当する役割を既存の状態トピックが果たせている。

VR の再スキャン(10秒毎 + 手動ボタン)についても同様の理由で Action は使わず、
`catchrobo_perception` 側でトピック (`~/rescan_request`) + latched な結果トピックの
組み合わせにしてある。詳細は `catchrobo_perception/README.md` を参照。

### 状態遷移ロジックをノードから分離する (`GameStateMachine`)

`TrapezoidalTrajectory` と同様、ROS に依存しない純粋ロジックとして切り出すことで
gtest だけで状態遷移を検証できるようにした。ノード (`game_state_manager_node`) は
ROS トピックとの薄い橋渡し層に徹する。

### スロット座標を赤/青で独立した配列にする (ミラー計算にしない)

「フィールドの座標系原点はロボットのベース座標系」という既存原則([`sharmech/README.md`](../../README.md)
の「座標系」参照)により、`field_color` が変わっても座標変換自体は不要 (TFを増やさない)。
一方でスロット座標の**値そのもの**は、単純な軸反転で赤→青に変換できるかが CAD 未確認のため
分からない。誤ったミラー公式を組み込むより、素直に独立した配列として持たせ、実測値が
揃った時点でそれぞれ個別に埋める方が安全と判断した。

## テスト

`sharmech_core/test/test_game_state_machine.cpp` (gtest, ROS非依存) が状態遷移一式
(1スロットの完全なサイクル、pick/place の状態ガード、却下時の復帰、完了判定) を検証する。
`test_motion_generator_node.cpp` には `/catchrobo/game/workspace_clamp` の上書き・reset の
回帰テストがある。ノードとしての結合テスト(`game_state_manager_node` を実際に起動して
`motion_generator_node` と繋げる)は未実装。

## 未決定事項

| 項目 | 内容 |
|---|---|
| シューティングボックスのスロット座標の実測値 | 現在は等間隔の仮値。CAD実測が揃うまで赤/青とも暫定値のまま (`CLAUDE.md` 参照) |
| グリッパの実フィードバック | `hardware_bridge_node` が UDP の `gripper_state` を publish していないため、grasp判定が時間待ちの暫定実装のまま |
| 却下・中断時の復帰処理 | グリッパを開き直す等、状態ごとのリカバリが未実装 (上記「既知の未対応」) |
| VR側の `pick_request` / `place_request` 送信ロジック、仮想フィールド・置き場フィールドのUI | WebXRクライアント側 (別リポジトリ) の実装待ち。本ドキュメントのトピック契約を満たす形で実装すること |
