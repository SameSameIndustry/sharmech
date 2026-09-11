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
4. VR の指定箱に「何個目を置いたか」(`box_count`) が届いていれば、そのスロットの
   上空へ運び、到達したら「縦にする」指示を MCU へ送り、**箱へは降下せず**その高さで
   離して設置する (2026-09-11 ユーザー決定。`PLACING` は運搬高さのまま動かない)
5. 箱にぶつからない高さまで退避する
6. 次のスロットへ進み、1に戻る (24箇所すべて埋まったら完了)

シューティングボックスは4箱×6箇所=24箇所。**どの位置に何番目に置くか**を
`placement_order` (スロットIDの配列) で持ち、スロットIDごとの座標は
`slot_x/y/z_<color>` で持つ。**何個目まで置くかは VR の
`/catchrobo/game/box_count` が決める** (`box_count = N` → `placement_order[N-1]`
のスロットへ。下記「box_count とスロットのキュー」)。フィールドの色 (赤/blue) によってスロット座標が
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
| `/catchrobo/game/box_count` | `std_msgs/Int32` | VR の指定箱にワークを離した**通算個数**(1始まり)。`N` は「`placement_order[N-1]` のスロットへ置きに行く」を意味する。**置きに行くべきスロットのキュー**として扱い、`place_request` 相当の「置け」指示も兼ねる(下記) |
| `/catchrobo/arm/status` | `sharmech_msgs/MotionStatus` | `motion_generator_node` の状態。`last_result` の変化でゴール到達/却下を検知する |
| `/catchrobo/game/toggle_manual_control` | `std_msgs/Empty` | `joy_teleop_node` が DualSense の4ボタン同時押しを検知して publish。**どの状態からでもトグルできる** (下記「自由操作」節) |
| `/catchrobo/game/reset` | `std_msgs/Empty` | 状態のリセット要求(VRメニューの「ステートリセット」)。**どの状態からでも** `kInit` へ入り、**初期位置 (`init_pose_*`) へ直線 1 本のゴールを出す** (2026-09-11〜。それ以前は MCU 側の初期関節角へ UDP bit2 で戻していた)。到達したら `kWaitingForPick` へ復帰する(下記「初期位置と状態のリセット」) |
| `/catchrobo/game/confirm` | `std_msgs/Empty` | **微調整の確定。** `kAdjustingPick` / `kAdjustingPlace` でのみ有効で、それ以外の状態では無視する。`joy_teleop_node` の確定ボタン (既定R3) と VR のサムズアップが、どちらもここへ publish する契約 |
| `/catchrobo/command/cartesian` | `sharmech_msgs/CartesianCommand` | `motion_generator_node` が `control_rate` (既定10Hz) で出す現在の目標姿勢と**動作許可 (`enable`)**。姿勢は**微調整でジョグした結果を知るために**購読し (publish はしない)、直後の垂直移動の起点に使う。`enable` の **false → true の立ち上がり**は「`motion_generator_node` が MCU の実姿勢へ同期し終えた = 動かしてよい」の合図で、`kInit` はこれを待ってから動き出す (2026-09-11 追加) |
| `/catchrobo/debug/change_state` | `std_msgs/String` | デバッグ専用。状態名 (`"APPROACHING"` 等) を受けて `forceState()` で強制的にその状態へ飛ばす。ゴール・グリッパ・クランプは一切 publish しない (その状態の見た目だけを確認したいとき用)。未知の状態名は無視して警告ログを出す |

`pick_request` の型を `PoseStamped` にしたのは、VR は既に `/catchrobo/field/cylinders`
(`cylinder_detector_node` の出力) から選んだ姿勢をそのまま持っているため。インデックス指定
方式にすると本ノードが検出結果を保持する必要が生じ、責務が増える。

### Publish

| トピック | 型 | 説明 |
|---|---|---|
| `/catchrobo/arm/target_pose` | `geometry_msgs/PoseStamped` | 自動シーケンスのゴール。**既存の VR/PS4 と同じトピックに直接 publish する**(下記)。**`INIT` の初期位置へのゴールもここから出る** (2026-09-11〜。専用の経路は持たない) |
| `/catchrobo/arm/gripper` | `std_msgs/Bool` | 自動シーケンスのグリッパ指令 |
| `/catchrobo/arm/orient_vertical` | `std_msgs/Bool` | PLACING 中のみ `true` |
| `/catchrobo/game/workspace_clamp` | `sharmech_msgs/WorkspaceClamp` | PLACING/RETRACTING 前後の作業領域クランプ上書き。詳細は [`motion_generator_node.md`](motion_generator_node.md) |
| `/catchrobo/game/jog_limit` | `sharmech_msgs/JogLimit` | **微調整中 (ADJUSTING_*) のジョグ速度上限。** 下記「微調整中はジョグを遅くする」 |
| `/catchrobo/game/state` | `std_msgs/String` | 現在のゲームステート。**latched (transient_local)**、`state_publish_rate` (既定10Hz) |

### パラメータ

| パラメータ | 既定値 | 説明 |
|---|---|---|
| `field_color` | (既定値なし。**必須**) | `"red"` / `"blue"`。launch 引数 `field_color:=` で指定。他の値ならノード起動失敗 |
| `box_center_x_red` / `_blue` | CAD実測 | 各箱の中心X [m]。**配列の長さ = 箱の数。箱を動かしたらここを変える** |
| `box_center_y_red` / `_blue` | 0.5255 | 箱の中心Y [m] (全箱共通) |
| `slot_cols_x` / `slot_rows_y` | 2 / 3 | 箱内の格子。X方向(短辺)の列数 / Y方向(長辺)の行数 |
| `cylinder_diameter_m` | 0.071 | 缶の直径 [m]。CAD実測を正本 (スケッチ実測の66mm説が正しければ 0.066) |
| `slot_gap_x_m` / `slot_gap_y_m` | -0.002 / 0.014 | **★当日調整する主役。** 隣り合う缶の隙間 [m] (中心間距離 = 直径 + この値)。缶が当たるなら増やし、箱からはみ出すなら減らす |
| `box_inner_size_x_m` / `_y_m` | 0.138 / 0.255 | 箱の内寸 [m]。**はみ出し警告に使うだけで座標計算には入らない** |
| `slot_x_blue` / `slot_y_blue` / `slot_z_blue` | 2026-08-30確定 (X/Yのみ) | 青フィールドのスロット座標。`field`がロボット自身のベース座標系であることと「赤の線対称」という前提から、redと同じローカル数値になっている(下記) |
| `placement_order` | `[0..23]` (箱単位で埋める順) | 配置する順番のスロットID列。「ちょうど6個」ボーナスを狙うなら箱単位が既定として妥当 |
| `slot_clamp_margin_m` | 0.03 | ORIENTING〜RETRACTING中の作業領域クランプの片側マージン [m] |
| `require_manual_confirm` | true | true: 掴む直前・離す直前で止まり操縦者の確定を待つ / false: 止まらず完全自動。実機で位置合わせの精度が出るまでは true 推奨 |
| `box_top_z_m` | 0.156 | **高さの基準面。箱の上端の高さ [m]。当日実測して入れるのはこれ1個でよく、下の相対値がすべて追従する** |
| `pick_z_m` | 0.20 | **掴みに降りる先の絶対 z [m]。`pick_request` の z は常にこれで上書きする** (VR は缶オブジェクトの原点 = 底面 z=0 を送ってくるため、そのまま使うと z=0 まで降りる。2026-09-11 ユーザー指示)。`field_origin_offset_z_m` が足される。正本は `robot_geometry.yaml` の `work_placement.pick_z_m` |
| `slot_release_below_box_top_m` | 0.106 | スロット座標の z (基準面から何m下か)。**2026-09-11 以降 `PLACING` はここへ降下せず運搬高さで離すので、動作には使われない** (座標定義として残置) |
| `approach_clearance_above_box_top_m` | 0.044 | APPROACHING (空のグリッパ) で水平移動する高さ。基準面から何m上か (既定で絶対 z=0.20 相当)。**`retract_...` と同じ値にしておくこと** (揃っていれば退避高さのまま接近でき、接近が完全な水平移動になる) |
| `transport_clearance_above_box_top_m` | 0.044 | TRANSPORT_LIFT / TRANSPORTING (缶を保持) の高さ。同上 |
| `retract_clearance_above_box_top_m` | 0.044 | 設置後に上げる高さ。同上 |
| `field_origin_offset_z_m` | 0.0 | Z方向の平行移動 [m]。基準面の実測値はそのままに全体を上げ下げする逃げ道 |
| `grasp_dwell_sec` | 0.3 | GRASPING状態でグリッパを閉じてから待つ時間 [s] (グリッパの実フィードバックが無いための暫定措置。下記) |
| `orient_dwell_sec` | 0.5 | ORIENTING状態でワークを縦にし切るまで待つ時間 [s]。**ピッチ機構の速度が未実測なので0.5は仮値**。短すぎると缶が斜めのまま箱へ降下する (下記「縦にするタイミング」) |
| `state_publish_rate` | 10.0 | `/catchrobo/game/state` の配信周期 [Hz] |
| `init_pose_r_red` / `_blue` | 0.30 (仮値。コード上の `declare_parameter` 既定は 0.0 で、これは起動時に弾かれる) | **初期位置の r [m]。** ターンテーブル軸からの距離。**正本は `sharmech/params/robot_geometry.yaml` の `init_pose` セクション** (生成物 `robot_geometry.generated.yaml` 経由)。実行中に `ros2 param set` で変えられる (次の `INIT` から効く) |
| `init_pose_theta_red` / `_blue` | 0.0 (仮値) | 初期位置の θ [rad]。**+X から時計回りが正** (UDP と同じ向き。`docs/mcu_spec.md` §3.2/§5) |
| `init_pose_z_red` / `_blue` | 0.15 (仮値) | 初期位置の z [m] (ベース座標系)。**`field_origin_offset_z_m` は掛からない** (下記) |
| `turntable_axis_x_m` / `_y_m` | 0.0 / 0.0 (未実測) | 上の極座標の原点 = ターンテーブル回転軸の位置 [m]。`hardware_bridge_node` と同じ値が `robot_geometry.yaml` の `kinematics` から生成される |
| `init_on_startup` | true | true: 起動直後に `kInit` から始まり、動作許可が出て `init_delay_sec` 後に初期位置へ動く (**起動しただけでアームが動く**) / false: 従来どおり `kWaitingForPick` から始まる。**起動時にだけ読む** (`ros2 param set` では変えられない) |
| `init_delay_sec` | 3.0 | `kInit` で動作許可 (`enable`) が出てから、実際に初期位置へのゴールを出すまでの待ち [s] (2026-09-11 ユーザー指示。同期した瞬間に動き出さず一呼吸置く)。`reset` でも同じ。0 以上。実行中に変えられる (次の `INIT` から) |
| `field_origin_offset_x_m` / `_y_m` | 0.0 / 0.0 | 本番設置での原点ズレ補正 [m]。読み込んだスロット座標全体をこの分だけ平行移動する。`motion_generator_node` と同じ値を使う想定 |

> **名前について。** 本ドキュメントの状態遷移の説明に出てくる
> `approach_clearance_z` / `transport_clearance_z` / `retract_clearance_z` は
> `GameStateMachine::Config` の内部フィールド (**絶対高さ** [m]) を指す。
> ROS パラメータ側は 2026-09-06 に基準面からの相対
> (`*_above_box_top_m`) へ変わっており、内部の絶対値は
> `box_top_z_m + *_above_box_top_m + field_origin_offset_z_m` として組み立てられる。

### 微調整中はジョグを遅くする (2026-09-10)

`ADJUSTING_PICK` / `ADJUSTING_PLACE` に入っている間だけ、`/catchrobo/game/jog_limit` で
`motion_generator_node` のジョグ速度上限を `adjusting_jog_v_max` (既定 0.05 m/s) に絞る。
状態を抜けたら `reset = true` を送って起動時の上限へ戻す。

**変化した瞬間だけ publish する** (状態は `state_publish_rate` で回っているので毎回送ると無駄)。

なぜこのノードがやるのか:

- **ゲームの状態を知っているのはこのノードだけ**だから。`motion_generator_node` は
  ゲーム進行を知らない輸送・生成層で、`/catchrobo/game/state` も購読していない。
  作業領域クランプ (`workspace_clamp`) と全く同じ構図
- **操縦層 (VR / PS4) に状態依存のロジックを置かなくて済む。** 以前は WebXR
  クライアントが ROS2 の状態名の一覧を持ち、状態ごとに速度倍率を掛けていた。
  ROS2 側が状態を1つ増やすたびにクライアント側の配列を手で直す必要があり、
  実際に `ADJUSTING_*` が漏れて微調整のジョグが出なくなる事故が起きた (2026-09-08)
- VR も PS4 もシミュレータも、何も変えずに同じ挙動になる

掴む/離す直前は缶に一番近く、行き過ぎるとワーク破損 (競技で-1点) に直結する。
`a_max` (既定 0.2 m/s²) があるので上限 0.05 m/s には 0.25 秒で到達し、以降は等速になる。
「倒した時間に比例して進む」予測しやすい挙動になり、離した後の滑りも 6mm 程度に収まる
(上限が高いと、倒し続けている間ずっと加速し続けて行き過ぎやすい)。

**実機の手感に合わせて `ros2 param set /game_state_manager_node adjusting_jog_v_max <値>`
で調整すること** (再起動不要。次に ADJUSTING へ入った時点から効く)。

### スロット座標の決まり方 (2026-09-06 に生成方式へ変更)

**24箇所の座標を直接書くのをやめ、箱の中心と缶の隙間から起動時に生成する。**
当日フィールドに合わなかったときに直す場所を最小にするための構成:

| 当日起きたこと | 直すパラメータ |
|---|---|
| 箱を動かした / 並べ方を変えた | `box_center_x_*` (箱の数もここで決まる) ・`box_center_y_*` |
| 缶同士が当たる / 箱に入らない | **`slot_gap_x_m` / `slot_gap_y_m`** |
| 高さが合わない | `box_top_z_m` (基準面。1個で全部追従) |
| フィールド全体がずれた | `field_origin_offset_x/y/z_m` |

**すべて実行中に `ros2 param set` で即反映できる (再起動不要)。**
不正な値は却下され、直前の設定のまま動き続ける。生成結果は起動時と変更時に
ログへ出る (格子の広がり vs 箱の内寸)。缶同士が重なる場合・箱をはみ出す場合は
警告が出るので、**当日その場で気づける。**

```
slot grid: 2x3/box, diameter=71mm, gap=(-2.0, 14.0)mm,
           span=(140.0, 241.0)mm vs box inner=(138.0, 255.0)mm, slot_z=0.050m
[WARN] 隣り合う缶が重なっています ...
[WARN] スロット格子が箱の内寸をはみ出しています ...
```

スロットIDは0始まりの通し番号で、箱ごとに「X列が外側・Y行が内側」の順
(箱0の (x0,y0),(x0,y1),(x0,y2),(x1,y0),(x1,y1),(x1,y2) → 箱1の…)。

**箱内の6個の並べ方そのものは未解決。** 既定値は箱の内寸を均等に2(X)×3(Y)分割した計算値
(詳細は `field_dimensions.md`)。なお**箱の位置自体はルール上 選手が自由に調整してよい**
ので、本番の並べ方を変えたら `sharmech/params/robot_geometry.yaml` の `shooting_box` も変えること
(2026-09-08〜。`config.yaml` には無い。手順は `sharmech/docs/parameter_tuning.md`)。実際のスロット配置が
分かり次第、この配列を差し替えること。

### なぜ `field_color` の launch 引数を必須にするか

**2026-08-30、赤フィールドを線対称にしたものが青フィールドという前提でスロット座標を
確定した(ユーザー確認済み)。** `field` 座標系がロボット自身のベース座標系であるため
(世界座標での鏡映と、ロボット自身から見た自己参照の座標系が打ち消し合う)、
赤・青で `slot_x/y_red` と `slot_x/y_blue` は同じローカル数値になっている
(詳細は `sharmech/docs/field_dimensions.md`)。**ただしロボット本体の組み方
(左右の関節配置等)まで鏡映になっている場合はこの前提が崩れる**ため、その場合は
別途確認が必要。どちらの色で起動しているか取り違えると実機が全く違う座標へ動こうと
する重大な事故につながるため、**config.yaml にも launch にもデフォルト値を置かず**、
起動のたびに `ros2 launch sharmech_bringup sharmech.launch.xml field_color:=red`
のように明示させる(値が同じであっても、取り違え防止のために明示は必須のままにする)。

## ゲームステートと各状態での動作

```mermaid
stateDiagram-v2
  [*] --> INIT: init_on_startup = true (既定)
  [*] --> WAITING_FOR_PICK: init_on_startup = false
  INIT --> INIT: enable 待ち (まだ動かない)
  WAITING_FOR_PICK --> APPROACHING: pick_request 受信
  APPROACHING --> APPROACH_DESCEND: ゴール到達 (ワーク上空)
  APPROACH_DESCEND --> ADJUSTING_PICK: ゴール到達 (require_manual_confirm=true)
  APPROACH_DESCEND --> GRASPING: ゴール到達 (require_manual_confirm=false)
  ADJUSTING_PICK --> GRASPING: confirm (操縦者の確定)
  GRASPING --> TRANSPORT_LIFT: grasp dwell 経過 (既定0.3s) かつ box_count のキューあり
  TRANSPORT_LIFT --> TRANSPORTING: ゴール到達 (垂直上昇の完了)
  TRANSPORTING --> ORIENTING: ゴール到達 (スロット上空)
  ORIENTING --> PLACING: orient dwell 経過 (既定0.5s)
  PLACING --> ADJUSTING_PLACE: ゴール到達 (require_manual_confirm=true)
  PLACING --> RETRACTING: ゴール到達 (require_manual_confirm=false)
  ADJUSTING_PLACE --> RETRACTING: confirm (操縦者の確定)
  RETRACTING --> WAITING_FOR_PICK: 次のスロットへ
  RETRACTING --> COMPLETE: 24箇所すべて完了
  WAITING_FOR_PICK --> MANUAL_CONTROL: 4ボタン同時押し
  MANUAL_CONTROL --> WAITING_FOR_PICK: 4ボタン再押し
  WAITING_FOR_PICK --> INIT: reset 受信
  INIT --> WAITING_FOR_PICK: ゴール到達 (初期位置)
  INIT --> MANUAL_CONTROL: 4ボタン同時押し (戻りは WAITING_FOR_PICK)
  WAITING_FOR_PICK --> INIT: enable の立ち下がり (MCU 未初期化・フィードバック途絶)
```

**簡略化の注記:** `MANUAL_CONTROL` と `INIT` は図では `WAITING_FOR_PICK` からのみ
描いているが、実際は `APPROACHING`/`GRASPING`/`TRANSPORTING`/`PLACING`/`RETRACTING`/
`COMPLETE` を含む**どの状態からでも**入れる。`MANUAL_CONTROL` はトグルし直すと
退避していたその状態へ戻る (`GRASPING` 中に入った場合は dwell タイマーも入れ直す)。
`INIT` は初期位置へ動いてから `WAITING_FOR_PICK` に復帰する (下記「初期位置と状態のリセット」節)。
全状態からの矢印を描くと読みにくくなるため、代表として1本にまとめている。
正確な条件は次の表を参照。

`INIT` に入った時点で動作許可 (`/catchrobo/command/cartesian` の `enable`) が出ていなければ
**ゴールを出さずに待ち** (図の自己遷移 `INIT --> INIT`)、立ち上がり (既に出ていれば `INIT` 突入)
から **`init_delay_sec` (既定 3s) 待って**初期位置へのゴールを 1 本出す (`tick` が予定時刻
`init_goal_due_sec_` を見て出す)。出したかどうかは `init_goal_sent_` (bool) で覚え、出す前に
届いた到達・却下は自分のものではないとして無視する。待ちの途中で `MANUAL_CONTROL` に入るか
`forceState` されたら予約は捨てる。

| # | 遷移 | 条件 | 備考 |
|---|---|---|---|
| 1 | `kWaitingForPick` → `kApproaching` | `pick_request` (PoseStamped) 受信 | 他の状態で受信しても無視 |
| 2 | `kApproaching` → `kApproachDescend` | `arm/status` の `last_result = SUCCEEDED` (ワーク上空に到達) | ここではまだ掴まない。真下へ降ろすゴールを発行する |
| 2b | `kApproachDescend` → `kAdjustingPick` | `last_result = SUCCEEDED` (降下完了) **かつ** `require_manual_confirm` | ゴールを出さず静止して待つ。この間ジョグで位置を微調整できる |
| 2c | `kAdjustingPick` → `kGrasping` | `/catchrobo/game/confirm` 受信 | グリッパ close を同時発行。`require_manual_confirm=false` なら 2b を飛ばして 2c 相当が直接起きる |
| 3 | `kGrasping` → `kTransportLift` | 経過時間 ≥ `grasp_dwell_sec` (既定0.3s) **かつ** `box_count` のキューが空でない | grasp成功の実フィードバックは無い (下記「grasp判定が時間待ちである理由」)。キューが空の間は掴んだ位置で待つ |
| 3b | `kTransportLift` → `kTransporting` | `last_result = SUCCEEDED` (上昇完了) | 掴んだ場所の真上まで上がってから、水平移動に入る |
| 4 | `kTransporting` → `kOrienting` | `last_result = SUCCEEDED` (スロット上空に到達) | 「置け」の指示は `box_count` が既に兼ねている (下記「box_count とスロットのキュー」)。`orient_vertical=true` とクランプ絞りをここで発行 |
| 5 | `kOrienting` → `kPlacing` | 経過時間 ≥ `orient_dwell_sec` (既定0.5s) | ピッチ機構の実フィードバックは無い (下記「縦にするタイミング」)。ゴールは xy=スロット・z=`transport_clearance_z` (**降下しない**。実質その場) |
| 5b | `kPlacing` → `kAdjustingPlace` | `last_result = SUCCEEDED` **かつ** `require_manual_confirm` | ゴールを出さず静止して待つ。離す前に位置を微調整できる |
| 5c | `kAdjustingPlace` → `kRetracting` | `/catchrobo/game/confirm` 受信 | グリッパ open + 退避を発行 |
| 6 | `kPlacing` → `kRetracting` | `last_result = SUCCEEDED` | グリッパ open・クランプ解除を同時発行 |
| 7 | `kRetracting` → `kWaitingForPick` | `last_result = SUCCEEDED` かつ 未処理のスロットが残っている | 次のスロットへ進む |
| 8 | `kRetracting` → `kComplete` | `last_result = SUCCEEDED` かつ `placement_order` を使い切った | 全24箇所完了 |
| 9 | `kApproaching`/`kApproachDescend`/`kGrasping`/`kTransportLift`/`kTransporting`/`kOrienting`/`kPlacing`/`kRetracting`/`kInit` → `kWaitingForPick` | `last_result = REJECTED`/`ABORTED` | 安全側フォールバック。`kWaitingForPick`/`kComplete`/`kManualControl` 中は対象外。グリッパを開き直す処理・ピッチを横へ戻す処理は無い (「既知の未対応」参照) |
| 10 | 任意の状態 ⇄ `kManualControl` | `/catchrobo/game/toggle_manual_control` | `kComplete` からも可。復帰時は退避先の状態へ。クランプは必ずデフォルトへ |
| 11 | 任意の状態 → 任意の状態 (デバッグ専用) | `/catchrobo/debug/change_state` | ゴール/グリッパ/クランプは一切publishしない。未知の状態名は無視+警告 |
| 12 | 任意の状態 → `kInit` | `/catchrobo/game/reset`、`init_on_startup=true` での起動、**`enable` の立ち下がり** (MCU 未初期化 bit3・フィードバック途絶 500ms) | `kManualControl`/`kComplete` からも可。グリッパ開 + 縦解除 + クランプ解除を同時発行する (起動時は初期状態なので何も出さない)。初期位置 (`init_pose_*`) へ**直線 1 本**のゴールを、動作許可 (`enable`) が出てから **`init_delay_sec` 後**に出す。**まだ出ていなければ出さずに待ち**、立ち上がり + `init_delay_sec` で出す。**配置の進み具合(`box_count` のキュー)は消さない** |
| 13 | `kInit` → `kWaitingForPick` | `last_result = SUCCEEDED` (ゴールを出した後のもの) | 初期位置に着くと、通常どおり `pick_request` を受けられる。却下・中断 (cancel) は #9 と同じ扱い。ゴールを出す前に届いた到達・却下 (途絶で中断された直前のゴールの結果、latched の古い status 等) は無視する |
| 14 | `kManualControl` → `kWaitingForPick` | トグル解除時に退避先が `kInit` だった | 自由操作は `INIT` の出口の 1 つ。戻ったときに途中だった初期位置への移動を再開しない |

**状態遷移の条件が変わったら、上の図と表を書き直すこと。** 正本は
[`game_state_machine.hpp`](../include/sharmech_core/utility/game_state_machine.hpp) と
[`game_state_manager_node.cpp`](../src/game_state_manager_node.cpp)。

| 状態 | 動作 |
|---|---|
| `kWaitingForPick` | 次に運ぶワークの選択待ち。`pick_request` を受理する |
| `kApproaching` | グリッパを開いたまま、選択されたワークの**真上**まで `approach_clearance_z` の高さで水平移動中 |
| `kApproachDescend` | ワークの真上から**垂直に降下**して掴む位置 (z = `pick_z_m`。`pick_request` の z は使わない) へ着ける |
| `kAdjustingPick` | **掴む直前の微調整待ち** (`require_manual_confirm=true` のときのみ)。ゴールを出さず静止し、操縦者がジョグで位置を合わせて確定するのを待つ |
| `kAdjustingPlace` | **離す直前の微調整待ち** (同上)。スロット上空 (運搬高さ) の姿勢のまま静止して待つ |
| `kGrasping` | 到達直後にグリッパを閉じ、`grasp_dwell_sec` だけ待つ(下記「grasp判定が時間待ちである理由」)。**`box_count` のキューが空ならここで宛先の指示待ちになる** |
| `kTransportLift` | 掴んだ位置で `transport_clearance_z` まで**垂直に上昇**する |
| `kTransporting` | 高さを保ったまま、キュー先頭のスロットの**真上**まで水平移動中。到達したら `kOrienting` へ |
| `kOrienting` | **スロット上空で静止したまま**、`orient_vertical` を `true` にして横倒しのワークを縦にする。作業領域クランプもここでスロット周辺 (`slot_clamp_margin_m`) に絞る。**ゴールは発行しない** (下記「縦にするタイミング」) |
| `kPlacing` | スロット上空 (`transport_clearance_z`) のまま**降下しない**。ゴールは到達点と同じ高さなので実質動かず、到達で離す段へ進む (2026-09-11 ユーザー決定。以前はスロット姿勢まで降下していた) |
| `kRetracting` | グリッパを開き、同じ xy で `retract_clearance_z` まで直線で退避。作業領域クランプをデフォルトに戻す。**縦のまま抜く** (横へ戻すのは次の `kApproaching`) |
| `kComplete` | `placement_order` を使い切った。以降 `pick_request` は無視される(実質的な終了状態)。ただし `box_count` が巻き戻ると `kWaitingForPick` へ復帰する |
| `kManualControl` | 自動シーケンス停止。**どの状態からでもトグルで入り、再度トグルで元の状態に戻る**(下記「自由操作」節) |
| `kInit` | 初期位置 (`init_pose_*`) へ **直線 1 本で動かしている最中**。グリッパは開・縦は解除・作業領域クランプはデフォルト。動作許可 (`enable`) が出るまでは動かずに待つ。到達したら `kWaitingForPick` へ(下記「初期位置と状態のリセット」節) |

**すべての状態遷移のゴールは直線1本のみ。** 経由点を持つ軌道は作らない、という
`sharmech/README.md` の既存方針をこの自動シーケンスにもそのまま適用している。
「一旦上に上げてから横に動かす」ではなく、各状態の到達点への直線移動を状態ごとに
複数回積み重ねる形にした(現在の状態 → 次の状態の目標、を1本ずつ)。
**`kInit` だけは例外的に1つの状態の中で最大3本を順に出す** (到達するたびに次の1本。
`motion_generator_node` から見れば依然として直線1本ずつなので、方針は保たれている。
`/catchrobo/game/state` の観測者からは区間に関わらず `INIT` 1つに見える)。

## box_count とスロットのキュー

**どのスロットへ何個目を置くかは、VR クライアントが送る
`/catchrobo/game/box_count` (`std_msgs/Int32`) が決める。** VR ではオペレータが
仮想フィールドの指定箱にワークを離すたびに通算個数が1つ増え、その値が publish される
(`~/catchrobo_webxr_controller/src/index.js` の `registerDesignatedBoxPlacement`)。

| box_count | 意味 |
|---|---|
| 0 → 1 | 1個目。`placement_order[0]` のスロット座標へ置きに行く |
| 1 → 2 | 2個目。`placement_order[1]` へ |
| N | `placement_order[N-1]` へ (**`count-1` が常に最新スロットIDの正本**) |

**box_count 自体は動き出しの契機ではない。** 受信時にやるのは「置きに行くべき
スロットのキュー」を更新することだけで(キュー長 = `box_count` - 消化済み個数)、
実際に動くのは `kGrasping` の dwell 経過後、キューが空でないときに `kTransportLift`
へ進むところから。そこから `kPlacing` → `kRetracting` までは**ゴール到達だけで
自動的に進む**(`place_request` に相当する「置け」指示も `box_count` が兼ねる)。
`kRetracting` の完了でキューを1つ消化する。

キューが空のまま掴んだ場合は `kGrasping` で待機し続ける。**掴んだ位置に留まる**ので、
オペレータが VR で指定箱に離した時点で運搬が始まる。

### 異常な値の扱い

**受け取った値を常に正本として追従する**(ユーザー確認済みの方針)。飛び・減少・
0リセットのいずれも、`count-1` を最新スロットIDとして消化済み個数を合わせ直す。
VR 側はフィールドを再設置するとカウントを0に戻す実装(`placeDesignatedBox`)なので、
0リセットは正常な運用の一部として起きる。`placement_order` の長さを超える値と
負値はクランプする。`+1` 以外の変化はノード側で警告ログを出す(追従はする)。

**ただし宛先を確定済みのサイクル (`kTransportLift` / `kTransporting` / `kOrienting` /
`kPlacing` / `kRetracting`) は
中断しない。** 途中でスロットが差し替わると、既に publish 済みのゴールと退避先の
xy がずれ、ワークを保持したまま別の箱の上へ動くことになるため。減少がこの3状態中に
届いた場合は、そのサイクルを最後まで終えてから効く。

### 手動微調整 (`kAdjustingPick` / `kAdjustingPlace`)

掴む位置・離す位置は、perception の検出誤差・機構のたわみ・スロット座標の実測誤差が
そのまま乗る。**実機でどれだけずれるかが分かるまでは、人が最後に一目見て直せる**
ようにしてある (`require_manual_confirm: true`)。

| | 内容 |
|---|---|
| 止まる場所 | 掴む直前 (降下しきった位置) と、離す直前 (スロット上空・運搬高さの位置) |
| 止まり方 | **ゴールを発行しない。** アームはその場に留まる |
| 調整の入力 | ジョグ (`/catchrobo/arm/cmd_twist`)。VR・PS4 どちらでもよい |
| 再開 | `/catchrobo/game/confirm` (PS4の確定ボタン / VRのサムズアップ) |

**この状態ではジョグを入れても自動シーケンスが中断されない。** 通常、自動シーケンスの
移動中にゼロでない Twist が来ると `motion_generator_node` がゴールを abort し
(`goal_mode: "twist_priority"`)、本ノードは `kWaitingForPick` まで巻き戻る。
微調整中はそもそも**実行中のゴールが無い** (到達済みで `motion_generator_node` は
IDLE) ため、ジョグは単に目標姿勢をずらすだけで abort イベントが起きない。
特別な抑制ロジックを入れずに両立できているのはこのため。

調整した分は `/catchrobo/command/cartesian` (`control_rate`。2026-09-11 以降は既定10Hz) を購読して追跡し、
**直後の垂直移動 (`kTransportLift` / `kRetracting`) の起点に反映する。**
nominal の座標へ戻してしまうと、ずらした分だけ横に動く斜め移動になり、
L字分解の意味が無くなるため。

`require_manual_confirm: false` にすると2状態とも経由せず、従来どおり
到達した瞬間に掴む/離す。試合本番で時間が足りない場合はこちらへ切り替える
(180秒で24箇所を捌く必要があるため、確定待ちの人的レイテンシは無視できない)。

#### VR側の対応 (2026-09-08 実装済みを確認)

WebXR クライアントは両手サムズアップ / メニューの「確定」で `/catchrobo/game/confirm`
(`std_msgs/Empty`) を publish する。ROS2 側はトピックを購読するだけで、ジェスチャ認識は
持たない。PS4 の確定ボタンは `joy_teleop_node` で実装済み。

### 斜めに動かない (すべての移動を「垂直 → 水平 → 垂直」に分解する)

**このノードが出すゴールは、必ず「xyを変えない垂直移動」か「zを変えない水平移動」の
どちらかになっている。** 斜めの移動は1区間も無い。

| 区間 | 種類 | ゴール |
|---|---|---|
| `kApproaching` | 水平 | ワークの xy・`approach_clearance_z` |
| `kApproachDescend` | 垂直 | ワークの姿勢そのもの (xyはそのまま) |
| `kTransportLift` | 垂直 | 掴んだ位置の xy・`transport_clearance_z` |
| `kTransporting` | 水平 | スロットの xy・`transport_clearance_z` |
| `kPlacing` | 垂直 | スロット姿勢 (xyはそのまま) |
| `kRetracting` | 垂直 | スロットの xy・`retract_clearance_z` |
| `kInit` ① | 垂直 | 今いる xy・`retract_clearance_z` (2026-09-11 追加) |
| `kInit` ② | 水平 | 初期位置の xy・`retract_clearance_z` |
| `kInit` ③ | 垂直 | 初期位置そのもの (xyはそのまま) |

分解前は接近と運搬が斜めの1直線で、それぞれ次の問題があった。

- **接近が斜めに降りる**: 退避高さからワークへ一直線に降りるため、降下しながら
  横移動する区間ができる。ワークは200mmピッチで3行×6列に並んでいるので、
  **手前のワークを薙ぎ払う**経路になりうる
- **運搬が斜めに上がる**: 掴んだ直後に低い高さのまま横へ動き出すため、
  **缶を引きずる**、あるいは隣のワークに引っ掛ける

垂直と水平に分ければ、横移動は必ず `*_clearance_z` の高さで行われるので、
**その高さが障害物より高いことだけを確認すれば安全が保証できる** (斜め移動だと
経路上のどこで高さがいくつになるかを個別に検証しなければならない)。

経由点を持つ軌道は作らない、という既存方針は保たれている。**1つの状態が出すゴールは
依然として直線1本**で、L字は「状態を分けて直線を積み重ねる」ことで作っている
(`sharmech/README.md`「経由点についての注意」参照)。

**`approach_clearance_z` は `retract_clearance_z` と同じ値にしておくこと。**
揃っていれば `kRetracting` の到達高さのまま `kApproaching` に入れるので、接近が
完全な水平移動になる。揃っていないと、その差分だけ斜めになる (次のサイクルの
接近開始時に高さを合わせ直す状態は設けていない)。

**既知の限界**: 却下・中断からの復帰では、アームがどの高さに居るかに関係なく
`kApproaching` のゴールが出るので、最初の1本は「今いる高さ → `approach_clearance_z`」の
斜め移動になる。通常のサイクル中は `kRetracting` が必ず退避高さで終わるため発生しない。
**起動直後については 2026-09-11 の `INIT` (下記「初期位置と状態のリセット」) で改善した** ——
`init_on_startup: true` なら最初に初期位置の z で止まるので、そこが退避高さと
違えばその差分だけ斜めになる (どこに居るか分からない状態ではなくなった)。
なお `INIT` 自身の移動は**直線 1 本**で L 字にはしない (下記)。

### 縦にするタイミング (`kOrienting` を独立した状態にした理由)

ワークはフィールドに**横倒し**で置かれており、シューティングボックスには
**蓋を上にして縦**に入れる必要がある (「ちょうど6個・蓋が上向き」でボーナス)。
つまりピッチ機構で横→縦の90°回転をどこかで行う。

**以前は `kPlacing` に入る瞬間に `orient_vertical = true` を出していたが、これは
降下開始と回転開始が完全に同時になるため、回転が終わる前に缶が箱へ入る。**
箱は内寸138×255mm・深さ145mmに6スロットと狭く、斜めのまま突っ込めばほぼ確実に干渉する
(2026-09-05 に判明し、`kOrienting` を新設して解消)。

タイミングの候補と判定:

| タイミング | 判定 |
|---|---|
| `kGrasping` 直後 (掴んだ直後) | ✗ 低い位置で142mmの缶を立てるため、隣の缶・治具に干渉する |
| `kTransporting` 開始時 (移動しながら回す) | △ 時間は十分だが、縦になった缶は下端が下がる。箱・設置済みの缶との干渉が**Z方向未実測のため検証できない** |
| **スロット上空で静止して回す (採用)** | ◎ 移動しないので干渉しない。時間も確保できる |
| `kPlacing` と同時 (以前の実装) | ✗ 回転が間に合わず斜めのまま突入 |

`kOrienting` は**ゴールを発行しない**ので、アームは `kTransporting` の到達点
(スロット上空・`transport_clearance_z`) で静止したまま回転だけを行う。
構造としては `kGrasping` と同じ「静止したまま dwell を待つ」状態であり、
実フィードバックが無いため固定時間待ちにしている点も同じ。

Z方向の寸法が実測できたら、`kTransporting` 中に回して dwell を省く最適化が
可能になる (1サイクルあたり `orient_dwell_sec` ぶんの短縮)。現状は安全側に倒している。

### 戻り (縦→横) を `kRetracting` で行わない理由

`kRetracting` は箱から真上へ抜く動作なので、ここで横に戻すと**箱の中でグリッパを
回す**ことになり壁に当たりうる。縦のまま抜き、横へ戻すのは次の `kApproaching`
(空中の長い移動なので回転する時間も余裕もある) にしてある。
`onPickPoseReceived()` が `orient_vertical = false` を出す形で実現している。

## 自由操作 (VRが使えない場合の脱出ハッチ)

**最悪VRが動かせない場合でも、DualSense(PS4互換)コントローラだけで最低限
試合を進められるようにする**ための機能 (2026-08-31追加)。

- `joy_teleop_node` が4ボタン同時押し(既定 L1+R1+L3+R3。詳細は
  [`joy_teleop_node.md`](joy_teleop_node.md#自由操作トグル-vrが使えない場合の脱出ハッチ))を
  検知すると `/catchrobo/game/toggle_manual_control` を publish する
- 本ノードはこれを受けて `GameStateMachine::toggleManualControl()` を呼ぶ。
  **直前の状態を1つだけ覚えておき、`kManualControl` とその状態の間をトグルする**
  (`kComplete` からも入れる。試合終了後の片付けで手動操作したい場合もあるため)
- `forceState`(デバッグ用の状態強制遷移)と同様、目標姿勢・グリッパ・
  `orient_vertical` は一切 publish しない。**ジョグ操作(cmd_twist)は
  このトグルとは無関係に常に有効**なので、状態を変えるだけで操作自体は
  即座にできる
- **作業領域クランプだけは必ずデフォルトへリセットする。** `kPlacing` 中に
  絞り込まれたクランプが残ったままだと、脱出ハッチのはずがジョグ操作を
  妨げてしまうため
- `kManualControl` 中は、通常なら `kWaitingForPick` へ引き戻す
  `onGoalRejectedOrAborted`(ゴール却下・中断時の安全側フォールバック)が
  対象外になる。手動ジョグ中に何らかの理由でゴールが却下されても、
  自由操作状態から勝手に抜けてしまわないようにするため

### 既知の制限

自由操作中に人間が手動でワークを配置しても、スロットの消化 (`order_index_`)
はステートマシンには反映されない (センサでオブジェクトの状態を検知していないため)。
元の状態 (`kApproaching` 等) に戻ったとき、自由操作に入る前の時点でステートマシンが
把握していたスロット割付のまま自動シーケンスが再開される。これは既知の制限であり、
VR復旧後に不整合が疑われる場合は運用側で判断すること。

## 初期位置と状態のリセット (`INIT`)

**初期位置は 2026-09-11 に ROS2 側へ移った。** それ以前 (2026-09-10) は MCU 側の
初期関節角が正本で、ROS2 は UDP `control_flags` bit2 を立てて頼み、到達を
`status_flags` bit5 で受け取るだけだった。現在は**このノードが行き先の座標を持ち、
普通のゴール (`/catchrobo/arm/target_pose`) として出す**。bit2 / bit5 は予約になり、
ROS2 は bit2 を常に 0 で送る (`sharmech/docs/mcu_spec.md` §3.2 / §4.6)。

### 行き先 (`init_pose_*`)

**正本は `sharmech/params/robot_geometry.yaml` の `init_pose` セクション**で、
`python3 sharmech/params/generate.py` が `robot_geometry.generated.yaml` へ
`init_pose_r_red` … の形で書き出す (手順は
[`parameter_tuning.md`](../../docs/parameter_tuning.md))。

- **UDP で MCU へ届くのと同じ極座標で持つ** —— 原点はターンテーブル軸
  (`turntable_axis_x_m` / `_y_m`)、θ は **+X から時計回りが正** [rad]、z はベース座標系 [m]。
  実機で「この関節角で止めたい」を決めるときに、フィードバックの r/θ/z をそのまま
  書き写せるようにするため (`docs/mcu_spec.md` §3.2/§5 と同じ約束)
- ノードが起動時に `PolarUtils` で直交座標へ直して `GameStateMachine::Config::init_pose`
  に入れる。変換結果は起動ログ (`game_state_manager_node started (… init_pose=(x, y, z) …)`) に出る
- **赤・青で別々**に持つ (フィールドの色で初期位置の場所が変わる)
- **`field_origin_offset_*` は掛からない。** 初期位置はロボット自身に対する姿勢であって、
  フィールドの設置誤差とは関係が無いため
- `r <= 0` や非有限値は**起動時に例外を投げてノード起動失敗** (fail-fast)。
  既定の 0.0 は「生成物が古くて値が入っていない」状態を意味する

### 動き方 (直線 1 本)

`kInit` に入ると、動作許可が出てから `init_delay_sec` (既定 3s) 待って初期位置へのゴールを
**1 本だけ** 出す (`GameStateMachine::scheduleInitGoal` → `tick` → `sendInitGoal`。pitch/yaw は 0)。他の状態のような「上げる → 水平 → 下ろす」の L 字分解は**しない**
(ユーザー決定 2026-09-11。シンプルさを優先)。箱の中や缶の列の間から斜めに抜けることに
なりうるが、**起動時は人間がおおよその初期位置にアームを置いてから電源を入れる運用で
カバーする**。試合中の `reset` / 途絶からの復帰で周囲に当たりそうなときは、先に
`MANUAL_CONTROL` のジョグで抜いてから `reset` を押す。

### 起動時も同じ経路を通る (`init_on_startup`、2026-09-11)

**`init_on_startup: true` (既定) なら、ノードは `kWaitingForPick` ではなく `kInit` から
始まる。** ただし**すぐには動かない**。`motion_generator_node` が MCU のフィードバックへ
目標姿勢を同期し終えて `/catchrobo/command/cartesian` の `enable` を false → true に
立ち上げた瞬間に、初期位置へのゴールを出す (それ以前の目標姿勢は原点の仮値なので
軌道の始点に使えない)。

```
起動 → INIT (enable 待ち、動かない)
     → MCU が 0x81 を返す → motion_generator が同期 → enable が立つ
     → INIT (init_delay_sec = 3s 待つ) → INIT (初期位置へ直線 1 本) → 到達 → WAITING_FOR_PICK
     → 最初の pick_request → APPROACHING → … → RETRACTING → WAITING_FOR_PICK (2 回目以降は INIT に戻らない)
```

`INIT` の出口は 3 つ: 到達 (→ `WAITING_FOR_PICK`)、`MANUAL_CONTROL` へのトグル (戻りは
`WAITING_FOR_PICK`)、却下・中断 (→ `WAITING_FOR_PICK`)。`pick_request` は `INIT` 中は
受けない (到達後の `WAITING_FOR_PICK` で受ける)。

### フィードバック途絶で強制 `INIT` (`enable` の立ち下がり、2026-09-11)

**`enable` が true → false に落ちたら、どの状態からでも `kInit` に入る** (ユーザー決定)。
`motion_generator_node` は MCU 未初期化 (bit3) と**フィードバック途絶** (`hardware_bridge_node`
の `feedback_timeout`、既定 500ms → `mcu_status.connected=false`) の両方で同期を取り消して
`enable=false` にするので、ケーブル抜け・MCU 再起動のどちらでもここを通る。
このときグリッパは開ける (`reset` と同じ同時発行)。ワークを保持したまま途絶すると
落とすことになるが、承知の上で単純さを優先した。ゴールは出さずに待ち、復帰して
`motion_generator_node` が実姿勢へ同期し直し `enable` が立ち上がった瞬間に初期位置へ動く。

途絶で中断された直前のゴールの `ABORTED` は `enable` の立ち下がりと前後して届くが、
`kInit` でゴールを出す前に届いた却下・中断・到達は無視するので、どちらが先でも `INIT` に留まる。

**起動しただけでアームが動く**ので、起動時に WARN を1本出している
(`INIT on startup: the arm will move to the init pose as soon as motion is enabled`)。
初期位置の値を実機で決める前の初回試験など、動かしたくない場合は
`config.yaml` の `game_state_manager_node.init_on_startup: false` にする
(この場合でも `/catchrobo/game/reset` の `INIT` は使える)。

### リセット (`/catchrobo/game/reset`)

**`/catchrobo/game/reset` (`std_msgs/Empty`) を受けると、どの状態からでも `kInit` に
入り、初期位置へ直線 1 本で戻る。** 到達したら `kWaitingForPick` へ復帰し、
そのまま次の `pick_request` を受けられる。送るのは VR クライアント
(`catchrobo_webxr_controller`) の操作メニューにある「ステートリセット」ボタン。

`kInit` に入るとき、状態と一緒に次を同時発行する。**手順が途中で崩れたときに、
アームを既知の姿勢へ戻して安全に仕切り直す**ためのもの。

| 同時発行するもの | 値 | 理由 |
|---|---|---|
| `target_pose` | 初期位置 (pitch/yaw = 0) | 直線 1 本。**同時ではなく `init_delay_sec` 後** (`tick`)。動作許可がまだなら立ち上がりを待つ (2026-09-11。それ以前は `init_request` (Empty) を1回出すだけでゴールは出さなかった) |
| `gripper` | `false` (開) | ワークを掴んだままリセットされると、その後どこで落ちるか分からない |
| `orient_vertical` | `false` (横) | 縦のまま広い範囲を動かさない |
| `workspace_clamp` | `reset = true` | `kOrienting`〜`kRetracting` の絞り込みが残っていると初期位置へ戻れない |

- **`kManualControl` からも入れる。** リセットは VR のボタンなので、押せている時点で
  VR は生きている (`kManualControl` は「VRが使えないときの脱出ハッチ」)。
  自由操作から引き出して初期位置へ戻す方が、ボタンが無反応になるより分かりやすい
- **却下・中断されたときは #9 と同じ扱い**で `kWaitingForPick` へ落ちる。
  理由はノードが警告ログに出す (初期位置が作業領域の外で `goal outside workspace` に
  なった場合など)
- **`kInit` から `kManualControl` へ入って戻ると `kWaitingForPick`** (途中だった移動を
  再開しない。自由操作は `INIT` の出口の 1 つ)
- **配置の進み具合 (`order_index_` / `authorized_count_`) は消さない。**
  「何個目まで置いたか」の正本は VR 側の `box_count` にあり、こちらだけ巻き戻すと、
  次に届いた `box_count` で既に置いたスロットへもう一度置きに行くことになる。
  配置をやり直したいときは VR 側で仮想フィールドを置き直す (カウントが0に戻る)
- **`/catchrobo/debug/change_state` で `INIT` へ飛ばしても初期位置へは動かない**
  (`forceState` はゴールを出さない)。実際に動かすのは `reset` だけ

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
- 自動シーケンス実行中に人間が手動ジョグを送っても効かないことは、当初 VR 側 UI の
  責務としていたが、**2026-09-10 に ROS2 側へ移した** (`/catchrobo/game/jog_limit` の
  `block`。「ジョグの速度制限と場面ごとの抑止」節)。手動ゴール (`target_pose`) は
  `goal_priority` の調停で実行中のゴールが優先される。操縦層に状態依存のロジックは置かない

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
| `pre_manual_state_` | `kManualControl` に入る直前の状態。トグルで戻すために1つだけ覚えておく |
| `current_pose_` | `/catchrobo/command/cartesian` で追っている現在の目標姿勢。微調整後の垂直移動の始点に使う |
| `motion_enabled_` | 直近の動作許可 (`enable`)。未受信なら false。**false → true の立ち上がり**で `kInit` のゴールを出し、**true → false の立ち下がり**で `kInit` へ入る (2026-09-11 追加) |
| `init_goal_sent_` | `kInit` で初期位置へのゴールを出したか。出す前に届いた到達・却下は無視する (同上) |
| `init_goal_due_sec_` | `kInit` でゴールを出す予定時刻 (動作許可が出た時刻 + `init_delay_sec`)。予約が無ければ `nullopt` |
| `pending_*` | 状態遷移の直後にのみセットされる「まだ publish していない指令」。ノード側が読んで publish したら消費(consume)する (edge-triggered) |

`pending_*` を edge-triggered にしているのは、**同じゴールを毎周期 publish すると
`motion_generator_node` 側で軌道が再生成され続けて永久に到達しなくなる**ため
(`onTargetPose` は新しいゴールを受理するたびに現在地から軌道を作り直す)。

## 異常系

| 事象 | 挙動 |
|---|---|
| `kWaitingForPick` 以外で `pick_request` を受信 | 無視 |
| `box_count` が `+1` 以外で変化した (飛び・減少・0リセット) | 警告ログを出し、**受け取った値に追従する**(`count-1` が正本)。ただし `kTransporting`/`kPlacing`/`kRetracting` 中は進行中のサイクルを優先し、次のサイクルから効く |
| `box_count` が `placement_order` の長さを超える / 負値 | `[0, placement_order.size()]` にクランプ + 警告ログ |
| 自動シーケンスのゴールが却下・中断された (`RESULT_REJECTED` / `RESULT_ABORTED`) | 警告ログを出し、`kWaitingForPick` へ戻る(下記「既知の未対応」)。**`kManualControl` 中は対象外** (自由操作から勝手に抜けないようにするため) |
| `field_color` が `red`/`blue` 以外、または未指定 | 起動時に例外を投げてノード起動失敗 (fail-fast) |
| `slot_x/y/z_<color>` が空または長さ不一致 | 同上 |
| `placement_order` が空、または範囲外のIDを含む | 同上 |
| `init_pose_r_<color>` が 0 以下 / 非有限、`theta`/`z` が非有限 | 同上 (2026-09-11 追加)。既定の 0.0 のまま = `robot_geometry.generated.yaml` が古いか読まれていない。メッセージに `generate.py` の実行を促す一文が入る |
| `kInit` で動作許可 (`enable`) がいつまでも立たない | **`INIT` のまま待ち続ける** (ゴールを出さないのでアームは動かない)。MCU が 0x81 を返していないか、bit3 (未初期化) が落ちていない。`ros2 topic echo /catchrobo/arm/mcu_status` で確認する |
| 動作許可 (`enable`) が true → false に落ちた (MCU 未初期化・フィードバック途絶 500ms) | **どの状態からでも `kInit` へ** (グリッパ開・縦解除・クランプ解除)。復帰して `enable` が立ち上がると初期位置へ動く (2026-09-11) |

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
(1スロットの完全なサイクル、pick の状態ガード、`box_count` のキュー動作
(キューが空の間は `kGrasping` で待つ・0リセット/飛び/クランプへの追従・
確定済みサイクルを中断しないこと・`kComplete` からの復帰)、却下時の復帰、完了判定、
`kOrienting` の待機動作 (dwell 未経過では降下ゴールを出さずその場に留まること・
`kRetracting` では縦のまま抜き次の `kApproaching` で横へ戻すこと)、
`kManualControl` のトグル・任意の状態からの出入り・クランプの強制リセット・
却下イベントの無視) を検証する。**2026-09-11 に `INIT` の一式を追加した**:
`init_on_startup` で `kInit` から始まること・動作許可が立つまで動かないこと・
立ち上がりから `init_delay_sec` 後に初期位置 (pitch/yaw = 0) へのゴールが 1 本出ること (それまでは出ないこと・待ちの途中の `MANUAL_CONTROL` で予約が消えること)・`enable` が true のまま
来ても再発火しないこと・`kInit` 以外での立ち上がりでは何も起きないこと・`reset` からの
ゴール発行と動作許可待ち・**`enable` の立ち下がりでどの状態からでも `kInit` へ入り
グリッパを開くこと・その前後に届く古い到達/却下を無視すること**・到達で
`kWaitingForPick` へ戻ること・`kManualControl` へ入って戻ると `kWaitingForPick` に
なること・`forceState(kInit)` では動かないこと・却下時の復帰・配置の進み具合を消さないこと。
`test_motion_generator_node.cpp` には `/catchrobo/game/workspace_clamp` の上書き・reset・
**フィードバック途絶での同期取消 (enable=0・ゴール ABORTED・復帰後の再同期)**・
長さ 0 のゴールで NONE → SUCCEEDED の両方が届くことの回帰テストがある。
自由操作トグルは実際に `sharmech.launch.xml` を起動し、`/joy` へ4ボタン同時押しを
模擬した `sensor_msgs/Joy` を publish して `/catchrobo/game/state` が
`MANUAL_CONTROL` ⇄ 元の状態を往復することを手動確認済み (2026-08-31)。
ノードとしての自動結合テスト(`game_state_manager_node` を実際に起動して
`motion_generator_node` と繋げる)は未実装。

**mock_mcu での E2E は、2026-09-11 の直線 1 本 + 途絶 INIT への作り直し後は未実施**
(`mock_mcu.py --power-on-pose` で電源投入位置を与えて確認する。手順は
`docs/match_runbook.md` の起動確認と同じ)。

## 未決定事項

| 項目 | 内容 |
|---|---|
| **初期位置 (`init_pose`) の値** | `robot_geometry.yaml` の `r/θ/z` は赤・青とも `status: estimate` の仮値 (0.30, 0.0, 0.15)。**実機で決める手順は [`parameter_tuning.md`](../../docs/parameter_tuning.md)「初期位置を実機で決める手順」。** 青の値は赤と同じ仮値のままで未決定。極座標の原点 (`turntable_axis_x/y_m`) は未実測 (0.0) だが、**極座標で持っているので後から軸の実測値を入れても初期位置の物理的な場所は変わらない** (`hardware_bridge_node` が同じ原点で極座標へ戻すため。変わるのは ROS2 内の直交座標の値と、作業領域クランプとの関係だけ) |
| シューティングボックスの箱内6スロットの正確な位置 | 箱の外形(赤フィールド)は実測確定。箱内の割付は均等分割の計算値 (`field_dimensions.md` 参照) |
| ロボット本体が左右対称に組まれているかの確認 | 「赤=青の線対称」という前提はロボット本体(左右関節配置等)が両チームで同じ組み方であることを仮定している。もし個体差・組み方の違いがあれば別途確認が必要 |
| グリッパの実フィードバック | `hardware_bridge_node` が UDP の `gripper_state` を publish していないため、grasp判定が時間待ちの暫定実装のまま |
| **ピッチ機構の回転速度** | `orient_dwell_sec` の 0.5 は仮値。実機のピッチ機構が横→縦を回し切る時間を実測して詰めること。短すぎると缶が斜めのまま箱へ降下し、長すぎると1サイクルあたりそのぶん試合時間を失う (24箇所×0.5s = 12s) |
| ピッチの実フィードバック | 0x81 にピッチの実状態を返すフィールドが無いため、`kOrienting` も grasp と同じ固定時間待ち。将来 MCU が返せるようになったら、グリッパとまとめて実確認へ置き換える |
| 却下・中断時の復帰処理 | グリッパを開き直す・ピッチを横へ戻す等、状態ごとのリカバリが未実装 (上記「既知の未対応」) |
| ~~VR側の `pick_request` 送信ロジック~~ | **2026-09-05 解消を確認。** WebXR クライアントは掴んだワークで `pick_request`、指定箱に離したときは `box_count` のみを送り `target_pose` は送らない (二重送信の競合も同時に解消) |
