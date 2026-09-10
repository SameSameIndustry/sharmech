# motion_generator_node

ロボット全体のモーションを管理する中核ノード。**実装済み** (`src/motion_generator_node.cpp`)。

全体アーキテクチャは [`sharmech/README.md`](../../README.md) を参照。

## 役割

性質の異なる2種類の操縦入力を受け取り、**1本の Cartesian ストリーム(位置 + 速度)に合流させて**下流へ流す。

| 入力 | 変換 |
|---|---|
| エンドエフェクタの最終目標姿勢 (トピック) | 軌道生成 → 経過時間でサンプリング |
| エンドエフェクタの目標速度 (Twist) | レート制限 → 積分 |

あわせて、操縦層からのグリッパ指令を中継する。**全ての操縦指令がこのノードを通る**ため、ここが単一の調停点になる。

### やらないこと

| やらないこと | 担当 |
|---|---|
| 逆運動学 (IK) | MCU (パターンA) / `kinematics_node` (パターンB) |
| ハードウェア通信 | `hardware_bridge_node` |
| ジョイスティック入力の解釈 | `joy_teleop_node` |
| WebXR ↔ ROS2 座標変換 | **WebXR クライアント側**(`field.js`)。ROS2 側では一切やらない |

このノードは**運動学を一切持たない**。Cartesian 空間だけで完結する。

## インターフェース

### Subscribe

| トピック | 型 | 説明 |
|---|---|---|
| `/catchrobo/arm/target_pose` | `geometry_msgs/PoseStamped` | 目標姿勢。**rosbridge 経由のブラウザ向けのゴール入力**(下記)。`game_state_manager_node` の自動配置シーケンスもここへ直接 publish する |
| `/catchrobo/arm/cmd_twist` | `geometry_msgs/Twist` | 先端の目標速度。**ベース(ロボット固定)座標系基準** |
| `/catchrobo/arm/gripper` | `std_msgs/Bool` | グリッパ指令 (`true`=閉, `false`=開) |
| `/catchrobo/arm/orient_vertical` | `std_msgs/Bool` | 「横倒しのワークを縦にする」指令。グリッパと同じ扱いでラッチして中継する |
| `/catchrobo/arm/cancel` | `std_msgs/Empty` | 実行中のゴールを中断する。初期位置要求 (INIT) の取り下げも兼ねる |
| `/catchrobo/arm/init_request` | `std_msgs/Empty` | 初期位置要求。`game_state_manager_node` が `/catchrobo/game/reset` で送る。**座標は持たない** —— UDP `control_flags` bit2 で MCU に「自前の初期関節角へ行け」と頼む (下記「起動時の同期と初期位置要求」) |
| `/catchrobo/arm/current_pose` | `geometry_msgs/PoseStamped` | 実姿勢。起動時の同期・状態トピックの残距離・INIT 中の追従に使う |
| `/catchrobo/arm/mcu_status` | `sharmech_msgs/McuStatus` | MCU の `status_flags`。bit3 (未初期化) の間は同期しない、bit5 (初期位置到達) で INIT 完了 |
| `/catchrobo/game/workspace_clamp` | `sharmech_msgs/WorkspaceClamp` | 作業領域クランプの動的上書き。`game_state_manager_node` が PLACING/RETRACTING 前後に送る (詳細は下記) |
| `/catchrobo/game/jog_limit` | `sharmech_msgs/JogLimit` | ジョグ速度上限の動的上書き。`game_state_manager_node` が微調整中 (ADJUSTING_*) の出入りで送る (詳細は下記) |

`/catchrobo/arm/` 名前空間は「調停前の生の操縦入力」であることを示す。VR と PS4 の両方がここへ publish する。
**VR クライアントは rosbridge 経由で直接ここへ publish する。中継ノードは無い。**
`game_state_manager_node` も同じ `/catchrobo/arm/target_pose` へ直接 publish する
(専用チャンネルを新設しない判断の経緯は [`game_state_manager_node.md`](game_state_manager_node.md) を参照)。

### Publish

| トピック | 型 | 説明 |
|---|---|---|
| `/catchrobo/command/cartesian` | `sharmech_msgs/CartesianCommand` | 位置 + 速度 + 動作許可 (`enable`) + 初期位置要求 (`init_request`)。`control_rate` で定期送信 |
| `/catchrobo/command/gripper` | `std_msgs/Bool` | 調停後のグリッパ指令 |
| `/catchrobo/command/orient_vertical` | `std_msgs/Bool` | 調停後の「縦にする」指令。`hardware_bridge_node` が UDP の `control_flags` bit1 に詰める |
| `/catchrobo/arm/status` | `sharmech_msgs/MotionStatus` | 現在のモードと進捗。**latched (transient_local)**、10Hz 程度 |

**Action Server も Service も持たない。** ゴールは `/catchrobo/arm/target_pose`、
キャンセルは `/catchrobo/arm/cancel`、状態の通知は `/catchrobo/arm/status` で行う。
理由は[Action を使わない](#action-を使わない)を参照。

### メッセージ定義

```
# sharmech_msgs/MotionStatus
std_msgs/Header header
uint8   mode                  # 0=IDLE, 1=GOAL, 2=JOG, 3=INIT (MCU が初期位置へ移動中)
geometry_msgs/Pose goal_pose  # mode==GOAL のときの目標
float64 distance_remaining    # [m]
float64 time_remaining        # [s]
uint8   last_result           # 0=none 1=succeeded 2=aborted 3=rejected
string  message               # 却下理由など
```

```
# sharmech_msgs/CartesianCommand
std_msgs/Header      header    # stamp = この指令の時刻
geometry_msgs/Pose   pose      # 目標位置 + 姿勢(クォータニオン)
geometry_msgs/Twist  twist     # 目標速度(並進 + 角速度)
bool                 enable        # 動作許可 (UDP control_flags bit0)。同期完了まで false
bool                 init_request  # 初期位置要求 (bit2)。INIT モードの間だけ true
```

`enable` / `init_request` を別トピックにせず位置と同じメッセージに載せるのは、
別トピックだと DDS の発見遅れで「位置は届くがフラグは既定値のまま」という数百 ms の
窓が開き、起動直後に bit0=1 のまま原点の仮目標へ動き出したため (2026-09-10 mock_mcu で実測)。

`time_from_start` に相当するフィールドは**持たない**。ストリーミング方式のため「届いた瞬間がその点の時刻」であり、時刻は `header.stamp` と UDP ヘッダの `timestamp_us` が担う。

### パラメータ

| パラメータ | 既定値 | 説明 |
|---|---|---|
| `control_rate` | 100.0 | ストリーム出力周期 [Hz] |
| `status_rate` | 10.0 | `/catchrobo/arm/status` の配信周期 [Hz] |
| `v_max` | 0.10 | 最大並進速度 [m/s]。**ゴールの軌道生成専用** (ジョグには効かない。下記 `jog_v_max`) |
| `a_max` | 0.20 | 最大並進加速度 [m/s²]。**軌道生成とジョグのレート制限で共用** |
| `w_max` | 1.0 | 最大角速度 (pitch/yaw) [rad/s] |
| `alpha_max` | 2.0 | 最大角加速度 [rad/s²] |
| `workspace_x_min` / `x_max` | -2.045 / 0.941 | 作業領域 X [m]。2026-08-30実測確定 (赤フィールド)。詳細は `sharmech/docs/field_dimensions.md` |
| `workspace_y_min` / `y_max` | -0.675 / 0.675 | 作業領域 Y [m]。同上 |
| `workspace_z_min` / `z_max` | 0.00 / 0.30 | 作業領域 Z [m]。**未確定のまま** (今回の実測は上面図のみでZ情報を含まない) |
| `field_origin_offset_x_m` / `_y_m` | 0.0 / 0.0 | 本番設置での原点ズレ補正 [m]。上記 workspace_x/y_min/max 全体をこの分だけ平行移動する |
| `jog_v_max` | 1.0 | **ジョグの並進速度上限 [m/s]。** 受信した `cmd_twist` の大きさをこの値で頭打ちにする (下記) |
| `twist_timeout` | 0.4 | ジョグのウォッチドッグ [s] (300〜500ms) |
| `goal_mode` | `goal_priority` | ゴールとジョグの調停方式。`goal_priority` / `twist_priority` / `exclusive` (下記「入力の調停」) |

`pos_tolerance` / `rot_tolerance` (実誤差での到達判定) は、MCU フィードバックを使った
到達判定を導入する時点で追加する。現在の到達判定は経過時間のみ。

`w_max` / `alpha_max` が別にあるのは、`v_max` / `a_max` が並進 [m/s] の次元であり
姿勢 (pitch/yaw) のレート制限と純回転ゴールの所要時間計算に使えないため。

`a_max` をゴール軌道とジョグで共用するのは、**ゴール指定とジョグで動作感を揃えるため**。
別々にすると同じロボットが操作方法によって違う挙動をする。

速度の上限だけは `v_max` (ゴール) と `jog_v_max` (ジョグ) に分けてある。ゴールは
ノードが自分で速度プロファイルを決めるのに対し、ジョグは操縦者が随時止められる
前提の入力で、妥当な上限が違うため。

### ジョグ速度のクランプ (`jog_v_max` / `/catchrobo/game/jog_limit`)

**受信した `cmd_twist` の並進成分の「大きさ」を `active_jog_v_max_` で頭打ちにする**
(軸ごとに切ると斜め入力で合成速度が最大 √3 倍になるため、比を保って縮める)。
超過したときは 5 秒に 1 回まで警告ログを出す。

**2026-09-10 追加。それ以前はジョグ側にクランプが一切無く、操縦層が publish した値が
そのまま目標速度になっていた** (`v_max` はゴールの軌道生成でしか使われていなかった)。
WebXR クライアントが自前で上限を持ってこれを肩代わりしていたが、
「誰が publish してもこの上限を超えられない」ことは作業領域クランプと同じく
このノードの責務なので、こちらへ移した。

`active_jog_v_max_` は通常 `jog_v_max` と同じ値だが、`game_state_manager_node` が
`sharmech_msgs/JogLimit` で一時的に絞ることができる。

```
# sharmech_msgs/JogLimit
bool    reset     # true: active_jog_v_max_ を jog_v_max へ戻す。false なら下記を採用
float64 v_max     # 並進ジョグの速度上限 [m/s]。0以下は不正として無視する
```

- 用途は**微調整中 (ADJUSTING_PICK / ADJUSTING_PLACE) だけ遅くすること**。
  掴む/離す直前は缶に一番近く、行き過ぎるとワーク破損 (競技で-1点) に直結する
- 上書きは `jog_v_max` より**緩くはできない** (`std::min`)。安全側の設定を
  ゲーム層のバグで壊せないようにする (作業領域クランプと同じ方針)
- `v_max <= 0` の上書きは無視する。「ジョグが一切効かない」状態を無言で作らないため
- **操縦層 (VR / PS4) は何も変えなくてよい。** 今まで通り自分の都合で `cmd_twist` を
  送ればよく、ROS2 のゲーム状態を知る必要が無い。状態を持っているのは
  `game_state_manager_node` だけ、という役割分担がこれで保たれる

### 作業領域クランプの動的上書き (`/catchrobo/game/workspace_clamp`)

上記の `workspace_x/y/z_min/max` は**起動時のデフォルト**。実際にゴール判定・ジョグクランプに
使われるのは内部の `active_workspace_*` で、通常はデフォルトと同じ値だが、
`game_state_manager_node` が PLACING/RETRACTING 中だけシューティングボックスの
スロット周辺に一時的に絞ることができる (`sharmech_msgs/WorkspaceClamp`)。

```
# sharmech_msgs/WorkspaceClamp
bool    reset     # true: active_workspace_* をデフォルトへ戻す。false なら下記を採用
float64 x_min
float64 x_max
float64 y_min
float64 y_max
float64 z_min
float64 z_max
```

「PLACING 中に遠くの target_pose へ急に飛ばない」という安全性を、`motion_generator_node`
自体は変更せず既存のクランプ機構の適用範囲を実行時に変えるだけで実現するための仕組み。
上書き値は起動時のデフォルト範囲を超えないよう `motion_generator_node` 側で
`std::clamp` される (`game_state_manager_node` 側のバグで安全域が丸ごと外れることを防ぐ)。

## 内部状態

**このノードは「現在の目標姿勢」の唯一の所有者。** 軌道の始点も自分が保持する値を使う。

起動直後の目標姿勢は作業領域内にクランプした初期値 (原点付近の仮値)。その後、
**まだ何も動かしていないうちに MCU フィードバックが届いたら、目標姿勢を実姿勢に
1度だけ同期する**。同期が済むまでは `enable=false` を送り続けるので、仮値が MCU に
追従されることはない (詳細は下記「起動時の同期と初期位置要求」)。

| 状態 | 説明 |
|---|---|
| `target_pose_` | **中核の状態変数。** 現在指令中の目標姿勢 |
| `target_twist_` | 現在指令中の目標速度 |
| `mode_` | `IDLE` / `GOAL` / `JOG` / `INIT` |
| `commanded_twist_` | 操縦層から届いた生の Twist(レート制限前) |
| `last_twist_time_` | ウォッチドッグ判定用 |
| `current_twist_` | レート制限後の、実際に積分に使う速度 |
| `trajectory_` | 生成済み軌道(`GOAL` 時のみ) |
| `trajectory_start_time_` | 軌道の開始時刻。**経過時間の基準** |
| `last_result_` / `status_message_` | 直近のゴールの結果と理由。状態トピックに載せる |
| `gripper_state_` | ラッチしたグリッパ状態 |
| `synced_with_feedback_` | 目標姿勢を実姿勢へ同期済みか。**false の間は `enable=false`**。MCU が bit3 (未初期化) を報告したら false に戻す |
| `latest_mcu_flags_` | 直近の `McuStatus.status_flags`。未受信 (nullopt) なら「MCU 状態不明」で、`current_pose` だけで同期する (mcu_status を出さない古いシムとの後方互換) |

### 起動時の同期と初期位置要求 (2026-09-10)

理想の挙動 (ユーザー確定) は「MCU の電源を入れると MCU が自前の初期位置へ行く →
ROS2 を起動してもその場から動かない → 目標を与えて初めて動く」。このノードが担う部分:

| 段階 | このノードの挙動 | UDP (hardware_bridge_node 経由) |
|---|---|---|
| 起動〜同期前 | ゴールは `not synced with MCU feedback yet` で却下、ジョグは捨てる。目標は仮値のまま | `enable=false` (bit0=0) → MCU は現在位置ホールド |
| 同期 | `mcu_status` が bit3 (未初期化) を報告していないときに `current_pose` が届いたら、制御タイマー内で目標 ← 実姿勢 (1回だけ) | 以後 `enable=true`。目標 = 実姿勢なので動かない |
| `/catchrobo/arm/init_request` | `mode_ ← INIT`。実行中のゴール/ジョグは捨てる。**目標は毎周期フィードバックを写す** (bit2 を落とした瞬間に指令と実姿勢が一致するように)。作業領域クランプは掛けない | `init_request=true` (bit2=1) → MCU が自前の初期関節角へ移動 |
| 到達 | `mcu_status` bit5 (`FLAG_AT_INIT_POSE`) を見て `IDLE` + `SUCCEEDED` ("init pose reached")。`synced_with_feedback_ ← true` | `init_request=false` |
| INIT 中の `cancel` | `IDLE` + `ABORTED`。MCU が到達を返さない場合の唯一の出口 | bit2 が落ち、MCU は追いかけていた実姿勢 (≒ 現在位置) に留まる |
| MCU が bit3 を報告 (MCU 再起動) | 同期を取り消し、ゴール/ジョグを `ABORTED` ("MCU uninitialized") | `enable=false` に戻る |

**初期位置の座標はこのノードも `game_state_manager_node` も持たない** (旧 `init_pose`
パラメータは廃止)。MCU が持つ初期関節角が正本で、sim / mock_mcu もそれぞれ自前の値を持つ。
`INIT` は `GOAL` と同じく `last_result` で完了を伝えるので、`game_state_manager_node`
から見ると「ゴール1本」と同じ扱いで済む。

### 不変条件

**`target_pose_` は制御タイマーの中でのみ書き換える。コールバックからは絶対に書き換えない。**

コールバックは「入力を記録する」だけに徹し、状態遷移の判定と `target_pose_` の更新は必ずタイマー内の1箇所で行う。これを崩すと、Twist コールバックと軌道サンプリングが同じ変数を非同期に書き換えて壊れる。

## 処理フロー

### コールバック(記録と状態遷移の判定のみ)

**`/catchrobo/arm/cmd_twist` 受信時**

```
commanded_twist_ ← msg
last_twist_time_ ← now

is_active = デッドゾーン処理後の値のいずれかが 0 でない     ★重要(下記)

if goal_mode == twist_priority:
    if is_active かつ mode_ == GOAL:  ゴールを abort し、mode_ ← JOG
    if is_active かつ mode_ == IDLE:  mode_ ← JOG
else:  # goal_priority / exclusive
    if mode_ == GOAL:  Twist を無視(記録もしない)
    if is_active かつ mode_ == IDLE:  mode_ ← JOG
```

### 入力の調停 (`goal_mode`)

| `goal_mode` | ゴール実行中に非ゼロ Twist | ジョグ中にゴール |
|---|---|---|
| `goal_priority` (**既定**) | 無視 (ゴール継続) | **受理** (ジョグを止めてゴールへ。速度指令は捨てる) |
| `twist_priority` | ゴールを abort してジョグへ | 却下 (`jog active`) |
| `exclusive` | 無視 (ゴール継続) | 却下 (`jog active`) |

**2026-09-06 に既定を `twist_priority` → `goal_priority` に変えた (ユーザー判断)。**
`game_state_manager_node` の自動シーケンスは `target_pose` を順に投げて進むが、
WebXR クライアントは `cmd_twist` を毎フレーム流し続けており、左コントローラーの
わずかな上下動 (数 cm/s) や手のジェスチャーの誤検出で非ゼロが混ざる。
`twist_priority` だとそれが接近中のゴールを abort し、`game_state_manager_node` が
`WAITING_FOR_PICK` へ落ちる事故が実機で出た。ジョグはそもそも
`MANUAL_CONTROL` (自動シーケンスが止まっている状態) で使うものなので、
ゴール側を優先しても運用上失うものは無い。
`goal_priority` でジョグ中のゴールを受理するとき、`commanded_twist_` /
`current_twist_` を 0 にしてから軌道を作る。残しておくと、ゴール到達後に
次のジョグへ入った瞬間、減速しきっていない古い速度が積分されて飛ぶ。
軌道の始点は現在の指令姿勢 `target_` なので位置の段差は出ない。

**`/catchrobo/arm/target_pose` 受信時**

```
目標姿勢が作業領域外        → 却下 (last_result_ ← rejected, 理由を status_message_ へ)
mode_ == JOG (ジョグ動作中) → goal_priority なら受理 (Twist を捨ててゴールへ)、それ以外は却下
それ以外                    → 受理
  trajectory_ ← 軌道生成(始点 = target_pose_, 終点 = ゴール)
  trajectory_start_time_ ← now
  mode_ ← GOAL              ★新しいゴールは実行中のゴールを上書きする
```

作業領域外のゴールは**受理してからクランプするのではなく、却下する**。黙って別の場所へ動くより安全。

**却下は `/catchrobo/arm/status` の `last_result` と `message` で伝わる。** トピックでゴールを
投げる方式は送信元に直接返せないが、状態トピックを見れば却下されたことと理由が分かる。

**`/catchrobo/arm/cancel` 受信時**

```
mode_ == GOAL なら:
    ゴールを破棄、last_result_ ← aborted
    mode_ ← IDLE (target_pose_ はその場で保持)
```

**`/catchrobo/arm/gripper` 受信時**

```
gripper_state_ ← msg.data     (publish はタイマー内で行う)
```

### 制御タイマー(`control_rate` で実行)

```
1. ウォッチドッグ判定
   if now - last_twist_time_ > twist_timeout:
       commanded_twist_ ← 0

2. レート制限(加速・減速の両方に適用)
   current_twist_ を commanded_twist_ へ向けて、a_max * dt を上限に近づける

3. モード別に target_pose_ / target_twist_ を更新
   JOG:
       target_pose_  += current_twist_ * dt
       作業領域にクランプ
       クランプが効いた軸は target_twist_ の該当成分を 0 にする  ★重要
       current_twist_ が 0 になったら mode_ ← IDLE

   GOAL:
       t = now - trajectory_start_time_          ★経過時間で評価する
       target_pose_, target_twist_ ← trajectory_ を t で評価
       if t >= 軌道の総所要時間:
           ゴールを succeed、mode_ ← IDLE

   INIT:
       target_pose_ ← 直近の current_pose (MCU が動かしている。こちらは追いかけるだけ)
       target_twist_ ← 0
       if mcu_status bit5 (初期位置到達): succeed、mode_ ← IDLE、synced ← true

   IDLE:
       target_pose_ 維持、target_twist_ ← 0

4. publish
   /catchrobo/command/cartesian ← (target_pose_, target_twist_,
                                   enable = synced || INIT, init_request = INIT)
   /catchrobo/command/gripper   ← gripper_state_
```

(0. として、同期前 (`synced_with_feedback_ == false`) かつ IDLE かつ MCU が bit3 を
報告していなければ、制御タイマーの先頭で target_pose_ ← current_pose の同期を行う)

### ★ ゼロでない Twist のみがゴールを abort する

`joy_teleop_node` は**スティックがニュートラルでもゼロの Twist を 50Hz で送り続ける**
(ウォッチドッグ待ちを避けるため)。

したがって「Twist を受信したらゴールを abort する」と素直に実装すると、**PS4 を繋いでいる
だけでゴールが発行された瞬間に毎回 abort され、ゴール指定が一切使えなくなる。**

ゼロの Twist は「ジョグ入力なし」を意味するので、ゴールに干渉させてはいけない。

### ★ 軌道は経過時間で評価する

**配列のインデックスを1つずつ進めてはならない。** インデックスで進めると軌道が持つ時間情報が失われ、移動距離に関係なく常に同じ時間で走り切ることになり、台形速度プロファイルが完全に無意味になる。

必ず `now - trajectory_start_time_` を使って軌道を評価する。

### ★ クランプ時は速度も 0 にする

作業領域でクランプしたとき、**publish する速度の該当成分も 0 にしなければならない。**

位置は境界で止まっているのに速度が「動いている」と報告すると、MCU が速度を使って外挿するため、**作業領域の外へはみ出す**。位置と速度は常に整合していなければならない。

## 異常系

| 事象 | 挙動 |
|---|---|
| Twist が `twist_timeout` の間届かない | 速度指令を 0 とみなし、レート制限に従って減速停止。`mode_` ← `IDLE` |
| ゴールが作業領域外 | 受理せず reject |
| ジョグ動作中にゴールが来た | `goal_priority` (既定): ジョグを止めて受理 / それ以外: reject |
| ゴール実行中に**ゼロでない** Twist が来た | `goal_priority` / `exclusive` (既定): 無視してゴール継続 / `twist_priority`: ゴールを abort し、ジョグへ移行 |
| ゴール実行中にゼロの Twist が来た | **何もしない。** ゴールは継続する |
| `/catchrobo/arm/cancel` を受信 | その場で停止、姿勢を保持、`mode_` ← `IDLE`、`last_result` ← `aborted` (INIT 中も同じ) |
| `/catchrobo/arm/current_pose` が来ない | **同期できないので `enable=false` のまま。ゴールは却下、ジョグは捨てる** (MCU 未接続で動かないのは意図どおり)。状態トピックは publish し続ける |
| 同期前にゴール/ジョグが来た | ゴールは `not synced with MCU feedback yet` で却下、ジョグは記録しない |
| INIT 中にゴール/ジョグが来た | ゴールは `init request in progress` で却下、ジョグは記録しない |
| INIT 中に `mcu_status` が届かない | 到達を検出できない。5秒ごとに警告 (`cancel` で抜ける) |
| MCU が bit3 (未初期化) を報告 | 同期を取り消し `enable=false`。実行中のゴール/ジョグは `aborted` ("MCU uninitialized") |

### ゴールを abort する理由(一時停止ではなく)

手動介入で機体が別の場所へ移った後に元の軌道を再開すると、**軌道の始点と実際の位置がずれているため、いきなり飛ぶ**。やり直したければ操縦層が再発行すればよい。

## 設計判断

全体に関わる判断(ヤコビ不使用、位置+速度の併送、ストリーミング方式など)は
[`sharmech/README.md`](../../README.md) の「主要な設計判断とその理由」を参照。ここでは本ノード固有のものを記す。

### 軌道生成と速度積分を同一ノードに置く

どちらも `target_pose_` という**同一の状態変数**を書き換える。ノードを分けると状態が2箇所に分散し、モード切替時の引き継ぎが破綻する。状態を持つ主体は1つにする。

### グリッパをこのノードで中継する

グリッパは運動学を通らない量なので `hardware_bridge_node` へ直行させることもできるが、**全ての操縦指令を単一の調停点に通す**方が一貫する。VR / PS4 のどちらから来た指令も同じ扱いになる。

### 独自メッセージ型を使う

`trajectory_msgs/JointTrajectoryPoint` は関節空間用(`float64[]`)であり、Cartesian に流用すると配列のインデックス規約を上下流で共有する必要が生じる。`MultiDOFJointTrajectoryPoint` は意味的には正しいが全フィールドが配列で冗長。

一般的な系(FZI `cartesian_controllers`、`mavros`)は位置と速度を**別トピック**にしているが、それだと「サイクル N の位置」と「サイクル N-1 の速度」が組み合わさる危険がある。**同じ UDP パケットに載せるものは同じメッセージで運ぶ。**

`MotionStatus` のために `sharmech_msgs` はどのみち必要なので、メッセージを1つ足すコストはゼロ。

### Action を使わない

当初はゴール指定を Action (`MoveToPose`) で受ける設計だったが、**`/catchrobo/arm/status` を
入れた時点で Action の役割が無くなったため廃止した。**

| Action が答えていた問い | 代替 |
|---|---|
| 私のゴールは成功したか | 状態トピックの `last_result` |
| 却下されたか・理由は | 状態トピックの `last_result` / `message` |
| 今実行中か・あと何秒か | 状態トピックの `mode` / `time_remaining` |
| キャンセル | `/catchrobo/arm/cancel` |

**状態トピックの方が優れている点が4つある。**

1. **再接続に強い。** VR クライアントは自動再接続する。Action 実行中に WebSocket が切れると
   result が失われるが、latched な状態トピックは再接続時に現在状態が即座に届く
2. **誰が指令したかに関わらず見える。** VR で見ている人は PS4 由来の動作も知るべき。
   Action の feedback は自分が送ったゴールしか教えてくれない
3. **観測者が何人でもよい。** デバッグ用ダッシュボードや `catchrobo_app` が後から購読できる
4. **ゴール経路が1本になる。** Action とトピックを併存させると「Action なら reject が返るが
   トピックなら返らない」という非対称を抱え込む。1本なら VR と PS4 が完全に等価になる

「実行中」は**イベントではなく状態**なので状態として流す。グリッパで `Empty` 2本ではなく
`Bool` 1本を選んだのと同じ理由。

**将来 Action が欲しくなる場面**として想定していた「自律動作の順序制御(掴む→運ぶ→置くを
上位ノードが順に実行し、各段の完了を待つ)」は、`game_state_manager_node` の追加という形で
実際に到来した。しかしそのときも Action は導入しなかった: `game_state_manager_node` は
`/catchrobo/arm/status.last_result` の変化を購読して「ゴール到達」を検知し、次のゴールを
`/catchrobo/arm/target_pose` へ直接 publish するだけで、Action の feedback/result に相当する
役割を既存の状態トピックがそのまま果たせている。詳細は
[`game_state_manager_node.md`](game_state_manager_node.md) を参照。

### VR / PS4 の入力調停は行わない

両方を同時に使うことはないため、「早い者勝ち」のままでよい。排他切り替えの仕組みは作らない。

### 経由点を持つ軌道は作らない

一連の動作(掴む→持ち上げる→運ぶ→置く等)は各段階で一旦止まってよい。操縦層が複数の
ゴールを順番に送ればよく、`trajectory_` は始点から終点への直線1本の生成のみでよい。
複数区間をまたぐコーナリング処理(経由点で速度を不連続にしないための平滑化)は不要。

## テスト

`sharmech_core/test/test_motion_generator_node.cpp` (gtest) がこのノードを実際に起動し、
ウォッチドッグ・作業領域クランプ・ゴール/ジョグの調停を検証する。「実装上の罠」節に
挙げた項目の回帰テストを兼ねるので、この節を変更したらテストも合わせて見直すこと。

## 未決定事項

| 項目 | 内容 | 暫定案 |
|---|---|---|
| ウォッチドッグ時の減速度 | `a_max` で減速すると全速から停止まで 0.5s / 2.5cm 進む。安全上これで良いか。即時 0 にすると加加速度が無限大になり機構に負担 | `a_max` で減速。必要なら `a_stop` を別パラメータ化 |
| 到達判定 | 現状は「軌道の所要時間が経過したら完了」で、実姿勢との誤差は見ていない (フィードバック自体は 2026-09-10〜 起動時の同期と INIT で使っている)。MCU 側のスルーレート制限で遅れた分は次のゴールに持ち越される | 実誤差で判定したくなったら `pos_tolerance` / `rot_tolerance` を追加する。追加する場合、遅れて到達しない MCU で永久に完了しない問題への対策 (タイムアウト) が要る |
