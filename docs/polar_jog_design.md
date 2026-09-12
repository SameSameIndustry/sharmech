# 極座標ジョグ (`/catchrobo/arm/cmd_polar_twist`) 設計書

2026-09-12 作成。**実装前の設計書** (実装後は各ノードの docs に内容を移し、本書は経緯として残す)。

## 目的

PS5 (DualSense) から先端を **r・θ・z (極座標) で直接ジョグ**できるようにする。
機構が r-θ 型 (ターンテーブル = θ、肩の対称二軸 = r、肘/膝 = z) なので、
操縦者にとっては「ターンテーブルだけ回す」「腕だけ伸ばす」が自然な操作になる。

割り当て (ユーザー指定): **L2/R2 で θ、左スティック縦で r、右スティック縦で z。L1 (デッドマン) 不要。**
直交 (x, y, z) ジョグとどちらを使うかは **config で切り替える**。

## ユーザー決定 (2026-09-12)

- **実装が楽な方。動けばいい。今動いている状態を壊さない。**
- 上記を満たすため、**「新トピックが来たときだけ動く追加コード」に限定**し、
  既存の直交ジョグ (`cmd_twist`) の経路は変更しない。

### 不採用: 操縦層 (`joy_teleop_node`) で接線速度に変換して `cmd_twist` のまま送る (案 A)

行数は最少だが、`motion_generator_node` が接線速度を 10Hz で直線積分するため
θ ジョグ中に 1 tick あたり r·(θ̇·dt)²/2 だけ r が外へ膨らむ (r=0.6, θ̇=0.5 rad/s で
約 15 mm/rad、0.2 rad/s でも 6 mm/rad)。位置合わせ用途で「動けばいい」を満たさない。
また直交座標の `a_max` レート制限が回転する速度ベクトルを追い切れず弧が崩れる
(r·θ̇² > a_max)。操縦層が指令姿勢とターンテーブル軸を知る必要も生じる。

## 採用: `motion_generator_node` にジョグ入口を 1 本足し、**ジョグの増分だけ極座標で計算する** (案 B 最小形)

状態 `target_` は直交座標のまま (設計判断「ROS2 層の内部は直交座標のまま」は維持)。
1 tick の間だけ (r, θ) に直して積分し、直交座標に戻す。弧が厳密で膨らまない。
上限・加速度・遮断・ウォッチドッグ・作業領域クランプ・同期前の破棄は**すべて既存を共用**する。

---

## 1. 契約

### `sharmech_msgs/PolarTwist` (新設)

```
# 先端の目標速度 (極座標)。原点 = ターンテーブル軸 (turntable_axis_x/y_m)、
# θ は +X から時計回りが正 (UDP mcu_spec.md §3.2、init_pose、mark_pose.py と同じ)。
float64 r_dot       # [m/s]   ターンテーブル軸から遠ざかる向きが正
float64 theta_dot   # [rad/s] 時計回り (上から見て +X → −Y) が正
float64 z_dot       # [m/s]   ベース座標系の z
```

`geometry_msgs/Twist` を流用して `linear.x = ṙ` と読み替える案は却下 (罠になる)。

### トピック `/catchrobo/arm/cmd_polar_twist`

| 項目 | 内容 |
|---|---|
| 型 | `sharmech_msgs/PolarTwist` |
| 向き | 操縦層 → `motion_generator_node` |
| QoS | `cmd_twist` と同じ (reliable, depth 10) |
| 送り方 | `cmd_twist` と同じ。非ゼロの間だけ定期送信、ゼロに戻った直後に 1 発ゼロ、以後は送らない |
| 調停 | `cmd_twist` と **「最後に届いたフレームが有効」**。VR / PS5 は同時に使わない前提のまま |

VR (webxr_controller)・シム (sim_threejs) は**変更不要** (追加のみ。どちらも購読・送信しない)。

---

## 2. `motion_generator_node` の変更 (追加のみ)

### パラメータ

| 名前 | 既定 | 説明 |
|---|---|---|
| `turntable_axis_x_m` / `turntable_axis_y_m` | 0.0 | 極座標の原点。`generate.py` が `robot_geometry.yaml` の `kinematics.turntable_axis_x/y_m` から生成 (`hardware_bridge_node` と同じ値) |

θ 用の速度・加速度上限は**新設しない**。先端速度 √(ṙ² + (r·θ̇)² + ż²) に既存の
`active_jog_v_max_` (= `jog_v_max`、微調整中は `jog_limit` で 0.05) を掛け、
加速度は成分ごと (ṙ、接線速度 r·θ̇、ż) に既存の `a_max` で制限する。

### 内部状態 (追加)

```cpp
enum class JogFrame : uint8_t { kCartesian, kPolar };
JogFrame jog_frame_{JogFrame::kCartesian};   // 最後に届いたジョグのフレーム
struct PolarRate { double r_dot{0.0}, tangential{0.0}, z_dot{0.0}; };  // 接線は [m/s] (= r·θ̇)
PolarRate commanded_polar_{};   // 操縦層から届いた生の値 (接線は受信時の r で m/s に直したもの)
PolarRate current_polar_{};     // レート制限後
double turntable_axis_x_{0.0}, turntable_axis_y_{0.0};
```

### `onCmdPolarTwist` (`onCmdTwist` と対称。`onCmdTwist` 自体は触らない)

```
jog_blocked_ または !synced_with_feedback_ なら return            (既存と同じ理由)
r = hypot(target_.x - ax, target_.y - ay)
tangential = theta_dot * r                                          (r < PolarUtils::kMinRadius なら 0)
speed = sqrt(r_dot² + tangential² + z_dot²)
speed > active_jog_v_max_ なら 3 成分を比で縮める (WARN_THROTTLE、既存と同文)
active = いずれか非ゼロ
goal_priority / exclusive でゴール実行中なら return                (既存と同じ)
jog_frame_ != kPolar なら current_twist_ = {} (直交側の残速度を捨てる); jog_frame_ = kPolar
commanded_polar_ = {r_dot, tangential, z_dot}; last_twist_time_ = now()
active なら: ゴール中なら ABORTED "preempted by jog" (既存と同文); mode_ = kJog
```

`onCmdTwist` 側にも対称の 1 行を足す: `jog_frame_ != kCartesian なら current_polar_ = {}; jog_frame_ = kCartesian`。
(これが既存関数への唯一の変更。直交ジョグしか使わない限り no-op)

### `onControlTimer` の kJog 分岐

既存の直交コードは**そのまま**。`jog_frame_ == kPolar` のときだけ以下に分岐する。

```
1. ウォッチドッグ (既存): 途絶したら commanded_twist_ = {} に加えて commanded_polar_ = {} も
2. レート制限 (既存の approach をそのまま使う):
     current_polar_.r_dot      = approach(current_polar_.r_dot,      commanded_polar_.r_dot,      a_max_*dt)
     current_polar_.tangential = approach(current_polar_.tangential, commanded_polar_.tangential, a_max_*dt)
     current_polar_.z_dot      = approach(current_polar_.z_dot,      commanded_polar_.z_dot,      a_max_*dt)
3. 積分 (kJog, kPolar):
     p = PolarUtils::toPolar(target_.x, target_.y, 0, 0, /*theta_ref=*/0.0, ax, ay)
         (theta_ref は 0 でよい。tick ごとに直交へ戻すので連続化は不要。ワイヤの連続化は hardware_bridge が担当)
     theta_dot = (p.r >= kMinRadius) ? current_polar_.tangential / p.r : 0.0
     r     = p.r + current_polar_.r_dot * dt
     theta = p.theta + theta_dot * dt
     target_.x = PolarUtils::toX(r, theta, ax);  target_.y = PolarUtils::toY(r, theta, ay)
     target_.z += current_polar_.z_dot * dt
     速度 (直交、解析的。k = PolarUtils::kThetaSign):
       target_vel_.x = ṙ·cos(kθ) − r·k·θ̇·sin(kθ)
       target_vel_.y = ṙ·sin(kθ) + r·k·θ̇·cos(kθ)
       target_vel_.z = ż   (pitch/yaw は 0)
     (hardware_bridge の toPolar がこの速度から ṙ, θ̇ を厳密に復元する)
4. クランプ (既存 clampToWorkspace):
     x か y のどちらかがクランプされたら target_vel_.x = target_vel_.y = 0、
       current_polar_.r_dot = current_polar_.tangential = 0
     z がクランプされたら target_vel_.z = 0、current_polar_.z_dot = 0
     target_ = clamped
5. idle 判定: current_polar_ と commanded_polar_ が両方ゼロなら mode_ = kIdle
```

原点特異点 (r < `kMinRadius`): θ̇ を 0 にして r だけ動かせる (既存 `PolarUtils` と同じ扱い)。
負の r は生じない (r_dot で r が 0 を割ると toX/toY が反対側へ出る = 直交では自然に通り抜ける。
問題になったら r 下限のクランプを足す。初版では入れない)。

### 触らないもの

ゴール (`target_pose`)・`jog_limit`・`workspace_clamp`・同期・`enable`・status・`command_pose` の可視化。
`/catchrobo/arm/status` の `mode` は極座標でも `MODE_JOG` のまま (フレームは status に出さない)。

---

## 3. `joy_teleop_node` の変更

### パラメータ (追加)

| 名前 | 既定 | 説明 |
|---|---|---|
| `jog_frame` | `"cartesian"` | `"cartesian"`: 従来どおり `cmd_twist` / `"polar"`: `cmd_polar_twist` を送る |
| `r_axis` / `r_scale` / `r_button_pos` / `r_button_neg` | 1 / 0.10 / -1 / -1 | r [m/s]。既定 = 左スティック縦 (config.yaml の `vx_axis` と同じ) |
| `theta_axis` / `theta_scale` / `theta_button_pos` / `theta_button_neg` | -1 / 0.3 / 7 / 6 | θ [rad/s]。既定 = R2 (7) で時計回り、L2 (6) で反時計回り。**デジタル** (押している間 `theta_scale` 一定)。向きが逆なら `theta_scale` を負に |
| (z は既存の `vz_*` を流用) | | 右スティック縦 (config.yaml で 4) |

既存の `declareDofMapping` / `readDof` をそのまま使う (新しい入力コードは不要)。

L2/R2 の**アナログ軸 (2/5) は初版では使わない**: 静止値が +1.0 で `readDof` がそのまま
全速と解釈するため反転変換が要る。デジタルで足りなければ後から足す。

### デッドマン

**コードでフレームに結びつけない。** 既存の `use_deadman` を config で `false` にする
(`jog_frame: "polar"` の直下に併記)。極座標モードでも L1 を要求したければ `true` に戻せる。

### タイマー (`onPublishTimer`)

`jog_frame_ == polar` のときは `twist` の代わりに `PolarTwist` を組む:
`r_dot = readDof(r_map_)`, `theta_dot = readDof(theta_map_)`, `z_dot = readDof(vz_map_)`。
「非ゼロの間だけ送る + ゼロ 1 発」のロジックは既存 `twist_was_active_` を共用する
(1 tick に送るのはどちらか一方なので共用で問題ない)。
`joy_timeout`・デッドマン (有効時) の扱いは既存と同じ経路。

`jog_frame_ == cartesian` のときの挙動は**現状と同一** (publisher を 1 本余分に作るだけ)。

### `config.yaml`

```yaml
joy_teleop_node:
  ros__parameters:
    jog_frame: "polar"        # "cartesian" (x,y,z) / "polar" (r,θ,z)。2026-09-12 追加
    use_deadman: false        # 極座標ジョグは L1 不要 (ユーザー指定)。直交へ戻すときは true に
    r_axis: 1                 # 左スティック縦 → r
    r_scale: 0.10
    theta_button_pos: 7       # R2 → 時計回り (+θ)
    theta_button_neg: 6       # L2 → 反時計回り
    theta_scale: 0.3          # [rad/s]。r=0.3 で接線 0.09 m/s
    # vz_axis: 4 (既存) → z
```

---

## 4. `params/generate.py`

`motion_generator_node` セクションに `turntable_axis_x_m` / `turntable_axis_y_m` を追加
(`hardware_bridge_node` と同じ 2 行)。`generate.py` → `--check` を通す。

---

## 5. テスト (`test_motion_generator_node.cpp`)

必須 1 本: **`PolarThetaJogKeepsRadius`** — feedback で (0.3, 0.0, 0.15) に同期 →
`cmd_polar_twist(theta_dot=+0.5)` を数百 ms 流す → `command/cartesian` の
`hypot(x, y)` が 0.3 ± 1e-6 のまま、`y` が負に進む (時計回り)、`hardware_bridge` 相当の
`PolarUtils::toPolar` で速度を戻すと `theta_dot ≈ +0.5` (レート制限後の値)。
ハーネスに `publishPolarTwist(r_dot, theta_dot, z_dot)` を足す。

任意 1 本: 既存の直交テストが全て通ること (= 既存経路が無変更である証拠)。
これは新規ではなく `colcon test` を通すだけ。

---

## 6. 更新する文書

| ファイル | 内容 |
|---|---|
| `sharmech_core/docs/motion_generator_node.md` | Subscribe 表に `cmd_polar_twist`、パラメータ表に `turntable_axis_*`、処理フローに kPolar 分岐、「最後に届いたフレームが有効」 |
| `sharmech_core/docs/joy_teleop_node.md` | `jog_frame`・r/θ の割り当て・デッドマンは config で切る旨 |
| `README.md` | トピック一覧に `cmd_polar_twist` (VR/シム未使用) |
| `sharmech_bringup/config/config.yaml` | §3 のとおり |
| `CLAUDE.md` | ノード表の 2 行に一言、「他リポジトリとの整合性確認」に「VR/シムは未対応でよい」を追記 |

---

## 7. やらないこと (初版)

- VR 側の極座標ジョグ (契約は用意されるが WebXR は送らない)
- L2/R2 のアナログ化
- r の下限・上限クランプ (作業領域は直交のまま)
- `MotionStatus` へのフレーム表示
- 極座標ゴール (`target_pose` は直交のまま)
