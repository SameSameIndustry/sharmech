# joy_teleop_node

PS4 コントローラの入力を、操縦層の共通インターフェースへ変換するノード。**実装済み** (`src/joy_teleop_node.cpp`)。

全体アーキテクチャは [`sharmech/README.md`](../../README.md) を参照。

## 役割

`/joy` を購読し、**config で定義された割り当てに従って**以下へ変換する。

| 出力 | 内容 |
|---|---|
| `/catchrobo/arm/cmd_twist` | スティック入力 → 先端の目標速度(ベース座標系) |
| `/catchrobo/arm/gripper` | ボタン → グリッパ開閉(トグル) |
| `/catchrobo/arm/target_pose` | ホームボタン → config で定義した待機姿勢へ復帰 |
| `/catchrobo/arm/cancel` | デッドマンを離したとき → 実行中のゴールを中断 |
| `/catchrobo/game/toggle_manual_control` | 4ボタン同時押し(既定 L1+R1+L3+R3) → 自由操作のトグル(下記) |

PS4 は **VR が使えないときのバックアップと、テスト用**という位置づけ。VR と同じ
`/catchrobo/arm/*` へ publish するので、下流から見ると VR と区別がつかない。

### やらないこと

| やらないこと | 担当 |
|---|---|
| 積分・レート制限・作業領域クランプ | `motion_generator_node` |
| 座標変換 | 不要(最初からベース座標系で出す) |
| 運動学 | MCU / `kinematics_node` |

## インターフェース

### Subscribe

| トピック | 型 |
|---|---|
| `/joy` | `sensor_msgs/Joy` |

### Publish

| トピック | 型 | 周期 |
|---|---|---|
| `/catchrobo/arm/cmd_twist` | `geometry_msgs/Twist` | `publish_rate` で定期送信 |
| `/catchrobo/arm/gripper` | `std_msgs/Bool` | `publish_rate` で定期送信 |
| `/catchrobo/arm/target_pose` | `geometry_msgs/PoseStamped` | ホームボタンの立ち上がりエッジ |
| `/catchrobo/arm/cancel` | `std_msgs/Empty` | デッドマンの立ち下がり |
| `/catchrobo/game/toggle_manual_control` | `std_msgs/Empty` | 4ボタン同時押しの立ち上がりエッジ(下記) |

### Subscribe (状態確認用・任意)

| トピック | 型 | 用途 |
|---|---|---|
| `/catchrobo/arm/status` | `sharmech_msgs/MotionStatus` | ホーム復帰が却下されたことをログに出す |

**Action は使わない。** ゴールもキャンセルもトピックで送る
(理由は [`motion_generator_node.md`](motion_generator_node.md#action-を使わない) を参照)。

## ★ 実装上の最大の罠: `/joy` は変化時にしか流れない

ROS2 の `joy` ノードは既定で**状態が変化したときだけ** publish する。スティックを一定量
倒したまま保持すると `/joy` が止まる。

`/joy` 受信時にだけ Twist を出す作りにすると、**スティックを倒し続けているのに
`motion_generator_node` のウォッチドッグ(400ms)が作動して勝手に停止する。**

したがって、

- 最新の `/joy` 状態を**保持**しておく
- **自前のタイマー**(`publish_rate`)で Twist を定期送信する

`joy` ノード側の `autorepeat_rate` に依存させる方法もあるが、外部ノードの設定に安全性を
預けることになるため採らない。

## ★ コントローラ切断時の暴走を防ぐ

上のタイマー方式には裏の危険がある。**コントローラが切断されると `/joy` が止まるが、
タイマーは回り続けるため、最後のスティック値を送り続けて暴走する。**

`joy_timeout` の間 `/joy` が届かなければ、**全入力をニュートラル(軸 0・ボタン非押下)
として扱う。**

## パラメータ

### 動作設定

| パラメータ | 既定値 | 説明 |
|---|---|---|
| `publish_rate` | 50.0 | Twist / グリッパの送信周期 [Hz] |
| `joy_timeout` | 0.5 | この時間 `/joy` が無ければ全入力をニュートラル扱い [s] |
| `deadzone` | 0.15 | スティックのデッドゾーン。**必須**(ドリフトで微速動作し続けるのを防ぐ) |
| `use_deadman` | `true` | デッドマンスイッチを使うか |
| `home_pose` | — | ホーム姿勢 `[x, y, z, pitch, yaw]` |

### 軸・ボタン割り当て

**すべて config。コードに番号を埋めない。** PS4 の軸・ボタン番号は**ドライバ依存**で、
ROS2 Humble の `joy` (SDL2 ベース) と `joy_linux` でも異なる。

各自由度は「軸」と「ボタン対」の**両方から駆動できる**(合算する)。使わない側は `-1` を指定。
スティックでも十字キーでもトリガでも同じ枠組みで扱える。

| パラメータ | 既定値 | 対応 |
|---|---|---|
| `vx_axis` / `vx_scale` | 1 / 0.10 | 左スティック 縦 → 前後 [m/s] |
| `vy_axis` / `vy_scale` | 0 / 0.10 | 左スティック 横 → 左右 [m/s] |
| `vz_axis` / `vz_scale` | 7 / 0.10 | 十字キー 縦 → 上下 [m/s] |
| `pitch_axis` / `pitch_scale` | 4 / 0.50 | 右スティック 縦 → pitch [rad/s] |
| `yaw_axis` / `yaw_scale` | 3 / 0.50 | 右スティック 横 → yaw [rad/s] |
| `<dof>_button_pos` / `<dof>_button_neg` | -1 | 軸の代わりにボタン対で駆動する場合 |
| `deadman_button` | 4 | L1 |
| `gripper_toggle_button` | 0 | × |
| `home_button` | 2 | △ |
| `manual_toggle_button_l1` | 4 | L1 (自由操作トグルの4ボタン同時押しの1つ。`deadman_button` と同じでよい) |
| `manual_toggle_button_r1` | 5 | R1 |
| `manual_toggle_button_l_stick` | 11 | L3 (左スティック押し込み) |
| `manual_toggle_button_r_stick` | 12 | R3 (右スティック押し込み) |
| `confirm_button` | 12 | R3。微調整の確定 (下記「微調整の確定ボタン」) |

反転は `scale` を負値にすることで表現する(反転フラグは持たない)。

> 既定値は目安。**実機で `ros2 topic echo /joy` を見て確認すること。**

`*_scale` はスティックを最大まで倒したときの速度。`motion_generator_node` の `v_max` を
超える値を入れても、上流で頭打ちになるだけで意味がない。

## 内部状態

| 状態 | 説明 |
|---|---|
| `last_joy_` | 最新の `/joy` メッセージ |
| `last_joy_time_` | `joy_timeout` 判定用 |
| `gripper_state_` | トグルで反転する状態。**起動時は `false`(開)** |
| `prev_buttons_` | 立ち上がりエッジ検出用の前回ボタン状態 |
| `manual_toggle_combo_was_active_` | 自由操作トグルの4ボタン同時押しが前回tickで揃っていたか。連打防止のエッジ検出用 |

## 処理フロー

### `/joy` 受信時

```
last_joy_      ← msg
last_joy_time_ ← now

エッジ検出(前回状態と比較):
    gripper_toggle_button の立ち上がり → gripper_state_ を反転
    home_button の立ち上がり           → /catchrobo/arm/target_pose に home_pose を publish
    deadman_button の立ち下がり        → /catchrobo/arm/cancel を publish

prev_buttons_ ← msg.buttons
```

### タイマー(`publish_rate`)

```
1. 入力の有効性判定
   if now - last_joy_time_ > joy_timeout:
       全入力をニュートラルとして扱う(警告ログ、スロットリング付き)

2. デッドマン判定
   if use_deadman かつ deadman_button が押されていない:
       twist ← 0

3. それ以外:
   各自由度について
       raw   ← axes[axis] (+ ボタン対の寄与)
       |raw| < deadzone なら 0
       twist ← raw * scale

4. publish
   /catchrobo/arm/cmd_twist ← twist
   /catchrobo/arm/gripper   ← gripper_state_
```

### デッドマンを離したときは「送信停止」ではなく「ゼロを送る」

送信を止めると `motion_generator_node` のウォッチドッグ(400ms)を待ってから減速が始まる。
**ゼロを明示的に送れば即座に減速が始まる。** 応答性が違う。

同じ理由で、`joy_timeout` 発動時も publish は止めずゼロを送り続ける。

### ゼロ Twist を送り続けてもゴールは妨げない

このノードはスティックがニュートラルでも**ゼロの Twist を送り続ける**。

`motion_generator_node` 側は「**ゼロでない Twist のみがゴールを abort する**」仕様なので、
PS4 を繋いだままでもゴール指定(ホーム復帰や VR からの指示)は正常に動く。
上流の仕様が変わるとここが壊れるので、両方のドキュメントに書いてある。

### グリッパは毎周期 publish する

状態量なので冪等であり、毎回送っても害がない。変化時のみ送る方式だと、
後から起動したノードが現在状態を知る手段がなくなる。

## ホーム復帰

`home_button` の**立ち上がりエッジ**で `home_pose` へのゴールを発行する。

| 状況 | 挙動 |
|---|---|
| ジョグ動作中(スティックを倒している) | `motion_generator_node` が却下する。`/catchrobo/arm/status` の `last_result` で分かるので警告ログを出す |
| ゴール実行中にスティックを倒した | `motion_generator_node` が abort し、ジョグへ移行(仕様通り) |
| ゴール実行中にデッドマンを離した | `/catchrobo/arm/cancel` を publish する |

最後の項目は、デッドマンが「操作者が能動的に指令している」ことを表すため。
自動移動中にデッドマンを離したら止まる方が一貫する。

## 自由操作トグル (VRが使えない場合の脱出ハッチ)

**最悪VRが動かせない場合でも、DualSense(PS4互換)コントローラだけで最低限
試合を進められるようにする**ための機能。4ボタン(既定: L1+R1+L3+R3)を
**すべて同時に**押した瞬間(立ち上がりエッジ)に `/catchrobo/game/toggle_manual_control`
(`std_msgs/Empty`) を1回だけ publish する。

- 個々のボタンの立ち上がりエッジではなく、**4つ揃った状態そのもの**の立ち上がりで
  判定する(`manual_toggle_combo_was_active_` で前回tickの状態を保持)。
  誤って連打しないよう、離して押し直すまでは再送しない
- 受け手は `game_state_manager_node`。どのゲームステートからでも
  `GameState::kManualControl` にトグルし、再度同じコンボを押すと元の状態へ戻る
  (詳細・状態遷移としての扱いは
  [`game_state_manager_node.md`](game_state_manager_node.md#自由操作-vrが使えない場合の脱出ハッチ))
- **このノード自身の動作(cmd_twist ジョグ)は自由操作かどうかに関わらず常に有効。**
  自由操作状態は「自動シーケンスを止めて、ジョグ操作の邪魔をさせない」ためのもので、
  ジョグ自体はこのトグルの前後で変わらない

L1 は `deadman_button` と兼用してよい(コンボの一部として押されている間も、
デッドマンとして同時に機能する)。

## 異常系

| 事象 | 挙動 |
|---|---|
| `joy_timeout` の間 `/joy` が来ない(切断) | 全入力ニュートラル。ゼロ Twist を送り続ける。警告ログ |
| 設定した `axis` / `button` の番号が `/joy` の配列長を超える | **起動時とメッセージ受信時に検証**。該当入力を 0 として扱い、エラーログを1度だけ出す |
| `deadzone` がスティックの中心ずれより小さい | 手を離しても微速で動き続ける。実機で `/joy` を見て調整する |
| ホームゴールが却下された | `/catchrobo/arm/status` を見て警告ログを出す。リトライしない |

配列長の検証は必須。**`axes[7]` を持たないドライバで既定値のまま起動すると範囲外アクセスで落ちる。**

## 設計判断

### ボタン割り当てを完全に config 化する

軸・ボタン番号はドライバとコントローラの組み合わせで変わる。コードに埋めると、
その都度リビルドが必要になる。**このノードの本質は「割り当て表」であり、
その表が設定ファイルにあるのが自然。**

### 反転フラグを持たず `scale` の符号で表現する

パラメータが半分になり、「反転かつ負のスケール」のような二重否定が起きない。

### 軸とボタン対の両方を受け付ける

十字キーは軸として現れるドライバとボタンとして現れるドライバがある。トリガ (L2/R2) も
軸だったりボタンだったりする。**どちらでも同じ枠組みで割り当てられる**ようにしておけば、
ドライバが変わっても config だけで吸収できる。

### VR と同じトピックへ publish する

`motion_generator_node` から見て VR と PS4 の区別がつかない。入力源を増やしても
下流に影響しない。

## 未決定事項

| 項目 | 内容 |
|---|---|
| `home_pose` の具体値 | 機構の確定待ち |
| 軸・ボタン番号の既定値 | 実機の `ros2 topic echo /joy` で要確認 |
| 速度スケールの切り替え | 低速モード / 高速モードをボタンで切り替えたいか |

## 微調整の確定ボタン

`game_state_manager_node` の微調整待ち (`ADJUSTING_PICK` / `ADJUSTING_PLACE`) を
抜けるための「これでよい」を `/catchrobo/game/confirm` (`std_msgs/Empty`) へ publish する。
立ち上がりエッジで1回だけ送る。VR 側はサムズアップで同じトピックへ送る契約
(詳細は [`game_state_manager_node.md`](game_state_manager_node.md) の「手動微調整」)。

**既定の R3 (12) は自由操作トグルの4ボタン同時押し (L1+R1+L3+R3) にも含まれている。**
そのため「L1 / R1 / L3 のいずれかが押されている間は確定を出さない」というガードを
入れてある。コンボを組むつもりで R3 を先に押しても誤確定しないようにするためで、
逆に言えば **R3 を単独で押したときだけ確定になる**。

コンボと無関係のボタン (例: ○ = 1) を `confirm_button` に割り当てれば、この
ガードは実質無効になり素直な単独押し判定になる。実機のボタン割り当てが決まったら
そちらへ移すことを検討してよい。
