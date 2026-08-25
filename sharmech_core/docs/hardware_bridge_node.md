# hardware_bridge_node

MCU へ流す UDP データを整形し、MCU からの状態を ROS2 へ戻すノード。**実装済み** (`src/hardware_bridge_node.cpp`)。

全体アーキテクチャと UDP プロトコル仕様は [`sharmech/README.md`](../../README.md) を参照。

## 役割

ROS2 トピックと UDP パケットの間の**変換と輸送のみ**を担う。

| 方向 | 処理 |
|---|---|
| 送信 | Cartesian 指令(または関節指令)+ グリッパ状態を UDP パケットに整形して送る |
| 受信 | MCU から実状態を受け取り、`/catchrobo/arm/current_pose` と `/joint_states` へ publish |

送信するパケットの種類は **config で切り替える**。

| `command_mode` | 購読するトピック | `packet_type` | 状態 |
|---|---|---|---|
| `cartesian` | `/catchrobo/command/cartesian` | 1 | パターンA(現在) |
| `joint` | `/catchrobo/command/joint` | 2 | パターンB(将来) |

### やらないこと

**このノードは判断をしない。** 何かを「決める」処理が入ったらここではない。

| やらないこと | 担当 |
|---|---|
| 補間 | **MCU**(通信路の下流でなければ意味がない) |
| レート制限・作業領域クランプ・ウォッチドッグ | `motion_generator_node` |
| 逆運動学 / 順運動学 | MCU (パターンA) / `kinematics_node` (パターンB) |
| 到達判定・状態遷移 | `motion_generator_node` |

## インターフェース

### Subscribe

| トピック | 型 | 条件 |
|---|---|---|
| `/catchrobo/command/cartesian` | `sharmech_msgs/CartesianCommand` | `command_mode == cartesian` |
| `/catchrobo/command/joint` | 型未定 | `command_mode == joint`(**将来**) |
| `/catchrobo/command/gripper` | `std_msgs/Bool` | 常時 |

**購読するのはどちらか一方のみ。** メッセージ型が異なるため、起動時に config を見て
対応する購読とエンコーダの組を生成する。

### Publish

| トピック | 型 | 説明 |
|---|---|---|
| `/catchrobo/arm/current_pose` | `geometry_msgs/PoseStamped` | MCU が FK して返した実姿勢 |
| `/joint_states` | `sensor_msgs/JointState` | 実測の関節角。`robot_state_publisher` 経由で rviz に表示できる |

### パラメータ

| パラメータ | 既定値 | 説明 |
|---|---|---|
| `command_mode` | `cartesian` | `cartesian` / `joint` |
| `mcu_ip` | `192.168.1.50` | MCU の IP アドレス |
| `mcu_port` | 8888 | 送信先ポート |
| `local_port` | 8889 | 受信待ち受けポート |
| `feedback_poll_rate` | 200.0 | 受信ポーリング周期 [Hz] |
| `feedback_timeout` | 0.5 | この時間フィードバックが無ければ警告 [s] |
| `protocol_version` | 1 | 送出するプロトコル版。受信時の検証にも使う |
| `joint_names` | `[]` | `/joint_states` に載せる関節名。順序は MCU の返す配列と一致させる |

## 処理フロー

### 送信: **サブスクリプション駆動**(タイマーではない)

`/catchrobo/command/cartesian` を受信した瞬間にパケットを組んで送る。上流が 100Hz で publish するので、
UDP も 100Hz になる。

```
onCartesianCommand(msg):
    packet ← encoder_->encode(msg, gripper_state_, send_seq_++)
    sendto(sockfd_, packet)
```

**タイマー方式にしない理由**

1. **時計が1つで済む。** タイマー方式だと上流の publish 周期と送信周期の2つが存在し、
   ずれると重複送信や取りこぼしが起きる
2. **上流の停止がそのまま安全側に倒れる。** `motion_generator_node` が落ちれば送信も止まり、
   MCU のウォッチドッグが作動して停止する。タイマー方式だと上流が死んでも最後の指令を
   送り続け、異常が隠れる

### グリッパは「同期」ではなく「ラッチ」

`/catchrobo/command/gripper` は `/catchrobo/command/cartesian` とは別トピックなので同時には届かない。
**最新値を保持しておき、Cartesian 指令が来たときに一緒にパケットへ詰める。**

グリッパは状態量で変化も低頻度なので、これで問題ない(`Empty` ではなく `Bool` を
選んだのがここで効く。イベント型だと現在状態を保持できない)。

### 受信: タイマーでポーリング

`feedback_poll_rate` のタイマーで、ソケットに溜まっているデータグラムを**すべて読み切る**。

```
onFeedbackTimer():
    while recvfrom(MSG_DONTWAIT) が成功する限り:
        検証(下記) → 失敗なら破棄して次へ
        最新のパケットとして保持
    最新パケットがあれば:
        /catchrobo/arm/current_pose ← Cartesian姿勢
        /joint_states       ← 関節角
        last_feedback_time_ ← now
```

ブロッキング recv 用のスレッドを立てる方式もあるが、コンポーネント内でのスレッド安全性を
考えなくてよいポーリング方式を既定とする。

## UDP パケット: MCU → ROS2 (`packet_type = 0x81`)

ヘッダは送信と共通(16 バイト)。方向の区別を明確にするため、**フィードバックは `0x80` 以上**を使う。

| offset | 型 | 名前 | 説明 |
|---|---|---|---|
| 0 | `uint8` | `protocol_version` | 現在 1 |
| 1 | `uint8` | `packet_type` | `0x81` = 状態フィードバック |
| 2 | `uint16` | `payload_length` | `28 + 4 × joint_count` |
| 4 | `uint32` | `seq` | フィードバック自身の連番 |
| 8 | `uint64` | `timestamp_us` | MCU 側の送信時刻 [μs] |
| 16 | `uint32` | `seq_echo` | **最後に受信した指令の `seq`**。往復遅延の測定に使う |
| 20 | `float32` | `x` | 実位置 [m] |
| 24 | `float32` | `y` | 実位置 [m] |
| 28 | `float32` | `z` | 実位置 [m] |
| 32 | `float32` | `pitch` | 実姿勢 [rad] |
| 36 | `float32` | `yaw` | 実姿勢 [rad] |
| 40 | `uint16` | `status_flags` | 下記 |
| 42 | `uint8` | `gripper_state` | 実際のグリッパ状態 |
| 43 | `uint8` | `joint_count` | 関節数 N |
| 44 | `float32[N]` | `joint_positions` | 実測の関節角 [rad] |

関節数 6 なら **合計 68 バイト**。`joint_count` を可変にしてあるので、将来モータ構成が
変わってもプロトコルを変えずに済む。

### `status_flags`

| bit | 意味 |
|---|---|
| 0 | 追従誤差過大 |
| 1 | ドライバ異常 |
| 2 | ウォッチドッグ作動中(指令途絶により停止した) |
| 3 | 未初期化 / 原点未確定 |
| 4-15 | 予約 |

### MCU 側が FK を行う

到達判定と VR への表示に必要なのは Cartesian 姿勢だが、MCU が持っているのはエンコーダ由来の関節角。
**パターンA では MCU が既に運動学(IK)を持っているので、FK も MCU 側で行う。**

こうすることで「ROS2 は運動学を持たない」という原則が保たれ、ROS2 は受け取って publish する
だけで済む。関節角も併せて返すのは診断と rviz 表示のため。

## 内部状態

| 状態 | 説明 |
|---|---|
| `sockfd_` | UDP ソケット。送信と受信で共用 |
| `gripper_state_` | ラッチしたグリッパ状態 |
| `send_seq_` | 送信パケットの連番。送るたびに +1 |
| `last_recv_seq_` | 受信済みの最大 `seq`。順序逆転の検出に使う |
| `last_feedback_time_` | フィードバック途絶の検出に使う |
| `encoder_` | `command_mode` に応じて生成されたエンコーダ |

## 異常系

| 事象 | 挙動 |
|---|---|
| `protocol_version` が不一致 | 破棄し、警告ログ(スロットリング付き) |
| `packet_type` が未知 | 破棄し、警告ログ |
| `payload_length` が実サイズと不一致 | 破棄し、警告ログ |
| 受信 `seq` が `last_recv_seq_` 以下 | **破棄**。UDP は順序を保証しないため古いパケットが後から届く |
| `feedback_timeout` の間フィードバックが無い | 警告ログ。**publish はしない**(下記) |
| `sendto` 失敗 | 警告ログ(スロットリング付き)。送信は継続する |
| MCU 未実装でフィードバックが一切無い | `/catchrobo/arm/current_pose` を **publish しない** |

### フィードバックが無いときにエコーしない

送信した目標姿勢を「現在姿勢」として publish すると、**実フィードバックがあるかのように誤認する**。
特に到達判定に使われると、指令した瞬間に到達したことになってしまう。

publish しない方が、下流(`motion_generator_node`)が「フィードバックが無い」ことを
正しく認識できる。`motion_generator_node` 側は `/catchrobo/arm/current_pose` が来ない場合の
挙動を規定済み。

## 設計判断

全体に関わる判断は [`sharmech/README.md`](../../README.md) を参照。ここでは本ノード固有のものを記す。

### パケット組み立てをエンコーダとして抽象化する

```
PacketEncoder (抽象)
  ├─ CartesianPacketEncoder   packet_type = 1   ← 今回実装
  └─ JointPacketEncoder       packet_type = 2   ← 将来追加
```

ノード本体に直書きすると、パターンB 追加時に本体を改造することになる。抽象化しておけば
**実装クラスを1つ足して config の分岐に1行加えるだけ**で済む。

### `control_flags` の動作許可ビットは常に 1

パケットにフィールドは確保するが、**現時点では制御経路を作らず常に 1 を入れる**。
サーボ ON/OFF が必要になった時点で、サービスなり topic なりを追加する。

### CRC は入れない

有線 Ethernet の CRC32 と UDP チェックサムによる検出に加えて独自 CRC を足すことも検討したが、
**シンプルな実装を優先し、今は入れない。** `payload_length` の不一致検出と `seq` の順序検証で
当面は足りると判断した。必要になれば 4 バイト追加できる。

### 有線 Ethernet でも UDP を使う

TCP は再送とヘッドオブラインブロッキングがあり、**古いデータの再送を待つ間に新しい指令が
届かなくなる**。リアルタイム制御では「古いデータは捨てて新しいものを使う」のが正しい。

## テスト

実機 (MCU) が無い間は `sharmech_core/scripts/mock_mcu.py` を UDP の送信先にして手元で
動作確認できる。詳細は `CLAUDE.md` の「テスト」節を参照。

## 未決定事項

| 項目 | 内容 |
|---|---|
| `joint_names` の具体値と順序 | 関節構成の確定待ち。MCU が返す `joint_positions` の並びと一致させる必要がある |
| `/catchrobo/command/joint` の型 | パターンB 実装時に決める |
| フィードバック途絶時の停止 | 現状は警告のみ。自動で停止させるべきかは要検討 |
| 動作許可の制御経路 | サーボ ON/OFF が必要になった時点で追加 |
