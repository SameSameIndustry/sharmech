# MCU (マイコン) ファームウェア仕様書

**対象読者: `M5_Lower-Controller` (マイコン側ファームウェア) の担当者。**

この文書は ROS2 側 (sharmech) とマイコンの間の**契約の正本**である。
パケットのバイトレイアウト・タイミング・異常系の振る舞いはここに書かれている通りに
実装すること。設計の背景・理由づけの詳細は [`sharmech/README.md`](../README.md) の
「MCU通信仕様 (UDP)」節にあるが、両者に食い違いがあればこの文書が優先する。

最終更新: 2026-09-11 (**初期位置が ROS2 側へ移り、`control_flags` bit2 と `status_flags` bit5 は
予約になった** (§3.2/§3.5/§4.6/§10)。ROS2 は bit2 を常に 0 で送り bit5 を見ないので、
マイコンは電源投入後の原点出しとホールドだけを担当すればよい。同日、**θ の正方向を時計回りに変更**
(§3.2/§3.4/§10。ROS2 側で反転する。MCU 側の `THETA_DEG_PER_RAD_SIGN` は「時計回りに対して
一致するか」の意味になる)。同日、**指令 (0x01)・フィードバック (0x81) の周期を 100Hz → 10Hz、
ウォッチドッグを 50ms → 500ms に変更**。パケットレイアウトは不変で `protocol_version` = 2 のまま)

---

## 1. 全体像とマイコンの担当範囲

```
操縦層 (VR / PS4)
   ▼
ROS2層 (別PC。軌道生成・調停・作業領域クランプ・ウォッチドッグ)
   │  有線Ethernet / UDP / 10Hz (指令・フィードバックとも)
   ▼
★マイコン (この仕様書の対象)
   │  ・UDP受信 (目標位置+速度のストリーム。xy平面は極座標 r-θ)
   │  ・自分の制御周期での補間
   │  ・IK (逆運動学) → 各モータへの指令
   │    (θ はターンテーブル、r は肩、z は肘/膝。xy平面のIKは不要)
   │  ・グリッパ開閉・「縦にする」動作
   │  ・FK (順運動学) → フィードバックのUDP送信
   ▼
モータドライバ / モータ / グリッパ
```

マイコンは 2 枚構成で、上図の担当は r/z 基板 (肩・肘/膝) と θ 基板 (ターンテーブル・グリッパ・
縦にする) に分かれる (§2.2)。**現行ファーム上での実装のしかたは §10 (基板別の実装仕様)。**

**マイコンがやらなくてよいこと** (ROS2側の責務なので実装しないこと):

| やらないこと | 理由 |
|---|---|
| 軌道生成 (台形速度プロファイル等) | ROS2側が10Hzのストリームとして送ってくる。マイコンは「今この瞬間の目標」に追従するだけ |
| 複数入力 (VR/PS4/自動シーケンス) の調停 | ROS2側で1本のストリームに合流済み |
| 座標変換・TF | 受信する値は最初からロボットベース座標系 (後述) |
| **xy平面の直交⇄極座標変換 (atan2/hypot)** | **ROS2側が送信直前に済ませて `r`,`θ` で送る。マイコンは受け取った `θ` をそのままターンテーブルに、`r` をそのまま肩機構に渡せばよい** |
| CRC 検証 | UDPチェックサムに委ねる (入れない、と決定済み) |

---

## 2. 物理・ネットワーク

- **接続: 有線 Ethernet。** プロトコルは UDP (TCPは使わない)
- ROS2 → マイコン: マイコンの **UDP 8888番** に指令が届く
- マイコン → ROS2: ROS2側PCの **UDP 8889番** へフィードバックを送る。
  **フィードバックはマイコン起動直後から自発的に送り始める**(後述) ため、
  ROS2側PCの IPアドレス:ポート はマイコン側の設定として静的に持つこと
  (最初の受信パケットの送信元から学習する方式にしない)
- IPアドレスは現場で確定する (ROS2側configの現在値はマイコン=192.168.1.100。
  `STM32_UDP2CAN_controller` の `main.c` `gWIZNETINFO.ip` と同じ値に揃えてある)

### 2.2 マイコンは 2 枚構成 (2026-09-10 ユーザー確定)

基板は 2 枚あり、**基板同士は通信できない** (デイジーチェーンではない)。
ROS2 側は **同じ指令パケット (0x01) を両方の基板へ送る**。各基板は受信したパケットの
うち自分の担当フィールドだけを使い、**自分の担当フィールドだけを埋めた 0x81 を
ROS2 側 PC の同じ 8889 番へ返す**。合成は ROS2 側 (`hardware_bridge_node`) が行う。

| 基板 | IP (現物) | ファーム | 担当 (指令で使うもの) | 0x81 で埋めるもの |
|---|---|---|---|---|
| r/z 基板 | 192.168.1.100 | `ip_100_section` ブランチ | `r`, `z` (肩・肘/膝の GIM ×4)、`control_flags` bit0 | `r`, `z`, `joint_positions` の肩・肘/膝 (index 0,1,3,4)、`status_flags` |
| θ 基板 | 192.168.1.101 | `enndeffector_UDP` ブランチ | `theta` (ターンテーブル)、`gripper`、`control_flags` bit0/bit1 | `theta`, `joint_positions[2]` (turntable)、`pitch`, `yaw`, `gripper_state`、`status_flags` |

- 担当外のフィールドは **0 で送る** (ROS2 側は読まない)。`joint_count` は両基板とも 5
- `status_flags` は基板ごとに自分の状態を立てる。ROS2 側で **bit0〜4 は OR、bit5 は AND**
  で合成する。**bit3 (未初期化) は片方でも立っていれば ROS2 は動作許可を出さない**ので、
  原点出しが終わるまで確実に立てること。
  (2026-09-11 に bit2/bit5 が予約になり、「bit5 は両基板が実装すること」という要求は
  無くなった。合成規則自体は AND のまま残してある)
- `seq` は基板ごとに独立でよい (ROS2 側は基板ごとに順序逆転を判定する)。**再起動で 0 から
  振り直してよい** (ROS2 側は途絶後の再開を再起動とみなして基準を捨てる)
- 2 枚とも 10Hz で自発送信する (§3.4 の送信条件は基板ごとに適用)
- **バイトオーダはリトルエンディアン、構造体はパディングなし** (STM32 (Cortex-M) /
  x86 ともリトルエンディアンなので変換不要)。実数は全て float32
  (FPU付きSTM32ならネイティブ)

### 2.1 負荷見積もり (STM32G431 + W5500 前提。2026-09-01確認)

想定ハードウェア (マイコン=STM32G431, Ethernet=W5500/SPI) に対する通信量:

| 項目 | 値 | 対キャパシティ |
|---|---|---|
| 指令受信 60B×10Hz + FB送信 64B×10Hz (フレーミング込み) | ≈17 kbps | 100BASE-TX の **0.02%** |
| W5500 SPIバス転送 (レジスタ操作込みで両方向 ~300B/10ms) | ≈240 kbps | SPI 10MHz でも **2.4%**、20MHz以上なら1%未満 |
| W5500 受信バッファ (既定2KB/ソケット) | 受信6.8KB/s | **約300ms** 読み出しが止まっても溢れない |

**データ量・周期の観点では問題にならない** (どこを見ても2〜3桁の余裕)。
注意が要るのは量ではなく作り方の2点のみ:

1. W5500 と同じSPIバスに他デバイス(モータドライバ・エンコーダ等)を同居させる
   場合はバス競合に注意 (専用バス推奨、または DMA + 優先度設計)
2. 1kHz制御ループの中でブロッキングなSPI/ネットワークI/Oを呼ばない
   (ジッタ源になる。W5500のINTnピン割り込みか低優先度タスクへ分離する)

---

## 3. パケット仕様

### 3.1 共通ヘッダ (16バイト。全パケット共通)

| offset | 型 | 名前 | 説明 |
|---|---|---|---|
| 0 | `uint8` | `protocol_version` | 現在 **2** (2026-09-08にxy平面を極座標へ変更。1とは非互換)。不一致のパケットは破棄すること |
| 1 | `uint8` | `packet_type` | 0x01=極座標指令, 0x02=関節指令, 0x81=フィードバック |
| 2 | `uint16` | `payload_length` | ペイロード長 [byte]。不一致は破棄 |
| 4 | `uint32` | `seq` | 送信ごとに+1される連番 |
| 8 | `uint64` | `timestamp_us` | 送信時刻 [μs]。**診断用。マイコンは無視してよい** (時刻同期は要求しない。補間に使う時間は自分のクロックで測ること) |

### 3.2 指令 `packet_type = 0x01` (極座標。パターンA・通常運用)

ペイロード44バイト、パケット全体60バイト。**10Hz (100ms 間隔) で届き続ける** (アイドル中も
速度0で届く。途絶=ROS2側の停止・切断を意味する)。2026-09-11 に 100Hz から変更
(ROS2 側 `config.yaml` の `motion_generator_node.control_rate`)。

**xy平面は極座標 (r, θ) で届く (2026-09-08変更。それ以前は `x`,`y`)。**
この機構は xy 平面が「ターンテーブルで θ、肩の対称二軸駆動で r」の r-θ 型なので、
直交座標で渡すとマイコンが毎周期 atan2/hypot をやり直すことになる。
`z` は肘/膝機構が直接与えるため変換の対象外。バイト数・オフセット・
`gripper`/`control_flags` の位置は変更前と同じ。

| offset | 型 | 名前 | 単位 |
|---|---|---|---|
| 16 | `float32` | `r` | m (ターンテーブル回転軸からの水平距離) |
| 20 | `float32` | `theta` | rad (**+X軸から時計回り (+X → −Y) が正**。2026-09-11 に反時計回りから変更。**±π を超えうる連続値**。下記) |
| 24 | `float32` | `z` | m |
| 28 | `float32` | `pitch` | rad (※末尾「未確定事項」参照) |
| 32 | `float32` | `yaw` | rad (※同上) |
| 36 | `float32` | `r_dot` | m/s (r の時間微分) |
| 40 | `float32` | `theta_dot` | rad/s (θ の時間微分) |
| 44 | `float32` | `vz` | m/s |
| 48 | `float32` | `pitch_rate` | rad/s |
| 52 | `float32` | `yaw_rate` | rad/s |
| 56 | `uint8` | `gripper` | 0=開, 1=閉 |
| 57 | `uint8` | `control_flags` | bit0=動作許可, bit1=縦にする指示, bit2=**予約 (ROS2 は常に 0 を送る)** |
| 58 | `uint16` | `reserved` | 0埋め。無視すること |

- **`theta` の正方向は上から見て時計回り (+X → −Y)。** ベース座標系 (右手系・z 上向き) の
  数学的な正方向 (反時計回り) とは逆で、ROS2 側が atan2 の符号を反転して送る
  (`polar_utils.hpp` の `kThetaSign`)。実機のターンテーブルが atan2 そのままだと逆に回った
  ため、2026-09-11 に ROS2 側で反転する方針とした。`theta_dot` も同じ向き。
  フィードバック (§3.4) の `theta` も同じ向きで返すこと
- **`theta` は `(-π, π]` に丸められていない。** 作業領域は X が -2.045〜+0.981 m
  なので -X 軸 (θ = ±π) を実際にまたぐ。atan2 の生値をそのまま送ると
  シューティングボックスの上で θ が +3.14 → -3.14 と飛び、ターンテーブルが
  1回転逆走する。ROS2側は直前に送った値の近傍へアンラップした連続値を送るので、
  **マイコン側で ±π に正規化し直さないこと** (せっかくの連続性が壊れる)。
  θ が可動域を超えたら §4.1 の可動域外破棄で弾いてよい
- 位置 (r,θ,z,pitch,yaw) と速度 (r_dot..yaw_rate) は**常に両方入っている**。
  使い方は §4.2 (補間) を参照。**極座標の速度なので、直交座標の速度とは違い
  `theta_dot` は原点に近いほど大きくなる** (θ̇ = −(x·vy − y·vx)/r²。符号は時計回り正)
- `gripper`・`control_flags` は**毎パケットに載る「状態 (レベル)」であって
  イベント (エッジ) ではない**。パケットを取りこぼしても次のパケットで正しい状態に
  回復する。「開→閉に変わった瞬間」が要る場合はマイコン側で前回値と比較する
- `control_flags` bit1 (縦にする指示): これが立っている間、横倒しのワーク
  (チップスター缶) を縦向きにしてから置く動作を行う。ROS2側は置く直前にのみ立てる。
  **どんな機構・アクチュエータで実現するかはマイコン側の設計判断**。
  **必ずスルーレート制限をかけて回すこと** — bit1は位置指令と違って速度を持たない
  ステップ入力なので、素直に実装すると全力で回る。缶を掴んだまま急激に回すと
  放り投げてワーク破損 (競技で-1点、5回で競技終了) になる。
  なおROS2側は、この回転が完了するまでの時間を `orient_dwell_sec` (既定0.5s) の
  固定待ちとして見込んでアームを静止させている。**実機の回転がこれより遅い場合は
  申告してほしい** (ROS2側のパラメータを合わせる)
- `control_flags` bit0 (動作許可) が 0 のパケットを受けたら、ウォッチドッグ発動時と
  同じ挙動 (**現在位置をホールド**。r/θ/z と速度は無視する) を取ること。
  **ROS2 は起動直後、マイコンのフィードバックに目標姿勢を同期し終えるまで 0 を送る**
  (§4.6)。同期前の目標は原点 (r=0, z=0) の仮値なので、これに追従すると ROS2 を
  起動しただけでアームが動いてしまう。bit0=0 でもグリッパと bit1 は反映してよい
- `control_flags` bit2 (0x04): **予約 (2026-09-11〜。ROS2 は常に 0 を送る)。**
  2026-09-10 の版では「初期位置要求」で、立っている間はマイコン側で定義した初期関節角へ
  移動して到達を `status_flags` bit5 で返す契約だったが、**初期位置の正本が ROS2 側
  (`params/robot_geometry.yaml` の `init_pose`) へ移った**ため廃止した。
  初期位置へは ROS2 が**普通の 0x01 ストリームで動かす**ので、マイコン側に特別な実装は
  要らない (§4.6)。**既にこのビットを実装したファームでも、そのまま残してよい** ——
  ROS2 が立てないので発動しない。新規実装なら無視すること。
  ビット割当自体は将来のために予約として残す

### 3.3 指令 `packet_type = 0x02` (関節指令。パターンB・当面優先度低)

ROS2側でIKを解いて関節角を直接送るモード。**ROS2側は起動設定でどちらか一方の
モードしか送らない** (0x01と0x02が混ざって届くことはない)。ヘッダの
`packet_type` で分岐できる構造にだけしておき、実装は0x01を優先してよい。

ペイロード44バイト、パケット全体60バイト。

| offset | 型 | 名前 | 単位 |
|---|---|---|---|
| 16 | `float32[5]` | `q[5]` | 関節角 rad |
| 36 | `float32[5]` | `qdot[5]` | 関節角速度 rad/s |
| 56 | `uint8` | `gripper` | 0x01と同じ |
| 57 | `uint8` | `control_flags` | 0x01と同じ |
| 58 | `uint16` | `reserved` | 0埋め |

**関節の並び順 (契約):**

```
index 0: shoulder_left   (肩 左モータ)
index 1: shoulder_right  (肩 右モータ)
index 2: turntable       (ターンテーブル)
index 3: knee_left       (肘/膝 左モータ)
index 4: knee_right      (肘/膝 右モータ)
```

この順序は 0x02 の `q[]`/`qdot[]` と、0x81 の `joint_positions[]` の両方に適用する。

### 3.4 フィードバック `packet_type = 0x81` (マイコン → ROS2)

**指令と同じく xy 平面は極座標で返す** (マイコンは xy 平面の直交座標を
一切扱わなくてよい)。ヘッダは共通 (方向の区別のためフィードバックは0x80以上を使う)。
関節数5なら **合計64バイト**。

| offset | 型 | 名前 | 説明 |
|---|---|---|---|
| 0-15 | | (共通ヘッダ) | `seq`はフィードバック自身の連番 (送信ごとに+1) |
| 16 | `uint32` | `seq_echo` | **最後に受信した指令の`seq`** (往復遅延の測定用) |
| 20 | `float32` | `r` | 実位置 [m]。**マイコン側でFKして返す** (肩の実測角から) |
| 24 | `float32` | `theta` | 実位置 [rad] (ターンテーブルの実測角。**指令と同じく時計回り正**)。**指令と違い ±π を超えた連続値でなくてよい** (ROS2側は cos/sin で直交座標へ戻すだけなので、どちらでも同じ点になる) |
| 28 | `float32` | `z` | 実位置 [m] (肘/膝の実測角から) |
| 32 | `float32` | `pitch` | 実姿勢 [rad] |
| 36 | `float32` | `yaw` | 〃 |
| 40 | `uint16` | `status_flags` | 下記 §3.5 |
| 42 | `uint8` | `gripper_state` | 実際のグリッパ状態 (0=開, 1=閉)。**必ず実状態を返すこと** (ROS2側で把持確認に使う予定) |
| 43 | `uint8` | `joint_count` | 5 |
| 44 | `float32[5]` | `joint_positions` | 実測の関節角 [rad]。並び順は §3.3 |

**2 枚構成での分担は §2.2 を参照。** 各基板は自分の担当フィールドだけを埋め、担当外は 0 で送る
(r/z 基板は `theta`・`pitch`・`yaw`・`gripper_state`・`joint_positions[2]` を 0、
θ 基板は `r`・`z`・`joint_positions[0,1,3,4]` を 0)。
(2026-09-11 まではここに「`status_flags` bit5 は両基板が実装する」とあったが、
bit5 は予約になったので不要。**bit3 は引き続き両基板が正しく立てること。**)

**送信条件: 指令の受信と無関係に、マイコン起動直後から10Hz (100ms 間隔) で自発送信する**
(2026-09-11 に 100Hz から変更。指令と同じ周期)。
指令へのエコー(受信したら返す)型にしないこと。理由:
①ROS2起動時に「目標姿勢を実姿勢に同期する」処理が最初の動作前に確実に走る
②「マイコンが死んだ」と「ROS2が送っていない」をROS2側で区別できる
③ウォッチドッグ発動・未初期化などの状態が指令を送らなくても見える。
帯域は 64バイト×10Hz ≈ 5kbps で有線Ethernetでは問題にならない。ROS2 側はこれを
`/catchrobo/arm/current_pose` としてそのまま流すので、VR・シムに見える実姿勢も 10Hz 更新になる。
ROS2 側は 0.5 秒 (5 発分) 途絶で警告し `current_pose` を止める (`feedback_timeout`)。

**FKはマイコン側で行う。** ROS2は運動学を持たない設計のため、Cartesian実姿勢は
マイコンがエンコーダ値からFKで計算して返す。指令値のエコーではなく実測から計算すること。

### 3.5 `status_flags` のビット定義

| bit | 意味 |
|---|---|
| 0 | 追従誤差過大 |
| 1 | ドライバ異常 |
| 2 | ウォッチドッグ作動中 (指令途絶により最後の目標位置をホールド中) |
| 3 | 未初期化・原点未確定 (原点出しが済むまで立てたまま送る) |
| 4 | 直近の指令を破棄した (可動域外・seq逆転等。破棄後 目安200ms 立てておく) |
| 5 | **予約 (2026-09-11〜。ROS2 は見ない。0 でよい)。** 旧「初期位置に到達・静止中」(`control_flags` bit2 への応答)。bit2 の廃止に伴い不要になった (§3.2 / §4.6)。返しても ROS2 の挙動は変わらない |
| 6-15 | 予約 (0) |

### 3.6 C構造体定義 (STM32側でそのまま使える形)

全パケットをC構造体で表すと以下の通り。**このヘッダをそのままファームウェアに
コピーして使ってよい** (ROS2側 `udp_protocol.hpp` と同一レイアウト)。

```c
#include <stdint.h>

#pragma pack(push, 1)

typedef struct {
  uint8_t  protocol_version;   // = 2
  uint8_t  packet_type;        // 0x01 / 0x02 / 0x81
  uint16_t payload_length;     // ペイロード長 [byte]
  uint32_t seq;                // 送信ごとに +1
  uint64_t timestamp_us;       // 診断用。無視してよい
} CommandHeader;               // 16 bytes

// packet_type = 0x01 (ROS2 → MCU)
typedef struct {
  float    r;                    // [m]   ターンテーブル軸からの水平距離
  float    theta;                // [rad] +X軸から時計回り正。±π を超えうる連続値
  float    z;                    // [m]
  float    pitch, yaw;           // [rad]
  float    r_dot;                // [m/s]
  float    theta_dot;            // [rad/s]
  float    vz;                   // [m/s]
  float    pitch_rate, yaw_rate; // [rad/s]
  uint8_t  gripper;              // 0=開, 1=閉
  uint8_t  control_flags;        // bit0=動作許可, bit1=縦にする指示, bit2=予約 (常に0)
  uint16_t reserved;
} PolarPayload;                  // 44 bytes

typedef struct {
  CommandHeader header;
  PolarPayload  payload;
} PolarPacket;                   // 60 bytes

// packet_type = 0x02 (ROS2 → MCU。パターンB)
typedef struct {
  float    q[5];               // [rad]   並び順は §3.3
  float    qdot[5];            // [rad/s] 同上
  uint8_t  gripper;            // 0x01 と同じ
  uint8_t  control_flags;      // 0x01 と同じ
  uint16_t reserved;
} JointPayload;                // 44 bytes

typedef struct {
  CommandHeader header;
  JointPayload  payload;
} JointPacket;                 // 60 bytes

// packet_type = 0x81 (MCU → ROS2)。実機は関節数5固定でよい
typedef struct {
  CommandHeader header;        // seq はフィードバック自身の連番
  uint32_t seq_echo;           // 最後に受信した指令の seq
  float    r;                  // FK結果 [m]
  float    theta;              // FK結果 [rad]
  float    z;                  // FK結果 [m]
  float    pitch, yaw;         // [rad]
  uint16_t status_flags;       // §3.5
  uint8_t  gripper_state;      // 実際のグリッパ状態 (0=開, 1=閉)
  uint8_t  joint_count;        // = 5
  float    joint_positions[5]; // 実測関節角 [rad]。並び順は §3.3
} FeedbackPacket;              // 64 bytes

#pragma pack(pop)

_Static_assert(sizeof(CommandHeader)   == 16, "layout mismatch");
_Static_assert(sizeof(PolarPacket)     == 60, "layout mismatch");
_Static_assert(sizeof(JointPacket)     == 60, "layout mismatch");
_Static_assert(sizeof(FeedbackPacket)  == 64, "layout mismatch");
```

実装メモ:

- 全フィールドが自然アラインメントになるよう設計してあるため、packed でも
  実質パディングは発生しない (`_Static_assert` が通ることを必ず確認)
- 受信バッファから読むときは `memcpy(&packet, buf, sizeof(packet))` を推奨
  (バッファへの直接キャストはアラインメント起因の未定義動作になりうる)
- `payload_length` はヘッダを含まないペイロード部の長さ:
  0x01/0x02 = 44、0x81 (5関節) = 28 + 4×5 = 48

---

## 4. 挙動の要求事項

### 4.1 受信処理

1. `protocol_version` ≠ 2、`payload_length` 不一致、未知の `packet_type` は**破棄**
2. **`seq` の逆転を検出したら古いパケットを破棄する** (UDPは順序保証がない)。
   破棄したら `status_flags` bit4 を一定時間立てて通知する
3. **受信値が可動域外なら破棄する** (通信化けとバグの両方を防ぐ最後の砦)。
   これも bit4 で通知する。可動域の定義はマイコン側が持ってよい
   (ROS2側も作業領域クランプを持っているので、ここは冗長な安全網)

### 4.2 補間 (最重要)

**受信した位置をそのままドライバに渡してはいけない。** 10Hz指令とドライバの
高速応答の組み合わせは「数msで追いつき残りの ~95ms 待つ」を毎秒10回繰り返す階段状の
動きになる (1 パケットあたりの段差は最大 v_max×0.1s = 10mm)。

**位置+速度を受け取り、次のパケットが来るまで自分の制御ループ周期で目標値を進める:**

```c
// 制御ループ (1kHz推奨) ごと
target_r     += r_dot     * dt;   // dt = 自分のループ周期
target_theta += theta_dot * dt;
// ... z, pitch, yaw も同様 (極座標のまま外挿してよい。
//     直交座標に戻してから外挿する必要は無い)
```

- 新しいパケットが来たら位置を絶対値で再同期する (積分誤差はここでリセットされる。
  10Hz時の再同期ジャンプは ½×a_max×dt² = ½×0.2×0.1² = 1mm 程度)
- 速度だけで積分し続けてはいけない (パケットロスで誤差が永久に蓄積する)。
  位置だけを使ってもいけない (上記の矩形波問題)。**必ず両方使う**

### 4.3 ウォッチドッグ

**次のパケットが500ms(推奨)来なければ、外挿をやめて最後の目標位置をホールドする。**
(2026-09-11 に 50ms から変更。指令が 100ms 間隔なので 50ms では毎パケット間で発動する。
500ms = 指令 5 発分の欠落で発動)

- **サーボONのまま位置保持。脱力(サーボOFF)しない** — アームが自重で落ちると
  ワーク破損(競技で-1点)になる。電源遮断が必要な異常はハードウェアE-stopの担当
- ホールド中は `status_flags` bit2 を立てる
- ストリームが再開したら自動で追従に復帰してよい (次項のスルーレート制限が
  効いているため復帰時にジャンプしない)

### 4.4 スルーレート制限 (常時)

**指令位置と実位置がどれだけ離れていても、自前の速度・加速度上限の範囲でしか
動かない (ジャンプしない) こと。** 起動直後やROS2再起動後は指令と実位置が大きく
ずれていることがあり、無制限に追従すると危険な高速移動になる。

- 上限値はROS2側の `v_max`/`a_max` (現在 0.1 m/s / 0.2 m/s²) より**大きく**
  設定すること。同じか小さいと通常の追従までなまる

### 4.5 ドライバの差異の吸収

モータドライバごとの制御方式の差異 (ODrive / DJI C610 / Feetech 等) は
**すべてマイコン側で吸収する**。ROS2側はハードウェア構成を一切知らない。

### 4.6 起動シーケンスと初期位置 (2026-09-11 改訂。初期位置は ROS2 側)

**マイコン側でやることは「原点出し」と「ホールド」だけ。** 初期位置 (競技開始姿勢) へ
動かすのは ROS2 側の仕事になった (2026-09-11)。

```
1. マイコン電源投入 (ROS2 は未起動でよい)
     → 原点出し (エンコーダ基準の確定)
     → その間 status_flags bit3 を立てたまま 0x81 を送り続ける (§3.4)
     → 済んだら bit3 を落とし、FK した実姿勢を返す。**その場でホールドしてよい**
       (どこに居てもよい。指定の姿勢へ自力で動く必要は無い)
2. ROS2 起動
     → ROS2 は bit3 が落ちたフィードバックを受けて目標姿勢を実姿勢に同期する。
       同期が済むまで control_flags bit0 = 0 (ホールド) で送るので、
       この間に届く r/θ/z (原点の仮値) を追ってはいけない
     → 同期後 bit0 = 1 に切り替わる。目標 = 実姿勢なのでこの瞬間は動かない
3. ROS2 が初期位置まで動かす (bit0 = 1 の普通の 0x01 ストリーム)
     → ROS2 側 (game_state_manager_node) が初期位置へ直線 1 本のゴールで動かす。
       **マイコンから見ると通常の追従と区別が付かない** (特別な処理は不要)
4. 操縦者が目標 (target_pose / cmd_twist) を与える → 通常運転
```

- **初期位置 (競技開始姿勢) の正本は ROS2 側**の `params/robot_geometry.yaml` の
  `init_pose` (極座標 r/θ/z。赤・青で別々)。マイコン側は座標を持たない。
  **値を変えるのに再ビルド・書き込みが要らない** (`ros2 param set` でも変えられる)
- 競技中の「初期位置へ戻す」(VR のステートリセット = `/catchrobo/game/reset`) も同じ経路。
  **bit2 / bit5 は使わない** (§3.2 / §3.5)
- 2026-09-10 の版では「マイコンが自前の初期関節角へ自力で移動し、競技中は bit2 で
  そこへ戻す」契約だった。**この方針は 2026-09-11 に撤回した。** 既にそう実装した
  ファーム (`*_INIT_DEG` への電源投入時移動・bit2 / bit5) はそのまま残してよい ——
  電源投入時にどこに居るかは ROS2 にとって自由で、bit2 は ROS2 が立てないため発動しない
- マイコンだけが再起動した場合も同じ: bit3 を立てて送り始めれば ROS2 は同期を
  取り消して bit0 = 0 に戻り、bit3 が落ちてから同期し直す。
  **0x81 が 500ms 途絶した場合 (ケーブル抜け等) も同じ扱い** (ROS2 側 `feedback_timeout`)。
  **どちらの場合も ROS2 は同期し直した瞬間に自動で初期位置へ戻る** (2026-09-11 ユーザー決定。
  ゲーム状態がどこにあっても `INIT` へ戻り、グリッパは開く)。マイコン側に特別な対応は無い

---

## 5. 座標系

受信する `r, θ, z` は **field座標系 = ロボットのベース座標系** を極座標で
表したもの (座標変換は存在しない。ROS2側が同じ座標系のまま極座標へ直しているだけ)。

- 原点 (ベース座標系): ロボット設置エリア (縦300×横700mm) の中心
- **極座標の原点 (r=0) はターンテーブルの回転軸。** ベース原点からずれていても
  ROS2 側 (`hardware_bridge_node` の `turntable_axis_x/y_m`) が送受信の両方で並進を
  吸収するので、**マイコンは r, θ を「回転軸からの距離・角度」として扱えばよい**。
  ずれの実測値は人間が測って ROS2 側に入れる (マイコン側の対応は不要)
- +X: 設置エリアから作業エリア (ワークが並んでいる方向) へ。**θ = 0 の向き**。
  ターンテーブルのエンコーダ零点はこの向きに合わせること (角度のずれはマイコン側で吸収)
- +Y: 右から左。**θ = +π/2 の向き** (θ は +X から +Y へ回る向きが正)
- +Z: 上
- 単位: m / rad (ROS標準)

直交座標へ戻したいときは `x = r·cos θ`, `y = r·sin θ`。
ROS2側は §3.4 のフィードバックをこの式で直交座標へ戻して
`/catchrobo/arm/current_pose` に流している。

寸法の詳細は [`field_dimensions.md`](field_dimensions.md) を参照。
IK/FKに必要なリンク長・機構定数はマイコン側が実測して持つ (ROS2側は持たない)。

---

## 6. ルールブック由来のハードウェア要求 (ソフトと独立に必須)

競技ルール上、以下は**マイコンやICからの信号に依存せず**回路で実現すること
(この仕様書のUDPソフトとは独立の要求。詳細はルールブック第16回):

- **非常停止ボタン**: 動力電源を直接遮断。赤色プッシュロックターンリセット式
- **動力電源表示灯**: 緑色、非常停止時消灯

つまり「UDPで停止コマンドを送るからE-stopはソフトでよい」は**不可**。

---

## 7. 開発・テストの進め方

### 7.1 リファレンス実装 (答え合わせ用)

`sharmech/sharmech_core/scripts/mock_mcu.py` は、この契約をROS2側から見て
再現するリファレンス実装 (Python、ROS2非依存)。**期待される挙動の答え合わせとして
読める**: 10Hz自発フィードバック・ウォッチドッグフラグ (500ms)・seq逆転破棄+bit4通知・
起動直後の bit3 (`--init-duration`)・bit0=0 ホールドを実装済み。
**電源投入直後に居る位置は `--power-on-pose r,theta,z`** (既定 0.30,0.0,0.15)。
2026-09-11 の改訂で bit2 / bit5 の実装は削除した (§4.6。初期位置へは ROS2 が
普通の 0x01 ストリームで動かすので、モックは電源投入位置でホールドしているだけでよい)。
ただし補間・IK・実モータ制御は含まない (そこは実機側の仕事)。

ROS2側スタック全体をこのモックに繋ぐには launch の `mock_mcu:=true` を使う
(`config.yaml` の実機IPは触らない):

```bash
python3 sharmech_core/scripts/mock_mcu.py --listen-port 8888
ros2 launch sharmech_bringup sharmech.launch.xml field_color:=red joy:=false mock_mcu:=true
```

### 7.2 ROS2側と繋いだ試験

```bash
# ROS2側 (このリポジトリのPC) で:
ros2 launch sharmech_bringup sharmech.launch.xml field_color:=red joy:=false
# config.yaml の mcu_ip をマイコンの実IPに合わせておく

# 動作確認:
ros2 topic echo /catchrobo/arm/current_pose      # マイコンのFK結果が出れば疎通OK
ros2 topic echo /catchrobo/arm/mcu_status        # status_flags・seq_echo・疎通状態
#   ★ 同期した瞬間に ROS2 が初期位置へ動かす (§4.6。/catchrobo/game/state が INIT → WAITING_FOR_PICK)。
#     動かしたくなければ ROS2 側 config.yaml の game_state_manager_node.init_on_startup: false
ros2 topic pub --once /catchrobo/arm/target_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: field}, pose: {position: {x: 0.3, y: 0.0, z: 0.1}, orientation: {w: 1.0}}}"
# → 10Hzの0x01ストリームが台形速度プロファイルで動くのが見える
```

### 7.3 実バイト列の例 (パーサの答え合わせ用)

以下は実際に `UdpProtocol::encodePolar` / `struct.pack` が生成するバイト列
そのもの。自作パーサのテストベクタとして使える。

**0x01 極座標指令 (60バイト):**
値: r=0.25, theta=1.2, z=0.075, pitch=0.1, yaw=0.2, r_dot=0.0667,
theta_dot=0.2667, vz=-0.0333, pitch_rate=0.1333, yaw_rate=0.2667,
gripper=0, control_flags=0x01, seq=100, timestamp_us=1724400000123456

```
offset  bytes (hex, リトルエンディアン)   field
   0    02                        protocol_version = 2
   1    01                        packet_type      = 0x01
   2    2c 00                     payload_length   = 44
   4    64 00 00 00               seq              = 100
   8    40 42 75 29 55 20 06 00   timestamp_us     = 1724400000123456
  16    00 00 80 3e               r     = 0.25  [m]
  20    9a 99 99 3f               theta = 1.2   [rad]
  24    9a 99 99 3d               z     = 0.075 [m]
  28    cd cc cc 3d               pitch = 0.1   [rad]
  32    cd cc 4c 3e               yaw   = 0.2   [rad]
  36    02 9a 88 3d               r_dot     = 0.0667  [m/s]
  40    e7 8c 88 3e               theta_dot = 0.2667  [rad/s]
  44    95 65 08 bd               vz        = -0.0333 [m/s]
  48    cc 7f 08 3e               pitch_rate = 0.1333 [rad/s]
  52    e7 8c 88 3e               yaw_rate   = 0.2667 [rad/s]
  56    00                        gripper       = 0 (開)
  57    01                        control_flags = 0x01 (bit0=許可)
  58    00 00                     reserved      = 0
```

60バイト連続:
```
02 01 2c 00 64 00 00 00 40 42 75 29 55 20 06 00 00 00 80 3e 9a 99 99 3f
9a 99 99 3d cd cc cc 3d cd cc 4c 3e 02 9a 88 3d e7 8c 88 3e 95 65 08 bd
cc 7f 08 3e e7 8c 88 3e 00 01 00 00
```

**0x81 フィードバック (5関節、64バイト):**
値: seq=2500 (FB自身の連番), seq_echo=100, r=0.2505, theta=1.199, z=0.076,
pitch=0.1, yaw=0.2, status_flags=0, gripper_state=0, joint_count=5,
joints=[0.21, 0.21, 1.05, -0.35, -0.35] (§3.3の並び順)

```
offset  bytes (hex)               field
   0    02                        protocol_version = 2
   1    81                        packet_type      = 0x81
   2    30 00                     payload_length   = 48 (= 28 + 4×5)
   4    c4 09 00 00               seq              = 2500
   8    50 42 75 29 55 20 06 00   timestamp_us
  16    64 00 00 00               seq_echo = 100
  20    89 41 80 3e               r     = 0.2505 [m]   (FK結果)
  24    d5 78 99 3f               theta = 1.199  [rad] (FK結果)
  28    e3 a5 9b 3d               z     = 0.076  [m]
  32    cd cc cc 3d               pitch = 0.1 [rad]
  36    cd cc 4c 3e               yaw   = 0.2 [rad]
  40    00 00                     status_flags = 0x0000 (正常)
  42    00                        gripper_state = 0 (開)
  43    05                        joint_count   = 5
  44    3d 0a 57 3e               joint[0] = 0.21  (shoulder_left)
  48    3d 0a 57 3e               joint[1] = 0.21  (shoulder_right)
  52    66 66 86 3f               joint[2] = 1.05  (turntable)
  56    33 33 b3 be               joint[3] = -0.35 (knee_left)
  60    33 33 b3 be               joint[4] = -0.35 (knee_right)
```

64バイト連続:
```
02 81 30 00 c4 09 00 00 50 42 75 29 55 20 06 00 64 00 00 00 89 41 80 3e
d5 78 99 3f e3 a5 9b 3d cd cc cc 3d cd cc 4c 3e 00 00 00 05 3d 0a 57 3e
3d 0a 57 3e 66 66 86 3f 33 33 b3 be 33 33 b3 be
```

---

## 8. マイコン側で用意が必要なもの (パケット以外の前提条件)

UDPの指令だけでは動かない。以下はパケットでは送られてこないため、
マイコン側で実測・設計・設定する必要がある。

| # | 項目 | 状態 |
|---|---|---|
| 1 | **機構の幾何パラメータ (IK/FK用)**: リンク長・ピボット間距離・ターンテーブル軸位置・肘/膝機構の基準高さ | **未実測** (ROS2側にも仮値しか無い)。これが無いとIKが解けず動かせない。最優先 |
| 2 | **原点出し (ホーミング) の手順**: 電源投入時のエンコーダ基準の確定方法 | マイコン側の設計裁量。完了まで `status_flags` bit3 を立てる (§3.5)。**その後どこで待つかは自由** (2026-09-11〜。競技開始姿勢へは ROS2 が動かすので、マイコン側の「初期関節角」は必須ではなくなった。§4.6) |
| 3 | **モータ・エンコーダ設定**: 回転方向・ギア比・カウント→rad換算 | マイコン側の設計裁量 |
| 4 | **関節可動域 (joint limits)**: 可動域外破棄 (bit4) 用 | マイコン側の設計裁量。実測後ROS2側の作業領域クランプと整合を取る |
| 5 | **グリッパ・「縦にする」機構の実物**: アクチュエータ・駆動回路 | 機構未定 (2026-09-01時点)。bit1の指示経路はプロトコル側で確保済み |
| 6 | **ネットワーク静的設定**: 自分のIP (r/z 基板 192.168.1.100、θ 基板 192.168.1.101) と、フィードバック宛先 (ROS2側PCのIP:8889。両基板とも同じ宛先) | 現場で確定 (§2、§2.2) |
| 7 | **ハードE-stop・動力電源表示灯**: IC非依存の回路 | ルールブック必須 (§6)。ソフトとは独立 |

## 9. 未確定事項 (変わる可能性のある箇所)

変更が入る場合は `protocol_version` の更新または本書の改訂として通知する。

| 項目 | 現状 | 備考 |
|---|---|---|
| `pitch`/`yaw` の扱い | 連続値 (float32) として定義済みだが**実質未使用** | 2026-09-05 確認: ピッチは機構的に2値 (横倒し/縦) であり、**`control_flags` bit1 が正本**。連続値の `pitch`/`pitch_rate` は VR運用・自動シーケンスのどちらでも常に0が流れる (PS4の右スティックだけが唯一の非ゼロ源だが、5軸機構に姿勢の自由度が無いため実現できない)。**位置(x,y,z)+速度+グリッパ+bit1 だけ実装すればよく、pitch/yaw は無視してよい。** 将来手首を追加する場合の枠として残してある |
| 把持確認 | ROS2側は固定時間待ち (0.3s) の暫定実装 | `gripper_state` を実状態で返してもらえれば、将来「実際に閉じたことの確認」に置き換える |
| ピッチ完了の確認 | ROS2側は固定時間待ち (`orient_dwell_sec` 既定0.5s) の暫定実装 | 0x81 にピッチの実状態を返すフィールドが無い。返せるようになれば把持確認とまとめて実確認へ置き換える |
| ターンテーブル回転軸の位置 | ベース原点からのずれは ROS2 側が吸収する (§5。2026-09-10 実装済み)。値は `sharmech/params/robot_geometry.yaml` の `kinematics.turntable_axis_x/y_m` (未実測。人間が測って入れる) | マイコン側の対応は不要。θ の零点方向 (+X) だけマイコン側で合わせる |
| θ の可動域 | マイコン側の裁量 | ROS2側は ±π を超える連続値を送りうる (§3.2)。物理的に回れない範囲は §4.1 の可動域外破棄で弾いてよい |
| 0x02 (関節指令) | ワイヤ仕様は確定・ROS2側実装済み | リンク長が未実測のため当面実機では使わない。実装優先度は低くてよい |
| 初期位置要求 (bit2) / 到達 (bit5) | **2026-09-10 に追加し、2026-09-11 に予約へ戻した** (初期位置が ROS2 側の `init_pose` へ移ったため。§3.2/§3.5/§4.6)。**パケットレイアウトはどちらの改訂でも不変**なので `protocol_version` は 2 のまま | ROS2 は bit2 を常に 0 で送り、bit5 を見ない。実装済みのファームはそのままでよい |
| 可動域の値 | マイコン側の裁量 | 実測が揃い次第、ROS2側の作業領域クランプ値と整合を取る |

---

## 10. 基板別の実装仕様 (ファーム担当者向け。2026-09-10)

§1〜§9 は「ROS2 から見た契約」であり、この章は **その契約を現行ファーム
`STM32_UDP2CAN_controller` の上にどう実装するか** を基板ごとに書いたもの。
ファーム側も Claude Code で作業する前提なので、判断の根拠と「人間が決める値」を明示してある。
現行ファームのコードは 2026-09-10 にリモートの各ブランチ (`ip_100_section` / `enndeffector_UDP`) を
読んで確認した (ローカル `main` は 9/5 で止まっているので **必ず `git fetch` してブランチを読むこと**)。

### 10.1 初版の範囲と共通の方針

**初版でやること (両基板共通):**

1. UDP 8888 番で 0x01 を受信し、§4.1 の検証 (version / type / length / seq) を通す
2. `control_flags` bit0 (動作許可) に従って目標を決める (§4.6。bit2 は予約で来ない)
3. 自分の担当フィールドだけを各モータの角度へ変換し、**既存の CAN 送信経路**でモータへ送る
4. 0x81 を **起動直後から 10Hz** で ROS2 側 PC へ返す (§3.4)。担当外のフィールドは 0 (§2.2)
5. 500ms 指令が途絶えたら bit2 を立てる (§4.3)。目標を更新しないだけでホールドになる

**初版でやらないこと (ユーザー決定 2026-09-10。「まず最低限動く」ことを優先):**

| 項目 | 扱い |
|---|---|
| §4.2 の速度による補間 | **やらない。** 受信した位置をそのまま目標にする。段差は ODrive の `POS_FILTER` (r/z 基板)・wave motor 基板の自前の減速・C610 のカスケードループ (θ 基板) が均す。ROS2 側の `v_max`=0.1 m/s / `a_max`=0.2 m/s² なら 100ms あたりの段差は最大 10mm (10Hz 化で 100Hz 時の 10 倍。階段が目立つなら §4.2 を追加する) |
| §4.4 のスルーレート制限 | **やらない。** 上と同じ理由。ただし §10.2 / §10.3 の可動域チェックは入れる (跳びは防げないが範囲外へは行かない) |
| 0x02 (関節指令、パターンB) | 受信しても**黙って破棄** (bit4 も立てない) |
| `pitch`/`yaw`/`*_rate` の連続値 | 無視 (§9)。姿勢は bit1 の 2 値だけ |
| CRC | 入れない (§決定済み) |

**「修正ではなく拡張」の方針 (ユーザー指示):** 現行ファームは CAN の送信順序・
起動シーケンス・再送・診断がよく調整されているので、**既存の行は一切変更しない。**
許される変更は次の 3 種類だけ。

| 変更 | 内容 |
|---|---|
| ① 新規ファイルの追加 | `ros2_link.c/.h` (UDP 受信・0x81 送信・フラグ・ウォッチドッグ)、`ros2_protocol.h` (§3.6 の構造体をそのまま)、`user_config.h` (人間設定の定数)、r/z 基板のみ `arm_ik.c/.h` |
| ② `main.c` への呼び出し行の追加 | `ROS2Link_Init(...)` を `HLControlUDP_Init` の直後に 1 行、`ROS2Link_Poll()` と `ROS2Link_Tick()` を `Delay_WithCANPoll()` のループ内と `while(1)` の末尾に各 1 行 (既存の `HLControlUDP_Poll()` の隣) |
| ③ `hl_control_udp.c/.h` への**公開関数の追加** | 目標値を書き込む setter (下記)。**既存の static 変数・既存関数の中身は触らない** |

③が必要な理由: `hl_control_udp.c` は各軸の目標を file-static 変数に持ち、
`HLControlUDP_ResendTick()` が 300ms ごとにそれを再送する (ODrive ウォッチドッグ feed)。
新モジュールが別経路で `Set_Input_Pos` を送ると、**300ms ごとに古い目標へ引き戻されて振動する**。
目標は必ず同じ static 変数へ書く必要があり、そのための入口を追加する。
5003 番のベンチプロトコルは残す (同じ変数に書くので**後から書いた方が勝つ**。
ROS2 が動いている間にベンチ GUI から送ると競合するが、運用で避ける)。

**W5500 ソケット:** 0〜3 は使用中 (Hello / ブリッジ 5001 / ハートビート 5002 / 高レベル制御 5003。
θ 基板は 5004 も)。**ソケット 4 を 8888 番で開き** (`socket(4, Sn_MR_UDP, 8888, 0)` +
`ctlsocket(CS_SET_IOMODE, SOCK_IO_NONBLOCK)`)、0x81 の送信も同じソケットから行う。
受信バッファは既定 2KB (指令 60B なら約 30 パケット = 300ms 分) なので、**毎回の `Poll()` で
`recvfrom` を空になるまで回し、最後のパケットだけ採用する** (5ms ごとの Poll に対し 10Hz なら
通常 0〜1 個。ハートビート組み立て等でブロックした後に複数溜まる)。フィードバック宛先は `ROS2_PC_IP` (人間設定。`main.c` の `dest_ip` と同じ
192.168.1.2 を既定に) のポート 8889。

**周期の作り方:** `Delay_WithCANPoll()` は約 5ms ごとに登録済みの Poll 群を回している。
新モジュールはそこに乗るだけで、**タイマ割り込みは使わない** (既存構造を変えない)。
`ROS2Link_Tick()` の中で `HAL_GetTick()` を見て、(a) 100ms ごとに 0x81 を 1 発送る、
(b) CAN 送信を 1 tick に 1 軸だけ行う (r/z 基板。下記)、(c) 最終受信から 500ms 経過で bit2 を立てる。
メインループ側の `Delay_WithCANPoll(1500)` の途中でハートビート組み立て等がブロックする区間は
数 ms なので、10Hz のフィードバックに数 ms のジッタが乗るのは許容する。

**人間が設定する定数の書き方 (`user_config.h`):**

- 1 行ずつ `/* ★HUMAN: 〜 */` のコメントを付け、**単位を名前に含める** (`_DEG`, `_M`, `_MS`)
- **寸法 (リンク長など未実測のもの) は `NAN` で初期化する。** `NAN` が残っている間、
  ファームは IK を走らせず、0x01 を毎回 bit4 (破棄) で応答して現在位置をホールドする。
  (2026-09-11 まではここに「bit2 は関節空間で完結するので寸法が無くても動く」と
  書いてあったが、bit2 は予約になった。**寸法が入るまでは Cartesian 指令で
  一切動かせない**ので、それまでは 5003 番の手動ジョグで動かすこと)
- 角度 [deg] の基準は **既存の 5003 番プロトコルと同じ** (`*_INITIAL_NATIVE_TURNS` からの
  偏差を出力軸 deg で表したもの。ユーザー決定)。IK の関節角 [rad] との間は
  `deg = SIGN × rad × 180/π + OFFSET_DEG` の 1 次式で結び、`SIGN` と `OFFSET_DEG` を人間が入れる
- `*_INIT_DEG` (電源投入後に自力で行く角度) は **2026-09-11 以降は任意** ——
  競技開始姿勢へは ROS2 が動かすので、**`*_INIT_AT_POWER_ON_POSE = 1` (電源投入時の
  現在位置をそのまま初期位置にし、起動時に動かさない) を推奨既定とする** (§4.6)。
  実装済みなら残してよい。既存の `*_INITIAL_NATIVE_TURNS` は「deg=0 の基準点」として
  **そのまま残す** (意味を変えない)

**フラグの共通処理:**

| 受信 | 動作 |
|---|---|
| bit0 = 0 | 目標を**更新しない** (直前の目標のまま = ホールド)。CAN へは既存の再送だけが流れる |
| bit0 = 1 | 担当フィールドを角度に変換して目標にする |
| bit2 = 1 | **来ない** (2026-09-11〜 ROS2 は常に 0 を送る)。旧実装 (`*_INIT_DEG` へ移動 → bit5) を残していても発動しない |
| seq 逆転・length 不一致・可動域外・IK 不能 | 破棄して bit4 を 200ms 立てる。目標は変えない |

### 10.2 r/z 基板 (192.168.1.100、`ip_100_section` ブランチ)

**接続されているモータ (現物・ファームから確認済み):**

| CAN node | 機種 | 対の規約 (既存 0x04/0x05 ハンドラ) | 既存の基準定数 |
|---|---|---|---|
| 0x06 (A) / 0x07 (B) | GIM6010-8 (ODrive CAN Simple、減速比 8、`POS_FILTER`) | **同符号。** 入力 deg を 0〜60 にクランプ → 符号反転 → `delta = deg/360×8`、A も B も `init + delta` | `GIM6010_A/B_INITIAL_NATIVE_TURNS` (-0.61 / -0.38) |
| 0x04 (C) / 0x05 (D) | GIM8018-8 (同上) | **逆符号。** クランプ無し、`delta = deg/360×8`、C = `origin + delta`、D = `origin − delta`。origin は起動時マルチターン正規化で確定 | `GIM8018_C/D_INITIAL_NATIVE_TURNS` (0.2 / 0.2) |
| 0x21 | TTL サーボ | 5003 番の open/close は残っているが、**グリッパは θ 基板の担当** (§10.3)。この基板の ROS2 モジュールは `gripper` を無視する | — |
| ESC 1 / VESC 10 | C610 / VESC | 初期化・受信監視のみ。使わない | — |

**担当:** `r` を肩の対に、`z` を肘/膝の対に割り当てる。**どちらの対 (GIM6010 か GIM8018) が肩かは
未確認**なので、人間設定 `ARM_SHOULDER_PAIR` (`PAIR_GIM6010` / `PAIR_GIM8018`) で切り替える。
`theta`・`gripper`・bit1・`pitch`/`yaw` は無視する (θ 基板の担当)。

**IK (ROS2 側 `parallel_arm_kinematics.hpp` の C 移植。数式は同一にすること):**

肩・肘/膝はどちらも「2 つの固定ピボットに付いたモータが対称に回る」同型の機構で、
ピボット中点からエンドエフェクタまでの距離 `d` と関節角 `φ` の関係は

```
順運動学  d(φ) = L1·sin φ + sqrt( L2² − (a − L1·cos φ)² )        (根号内 < 0 なら到達不可)
逆運動学  ρ = hypot(a, d)          (ρ > L1+L2 または ρ < |L1−L2| なら到達不可)
          α = acos( (L1² + ρ² − L2²) / (2·L1·ρ) )
          β = atan2(d, a)
          候補 φ ∈ { β − α, β + α }
          → 各候補を順運動学に入れて |d(φ) − d| < 1e-6 のものだけ残し、
            直前の関節角に近い方を採る (両方残ることがある。単調でない曲線のため)
```

`a` = ピボット間距離の半分、`L1` = ピボット側リンク、`L2` = EE 側リンク。
`φ` は「もう一方のピボットへ向かう向き」を 0、EE 側へ回るほど正 (ROS2 側と同じ定義)。
**肩:** `d = r` (受信した `r` をそのまま。ターンテーブル軸のオフセットは ROS2 側が済ませている §5)。
**肘/膝:** `d = z − knee_base_height_m`。
到達不可なら bit4 を立てて目標を変えない。左右モータは常に同じ `φ` (対称駆動)。

**人間が設定する定数 (`user_config.h`):**

| 定数 | 単位 | 対応する ROS2 側の名前 (`robot_geometry.yaml`) | 初期値 |
|---|---|---|---|
| `SHOULDER_PIVOT_HALF_SEPARATION_M` | m | `kinematics.shoulder_pivot_half_separation_m` | `NAN` (要実測) |
| `SHOULDER_PROXIMAL_LINK_LENGTH_M` | m | `kinematics.shoulder_proximal_link_length_m` | `NAN` |
| `SHOULDER_DISTAL_LINK_LENGTH_M` | m | `kinematics.shoulder_distal_link_length_m` | `NAN` |
| `KNEE_PIVOT_HALF_SEPARATION_M` | m | `kinematics.knee_pivot_half_separation_m` | `NAN` |
| `KNEE_PROXIMAL_LINK_LENGTH_M` | m | `kinematics.knee_proximal_link_length_m` | `NAN` |
| `KNEE_DISTAL_LINK_LENGTH_M` | m | `kinematics.knee_distal_link_length_m` | `NAN` |
| `KNEE_BASE_HEIGHT_M` | m | `kinematics.knee_base_height_m` (肘/膝の d=0 に対応するベース座標 z) | `NAN` |
| `ARM_SHOULDER_PAIR` | — | — (どちらの対が肩か) | 要確認 |
| `GIM6010_DEG_PER_RAD_SIGN` / `GIM8018_DEG_PER_RAD_SIGN` | ±1 | — (IK の φ の正方向と 5003 deg の正方向が一致するか) | 要確認 |
| `GIM6010_DEG_OFFSET` / `GIM8018_DEG_OFFSET` | deg | — (φ = 0 のときの 5003 deg) | 要実測 |
| `GIM6010_INIT_DEG` / `GIM8018_INIT_DEG` | deg | — (電源投入時の到達先) | **任意** (2026-09-11〜。下の `ARM_INIT_AT_POWER_ON_POSE = 1` なら不要) |
| `ARM_INIT_AT_POWER_ON_POSE` | 0/1 | — (1 なら電源投入時の現在位置を初期位置にし、起動時に動かさない) | **1 (推奨既定。競技開始姿勢へは ROS2 が動かす。§4.6)** |
| `GIM6010_MIN_DEG` / `MAX_DEG`、`GIM8018_MIN_DEG` / `MAX_DEG` | deg | — (bit4 の可動域。GIM6010 は既存クランプ 0〜60 の内側) | 要決定 |
| `GIM6010_LEFT_NODE` / `GIM8018_LEFT_NODE` | node id | — (`joint_positions` の `*_left` にどちらを載せるか) | 要確認 |
| `ARM_TRACKING_ERROR_DEG` | deg | — (bit0 の閾値) | 5 |
| `ARM_INIT_TOLERANCE_DEG` | deg | — (電源投入時の到達判定。bit3 を落とす条件) | 1 |
| `ROS2_PC_IP` | — | — | 192,168,1,2 |

測り方は `sharmech/docs/measurement_checklist.md`。ROS2 側の `robot_geometry.yaml` にも同じ値を
入れる (パターン B とシミュレータ用。**値の正本は人間の実測で、両方に同じ数を書く**)。

**CAN 送信 (既存経路の再利用):** `hl_control_udp.c` に次の公開関数を**追加**する
(既存の 0x04/0x05 ハンドラの中身と同じ換算・同じ guard を使い、`s_*_target` / `s_*_armed` へ書く)。

```c
/* 0x04 ハンドラと同じ換算 (0〜60 クランプ → 符号反転 → init + deg/360*8) で目標を記憶するだけ。送信しない */
void HLControlUDP_SetGim6010TargetDeg(float deg);
/* 0x05 ハンドラと同じ換算 (C = origin + Δ, D = origin − Δ)。正規化未完了なら無視して 0 を返す */
uint8_t HLControlUDP_SetGim8018TargetDeg(float deg);
/* armed な軸のうち 1 軸だけ Set_Input_Pos を送る (呼ぶたびに node7→6→5→4 の順で進む)。
 * MotorCanReady 前・正規化前は何もしない。送ったら 1 を返す */
uint8_t HLControlUDP_FlushOneAxis(void);
```

`ROS2Link_Tick()` は 5ms ごとに `HLControlUDP_FlushOneAxis()` を 1 回呼ぶ。4 軸で 20ms、
つまり**各軸 50Hz の更新**になる (10Hz の指令に対して十分。新しい目標は次の Flush で反映)。1 パケット受信ごとに 4 軸へ
`HAL_Delay(5)` を挟んで送る既存ハンドラの形は**使わない** (15ms ブロックして 10ms 周期に収まらない)。
既存の `ResendTick` (300ms) はそのまま動き、同じ変数を再送するので競合しない。
既存の起動時 5ms 間隔・node7→6→5→4 の順序 (低優先度ノードの飢餓対策) を守ること。

**起動シーケンス (§4.6):** 既存の `GIM_StartupBringUp()` / `GIM_StartupTick()` はそのまま。
両 GIM6010 が CLOSED_LOOP かつ両 GIM8018 が正規化済みになったら、新モジュールが
`*_INIT_DEG` を setter で書き、全軸が `ARM_INIT_TOLERANCE_DEG` 内に入った時点で bit3 を落とす。
それまで (電源投入から 7〜10 秒程度) は bit3 を立てた 0x81 を送り続ける。
**これは現状 (起動後は現在位置をホールドするだけ) からの挙動変更**なので、
`ARM_INIT_AT_POWER_ON_POSE = 1` のときは電源投入時に捕捉した現在位置 (既存の
`CaptureHold` が記憶する `pos_estimate`) をそのまま初期位置として扱い、起動時に動かさない。
**2026-09-11 以降はこの 1 (現状維持) を推奨既定とする** —— 競技開始姿勢へは ROS2 が
同期後に動かすので (§4.6)、電源投入時にどこに居ても構わない。
`*_INIT_DEG` へ自力で行かせたい場合だけ人間が 0 に切り替える。

**0x81 の中身:**

| フィールド | 値 |
|---|---|
| `r`, `z` | 各対の実測角 → FK。実測角は `pos_estimate` (モータ軸 turns) から `deg = ±(pos − init_native)/8×360` (対の符号規約の逆算) → 1 次式の逆で rad。対の 2 軸は平均せず **左側 (`*_LEFT_NODE`) の値**を使う |
| `theta`, `pitch`, `yaw`, `gripper_state` | 0 (θ 基板の担当) |
| `joint_positions[0..4]` | `[shoulder_left, shoulder_right, 0, knee_left, knee_right]` (rad、φ の定義) |
| bit0 | いずれかの軸で `|pos_estimate − 目標| > ARM_TRACKING_ERROR_DEG` |
| bit1 | いずれかの軸で `axis_error != 0`、または heartbeat が 1 秒以上来ていない |
| bit2 | 最終受信から 500ms 超 |
| bit3 | 上記の起動シーケンス完了前 (`GIM6010_IsClosedLoop` × 2 と `GIM8018_IsStartupNormalized` × 2、`ARM_INIT_AT_POWER_ON_POSE = 0` なら初期位置到達も) |
| bit4 | 直近 200ms 以内に破棄あり (seq 逆転・可動域外・IK 不能・寸法が `NAN`) |
| bit5 | **常に 0 でよい** (2026-09-11〜 予約。旧: bit2 受信中かつ全軸が `*_INIT_DEG` の `ARM_INIT_TOLERANCE_DEG` 内) |

`pos_estimate` は ODrive の周期送信 (`encoder_rate_ms`。GIM6010 は 10〜50ms、GIM8018 は 97ms に
設定されている) で更新されるので、0x81 の値は最大 100ms 古い。初版では許容する
(ROS2 側の同期は静止時に行われる)。

**CAN バス負荷の注意:** GIM8018 の node5 は同一周期のノードに飢餓させられた実績がある
(`tools/odrive_setup/ODRIVE_ABSOLUTE_ANGLE_NOTES.md` §8)。50Hz×4 軸の追加送信で
`txQFail` / node5 の `hbAge` (ハートビート 5002 に出る) が悪化しないことを実機で確認する。
悪化するなら `FlushOneAxis` の呼び出し間隔を 10ms に落とす (各軸 25Hz)。

### 10.3 θ 基板 (192.168.1.101、`enndeffector_UDP` ブランチ)

**接続されているモータ (現物・ファームから確認済み):**

| CAN id | 機種 | 既存の駆動のしかた | ROS2 での役割 |
|---|---|---|---|
| ESC 3 (`C610_CURRENT_ESC_ID`) | DJI C610 + M2006、**AS5600 磁気エンコーダ (I2C) で角度フィードバック** | 5003 番 0x07: `target = norm360(C610_POS_ORIGIN_DEG + offset)` へカスケード位置制御 (位置 P → 速度 PI → 電流 ±0.8A、20Hz、速度上限 60°/s)。**電源投入直後は無効 (フリー)** で最初の 0x07 から制御開始 | **θ (ターンテーブル)** |
| 0x11 / 0x12 | wave motor ×2 (受信専用基板、差動機構) | 5003 番 0x01/0x08/0x09: ピッチ = 2 台逆回転、ヨー = 同回転。`HLControlUDP_WaveTick()` が起動時に 0 を送って整定を待ち、以後は目標変更時にバースト再送 | **bit1 (縦にする)** |
| 0x21 | TTL サーボ (受信専用) | 5003 番 0x02/0x03: open = −25°、close = +50°。ヨー軸に機械連動しているため `TtlYawFollowTick()` がヨー角へ追従 | **`gripper`** |

**担当:** `theta`・`gripper`・bit1、それに bit0 (bit2 は予約)。`r`・`z`・`pitch`/`yaw` の連続値は無視する。

**θ → ターンテーブル:** 受信 `theta` [rad] は ±π を超えた連続値で届く (§3.2)。
**正方向は上から見て時計回り (+X → −Y)** (2026-09-11〜。ROS2 側で反転済みなので、
MCU 側で追加の反転はしない)。

```
table_deg  = THETA_DEG_PER_RAD_SIGN × theta × 180/π           (出力軸のターンテーブル角、+X が 0)
offset_deg = table_deg × TURNTABLE_ENC_DEG_PER_TABLE_DEG        (AS5600 が測っている軸の角度に換算)
→ 既存の 0x07 ハンドラが呼んでいるのと同じ関数へ offset_deg を渡す
   (target = norm360(C610_POS_ORIGIN_DEG + offset_deg)。C610_POS_ORIGIN_DEG が「θ=0 = +X」に対応する)
```

- 既存の位置ループは誤差を ±180 に折り返して最短経路で回すので、10Hz の増分 (v_max なら 1 発 ≤ 10mm 相当) に対しては
  ROS2 側のアンラップと同じ向きに連続して回る。**AS5600 は単回転 (0〜360°) なので、ターンテーブルの
  可動域は 1 回転未満に限る。** `TURNTABLE_MIN_DEG` / `MAX_DEG` の外は bit4 で破棄する
  (ケーブルの巻き込み防止。ROS2 側が ±π を超える値を送っても物理可動域で弾く)
- 既存ループの性能 (20Hz、±0.8A、60°/s) は初版ではそのまま使う。追従が遅いなら
  `c610_position.h` のゲインを人間が調整する (ROS2 側の要求は §4.4 の「v_max より速く」)

**`gripper` → TTL サーボ:** `gripper = 1` で既存の close (+50°)、`0` で open (−25°)。
値が**変化したときだけ**既存の open/close と同じ関数を呼ぶ (再送とヨー追従は既存の
`TtlYawFollowTick()` が担当するので、パケットごとに毎回呼ばない)。

**bit1 → wave motor (ピッチ):** `pitch_deg = bit1 ? WRIST_PITCH_VERTICAL_DEG : WRIST_PITCH_HORIZONTAL_DEG`、
`yaw_deg = WRIST_YAW_HOLD_DEG` (既定 0)。値が変化したときだけ既存の 0x09 (ピッチ+ヨー同時) と
同じ関数を呼ぶ。実送信・ソフトスタート・バースト再送は既存の `WaveTick()` が担当する。

`hl_control_udp.c` に追加する公開関数:

```c
void HLControlUDP_SetC610TargetOffsetDeg(float offset_deg);   /* 0x07 ハンドラと同じ */
void HLControlUDP_SetGripperClosed(uint8_t closed);           /* 0x02/0x03 ハンドラと同じ */
void HLControlUDP_SetWristPitchYawDeg(float pitch, float yaw); /* 0x09 ハンドラと同じ */
float HLControlUDP_GetC610MeasDeg(void);                      /* AS5600 由来の meas_deg (0x81 用) */
uint8_t HLControlUDP_IsC610FeedbackAlive(void);               /* AS5600 サンプルが途絶していないか */
```

**起動シーケンス (§4.6):** 現状は最初の 0x07 が来るまでターンテーブルはフリー。
2026-09-11 の改訂で**電源投入時に指定角へ自力で行く必要は無くなった** (競技開始姿勢へは
ROS2 が同期後に動かす) ので、**`TURNTABLE_INIT_AT_POWER_ON_POSE = 1` を推奨既定とする** ——
起動から 2 秒 (`C610_CURRENT_STARTUP_DELAY_MS`) 経過かつ AS5600 が有効 (`ok=1`, `mag=1`) に
なった時点の `meas_deg` をそのまま目標にして制御を始める (**その場で保持するだけで動かない**)。
0 に切り替えると `TURNTABLE_INIT_DEG` を setter で書いて**電源投入でターンテーブルが動く**ので、
人間はそれを承知の上で値を決めること。いずれの場合も、制御を始めるまでは bit3 を立てておく。
wave motor は既存の `WaveTick()` が起動時にオフセット 0 を送る (= `WRIST_PITCH_HORIZONTAL_DEG` を 0 に
しておけば「横倒し」が初期姿勢)。到達を確認する手段が無いので、`WRIST_INIT_SETTLE_MS` 経過で到達扱い。

**人間が設定する定数 (`user_config.h`):**

| 定数 | 単位 | 意味 | 初期値 |
|---|---|---|---|
| `THETA_DEG_PER_RAD_SIGN` | ±1 | θ の正方向 (**+X → −Y、時計回り**。2026-09-11 に反時計回りから変更) と AS5600 の増加方向が一致するか | 要確認 |
| `TURNTABLE_ENC_DEG_PER_TABLE_DEG` | — | AS5600 が測っている軸 1° あたりのターンテーブル角。出力軸直付なら 1.0 | 要確認 |
| `C610_POS_ORIGIN_DEG` (既存、`main.c`) | deg | θ = 0 (+X) のときの `meas_deg`。既存の意味のまま | 要実測 |
| `TURNTABLE_INIT_DEG` | deg | 電源投入時の到達先 (ターンテーブル角) | **任意** (2026-09-11〜。下が 1 なら不要) |
| `TURNTABLE_INIT_AT_POWER_ON_POSE` | 0/1 | 1 なら電源投入時の現在角を INIT にする (起動時に動かさない。現状維持) | **1 (推奨既定。§4.6)** |
| `TURNTABLE_MIN_DEG` / `MAX_DEG` | deg | bit4 の可動域 (1 回転未満) | 要決定 |
| `TURNTABLE_TRACKING_ERROR_DEG` | deg | bit0 の閾値 | 5 |
| `TURNTABLE_INIT_TOLERANCE_DEG` | deg | 電源投入時の到達判定 (bit3 を落とす条件) | 1 |
| `WRIST_PITCH_VERTICAL_DEG` / `WRIST_PITCH_HORIZONTAL_DEG` | deg | bit1 = 1 / 0 のピッチ (5003 番 0x01 と同じ定義) | 要実測 / 0 |
| `WRIST_YAW_HOLD_DEG` | deg | ヨーの固定値 | 0 |
| `WRIST_INIT_SETTLE_MS` | ms | wave/TTL の「到達」とみなす待ち時間 | 1500 |
| `ROS2_PC_IP` | — | フィードバック宛先 | 192,168,1,2 |

**0x81 の中身:**

| フィールド | 値 |
|---|---|
| `theta` | `(meas_deg − C610_POS_ORIGIN_DEG) / TURNTABLE_ENC_DEG_PER_TABLE_DEG × π/180 × THETA_DEG_PER_RAD_SIGN`。±π に折り返してよい (§3.4) |
| `joint_positions[2]` | `theta` と同じ値。他の 4 つは 0 |
| `pitch` | bit1 の現在状態のエコー: 縦なら π/2、横なら 0 (実測できないため) |
| `yaw` | 0 |
| `gripper_state` | 最後に受け取った `gripper` のエコー (TTL サーボは応答を返さない。§9) |
| `r`, `z` | 0 (r/z 基板の担当) |
| bit0 | `|meas_deg − target| > TURNTABLE_TRACKING_ERROR_DEG` |
| bit1 | AS5600 の磁石未検出 (`mag=0`) または AS5600 サンプル途絶 (`C610Position_Failsafe` が電流を 0 にした状態) |
| bit2 | 最終受信から 500ms 超 |
| bit3 | 起動 2 秒前、AS5600 無効、または (`TURNTABLE_INIT_AT_POWER_ON_POSE = 0` のとき) 初期位置未到達 |
| bit4 | 直近 200ms 以内に破棄あり (seq 逆転・可動域外) |
| bit5 | **常に 0 でよい** (2026-09-11〜 予約。旧: bit2 受信中かつ `|meas_deg − INIT| < TURNTABLE_INIT_TOLERANCE_DEG` かつ `WRIST_INIT_SETTLE_MS` 経過) |

**CAN バスの注意:** C610 の 1kHz フィードバックで wave motor 基板の受信が飽和する問題は
既存のバースト再送で対処済み。新モジュールは wave/TTL への送信を「変化時のみ」にして
バス負荷を増やさない。

### 10.4 試験手順 (基板ごと)

1. **回帰:** 5003 番のベンチ GUI (`tools/can_id_scanner/can_id_scanner.py`) が従来どおり動くこと
   (新モジュールが何も受信していない状態で既存挙動が変わっていないことの確認)
2. **自発送信:** 電源投入直後から Wireshark (`udp.port == 8889`) に 0x81 が 10Hz で流れ、
   bit3 が立った状態から起動シーケンス完了で 0 になること。担当外フィールドが 0 であること
3. **ROS2 接続:** ROS2 側 PC で `ros2 launch sharmech_bringup sharmech.launch.xml field_color:=red`
   (config.yaml の `mcu_ip` / `mcu_theta_ip` が両基板)。`ros2 topic echo /catchrobo/arm/mcu_status` で
   `connected: true`、`status_flags: 0` になること。**片方の基板だけでも `mcu_status` は出るが
   `/catchrobo/arm/current_pose` は両方揃うまで出ない** (ROS2 側の仕様。§2.2)
4. **初期位置 (2026-09-11 改訂。マイコン側に特別な実装は不要):** 手順 3 で ROS2 を上げると、
   同期が済んだ瞬間に **ROS2 が初期位置まで動かす** (`/catchrobo/game/state` が `INIT` の間に
   直線 1 本で動き、着いたら `WAITING_FOR_PICK` になる)。
   マイコンから見ると bit0=1 の通常の 0x01 追従でしかない。**bit2 は来ないので、
   実装していてもしていなくても結果は同じ。** 途中でやり直すには
   `ros2 topic pub --once /catchrobo/game/reset std_msgs/msg/Empty '{}'` (どの状態からでも `INIT` へ戻る)。
   **アームが動く**ので周囲を空けてから ROS2 を起動すること。
   動かしたくない場合は ROS2 側の `config.yaml` で
   `game_state_manager_node.init_on_startup: false` にしてもらう
5. **追従:** `ros2 topic pub --once /catchrobo/arm/target_pose geometry_msgs/msg/PoseStamped ...` で
   初期位置から数 cm 離れた目標を与え、r/z 基板 (r, z) と θ 基板 (θ) がそれぞれ動くこと
6. **途絶:** ROS2 を止めて 500ms 後に bit2 (ウォッチドッグ) が立ち、位置を保持すること。再開で bit2 が落ちること
7. **バス監視:** ハートビート 5002 の `txQFail` / `canBusOff` / node5 の `hbAge` が悪化しないこと

答え合わせ用の参照実装は ROS2 側の `sharmech_core/scripts/mock_mcu.py` (`--board arm` /
`--board theta` が各基板の 0x81 の埋め方を再現している。§7.1)。
