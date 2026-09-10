# 物理パラメータの微調整手順 (人間向け)

**このロボットの寸法・座標・高さの類は、`sharmech/params/robot_geometry.yaml` 1ファイルだけを
編集し、`python3 sharmech/params/generate.py` で3リポジトリの設定を生成する。**
それ以外の場所に数値を書かない。これが唯一のルールで、以下はその具体的な手順。

2026-09-08 導入。それ以前は同じ値が ROS2 の `config.yaml`・シミュレータの `config.js`・
VR クライアントの `config.js` に別々に手書きされていて、片方だけ直して食い違う
(例: ターンテーブル軸の Y が ROS2 側 0.0 / シム側 0.175) 事故が起きていた。

---

## 0. ファイルの役割 (どれを触ってよいか)

| ファイル | 役割 | 編集 |
|---|---|---|
| `sharmech/params/robot_geometry.yaml` | **正本。** 全ての物理パラメータと、その由来 (`status`/`source`/`date`) | **ここだけ編集する** |
| `sharmech/params/generate.py` | 正本から下の3つを生成するスクリプト。`--check` で最新か確認 | パラメータを**追加**するときだけ (§7) |
| `sharmech_bringup/config/robot_geometry.generated.yaml` | ROS2 各ノードへの上書きパラメータ。launch が `config.yaml` の**後**に読む | **編集禁止** (生成物。git には入れる) |
| `~/catchrobo_sim_threejs/src/generated/robotParams.js` | シミュレータ用 | **編集禁止** (生成物。そのリポジトリの git に入れる) |
| `~/catchrobo_webxr_controller/src/generated/robotParams.js` | VR クライアント用 | **編集禁止** (同上) |
| `sharmech_bringup/config/config.yaml` | 物理量**以外**の設定 (制御周期・ボタン割当・IP・調停モードなど) | 物理量を書き戻さないこと |
| `sharmech/docs/measurement_checklist.md` | 未実測のものを**何をどう測るか** | 実測のとき読む |
| `sharmech/docs/field_dimensions.md` | フィールド座標系の定義と実測の根拠 | 座標系に迷ったら読む |

生成物を直接編集しても、次に `generate.py` を走らせた時点で消える。
逆に、正本を編集して `generate.py` を走らせ忘れると、**ROS2 もシムも VR も古い値のまま動く。**
`generate.py --check` がこれを検出する (§8)。

---

## 1. 値の1行の読み方

```yaml
shooting_box:
  slot_gap_y_m:                 # ← パラメータ名。全体では shooting_box.slot_gap_y_m と呼ぶ
    value: 0.014                # ← 値。m/rad/s の値は必ず小数点付き (0 ではなく 0.0)
    unit: m
    status: decision            # ← この値をどれだけ信じてよいか (下表)
    source: "内寸を均等割りした値"   # ← どこから来た値か
    date: 2026-09-06            # ← その値を入れた日。値を変えたら必ず更新
    note: "★当日調整。..."        # ← 補足
```

| `status` | 意味 | 実機に使ってよいか |
|---|---|---|
| `unmeasured` | 未実測の仮値。ほとんどが 0.0 | **不可。** `generate.py` が毎回一覧を出す。0.0 のままなら IK が解けず動かない、という安全側の性質を意図的に使っている |
| `estimate` | 目安。実測ではないが試運転には使える | 試運転のみ |
| `cad` | フィールド CAD (`~/Documents/catchrobo_docs/field_vol16`) から読んだ値 | 可。ただし現物と違うことがある |
| `measured` | 現物を実測した値 | 可 |
| `fitted` | 実機のデータから最小二乗などで解いた値 | 可 |
| `decision` | 物理量ではなく運用上の決め事 (隙間・余裕・退避高さ) | 可。**当日調整の主対象** |

**`unmeasured` の `value` を「それらしい値」に書き換えてはいけない。** 測るか解くかして、
`status` ごと変える (§4)。AI に作業を頼むときも同じで、AI は `unmeasured` の値を決めない
(CLAUDE.md「勝手に決めてはいけない未決定事項」)。

---

## 2. 手順A: 値を1つ変える (最も基本の流れ)

例: 缶同士が Y 方向に当たるので隙間を 14mm → 20mm にしたい。

```bash
cd ~/catchrobo_ros2_ws/src

# 1. 正本を編集する (value / date を変える。理由があれば note に書く)
$EDITOR sharmech/params/robot_geometry.yaml
#     shooting_box.slot_gap_y_m: value: 0.014 → 0.020, date: 2026-09-08

# 2. 生成する
python3 sharmech/params/generate.py
#     [generate] wrote: .../sharmech_bringup/config/robot_geometry.generated.yaml
#     [generate] wrote: /home/.../catchrobo_sim_threejs/src/generated/robotParams.js
#     [generate] wrote: /home/.../catchrobo_webxr_controller/src/generated/robotParams.js
#     [generate] 注意: 以下は未実測の仮値 ...   ← これは今回の変更と無関係。無視してよい

# 3. 反映する
#    ROS2: 再起動する (--symlink-install なので colcon build は不要)
ros2 launch sharmech_bringup sharmech.launch.xml field_color:=red
#    シミュレータ / VR: ブラウザをリロードする (webpack dev server なら自動)

# 4. 確認する
python3 sharmech/params/generate.py --check      # 3ファイルとも "up to date" になること
ros2 param get /game_state_manager_node slot_gap_y_m   # 0.02 と出ること

# 5. コミットする (3リポジトリそれぞれ)
cd ~/catchrobo_ros2_ws/src/sharmech && git add params/ sharmech_bringup/config/ && git commit -m "slot_gap_y_m を 0.020 に (缶の干渉)"
cd ~/catchrobo_sim_threejs   && git add src/generated/ && git commit -m "robotParams.js を再生成"
cd ~/catchrobo_webxr_controller && git add src/generated/ && git commit -m "robotParams.js を再生成"
```

コミットが3回になるのは、3つが独立したリポジトリだから。生成物をコミットしないと、
他の人がそのリポジトリを clone したときに値が古いままになる。

---

## 3. 手順B: 実行中に試してから確定する (当日・実機での微調整)

`motion_generator_node` と `game_state_manager_node` のパラメータは**再起動せずに**
`ros2 param set` で変えられる (不正値は却下され、直前の設定のまま動き続ける)。
実機を目の前にして「あと 5mm 上」のような調整をするときは、こちらが速い。

```bash
# 1. 実行中に変えて試す (何度でも)
ros2 param set /game_state_manager_node slot_gap_y_m 0.018
ros2 param set /game_state_manager_node slot_gap_y_m 0.020
#    → 起動ログに格子の広がりと箱の内寸の比較が出る。缶を置いて確かめる

# 2. 決まったら、その値を正本に書く   ★ここを忘れると再起動で消える
$EDITOR sharmech/params/robot_geometry.yaml        # value: 0.020, date: 今日
python3 sharmech/params/generate.py

# 3. 次の起動でその値になることを確認する
python3 sharmech/params/generate.py --check
```

**`ros2 param set` は一時的な値であって、正本ではない。** 再起動すると生成物の値に戻る。
「さっき合わせたのに戻った」はこれが原因なので、決まった値はその場で正本に書く。

高さの基準面を実機で合わせる例 (箱の上端に EE を当てて読む):

```bash
# ジョグで EE を箱の上端の高さに合わせてから
ros2 topic echo /catchrobo/arm/current_pose --once     # → position.z を読む (例 0.161)
ros2 param set /game_state_manager_node box_top_z_m 0.161
# 動作を確認したら robot_geometry.yaml の shooting_box.top_z_m を 0.161 にして generate
```

再起動が要るもの (実行中に変えられないもの): `kinematics_node` のリンク長・軸位置
(パターンB。起動時に一度だけ読む)。

---

## 4. 手順C: 未実測 (`unmeasured`) を実測値で埋める

対象は `generate.py` を走らせるたびに出る一覧 (2026-09-08 時点で 10 個。
機構の幾何 9 個 + ワーク中心の高さ 1 個)。**測り方は `measurement_checklist.md`。**

```yaml
# 変更前
  turntable_axis_y_m:
    value: 0.0
    unit: m
    status: unmeasured
    source: "未確認。ベース座標系原点と一致する仮定"

# 変更後 (実測した場合)
  turntable_axis_y_m:
    value: 0.012
    unit: m
    status: measured
    source: "ノギス実測。設置エリア中心の墨出し線からターンテーブル軸中心まで"
    date: 2026-09-15
```

`status` を `measured` (または実機データから解いたなら `fitted`) に変え、`source` に
**測り方**を書く (後で「本当にこの値か」を疑ったとき、測り直せるように)。
`generate.py` → `--check` → `measurement_checklist.md` §5 の検算 (`colcon test`) → コミット。

機構パラメータを埋めたら、**MCU 担当者にも同じ値を渡す** (`mcu_spec.md` §8 #1。
MCU 側も IK/FK に同じ値が要る)。ターンテーブル軸が原点からずれていた場合は、
UDP で送る極座標の原点もずれるので `polar_utils.hpp` の修正が要る (`mcu_spec.md` §9)。

---

## 5. 当日 (競技場) にやること・順番

フィールドは会場で組まれるので、CAD の値と現物がずれる前提で臨む。
ずれの吸収先は「**大きいものから順に**」決めてある。すべて `robot_geometry.yaml`。

| 順 | 症状 | 変えるもの (`robot_geometry.yaml`) | 実行中変更 |
|---|---|---|---|
| 1 | ロボットの設置位置がフィールドに対して全体にずれている | `field_origin_offset.x_m / y_m / z_m` (3ノードへ**同じ値**が生成される) | ○ 両ノードへ同じ値を `ros2 param set` |
| 2 | 箱の並び (位置・数) が想定と違う (ルール上、選手が自由に置ける) | `shooting_box.center_x_red` (配列。長さ = 箱の数)、`center_y_red`。青も同様 | ○ |
| 3 | 箱の上端の高さが CAD と違う | `shooting_box.top_z_m` ← **Z 方向はこれ1個** (離す高さ・退避高さが追従する) | ○ |
| 4 | 缶同士が当たる / 箱からはみ出す | `shooting_box.slot_gap_x_m / slot_gap_y_m` (当たる→増やす、はみ出す→減らす) | ○ |
| 5 | 缶を離す高さが高すぎる/低すぎる | `shooting_box.release_below_top_m` | ○ |
| 6 | 運搬中に何かに当たる | `shooting_box.transport_clearance_above_top_m` (approach / retract も同じ値に) | ○ |
| 7 | ワークが想定位置に無い | perception が検出するので通常は不要。初期配置の外形 (VR の表示範囲) は `work_placement.*` | — (要再生成) |

1 を先にやる理由: 2 以降は 1 で決めた原点の上に乗るため、後から 1 を変えると 2〜6 を
やり直すことになる。

各項目の意味・スロット ID の並びは `sharmech_core/docs/game_state_manager_node.md`。
座標系 (原点・軸の向き) は `field_dimensions.md`。

---

## 6. パラメータ一覧と生成先

`robot_geometry.yaml` のキー → 各リポジトリでの名前。JS 側は sim / webxr で同じファイル内容。

| `robot_geometry.yaml` | ROS2 (生成物 `robot_geometry.generated.yaml`) | JS (`generated/robotParams.js`) | 実行中変更 |
|---|---|---|---|
| `kinematics.shoulder_*` / `knee_*` / `turntable_axis_*` / `knee_base_height_m` | `kinematics_node.<同名>` | `PARALLEL_ARM.shoulder.{pivotHalfSeparation, proximalLinkLength, distalLinkLength}` 等 | × (再起動) |
| `workspace.x_min_m` … `z_max_m` | `motion_generator_node.workspace_x_min` … | `WORKSPACE.{xMin … zMax}` | ○ |
| `cylinder.radius_m` / `length_m` | `game_state_manager_node.cylinder_diameter_m` (= 2 × radius) | `CYLINDER.{radius, length}`、`REAL_FIELD` (派生) | ○ |
| `shooting_box.center_x_red` 等 | `game_state_manager_node.box_center_x_red` 等 | `SHOOTING_BOX.centerXRed` 等 | ○ |
| `shooting_box.inner_size_*`, `slot_cols_x`, `slot_rows_y`, `slot_gap_*` | `box_inner_size_*_m`, `slot_cols_x`, `slot_rows_y`, `slot_gap_*_m` | `SHOOTING_BOX.*` | ○ |
| `shooting_box.top_z_m`, `release_below_top_m`, `*_clearance_above_top_m` | `box_top_z_m`, `slot_release_below_box_top_m`, `*_clearance_above_box_top_m` | `SHOOTING_BOX.*` | ○ |
| `work_placement.*` | `field_geometry.*` (読むノードは無い) | `WORK_PLACEMENT.*`、`REAL_FIELD` (派生: 外形 = 初期配置 + 円柱寸法) | — |
| `field_origin_offset.x_m / y_m / z_m` | `motion_generator_node`・`game_state_manager_node`・`field_geometry` の `field_origin_offset_*_m` (同じ値) | `FIELD_ORIGIN_OFFSET.{x,y,z}` | ○ |
| (全キー) | — | `STATUS['section.param']`、`isPlaceholder(key)` | — |

`REAL_FIELD` (VR がワークエリアを表示する外形) は手で入れる値ではなく、初期配置と円柱半径
からの派生値として生成される。以前は半径を変えると `width` も手計算で直す必要があった。

---

## 7. パラメータを新しく追加する

物理量が1つ増えたとき (例: グリッパの開き幅)。順番に:

1. **`robot_geometry.yaml` に項目を足す** (`value / unit / status / source / date`)。
   単位が m/rad/s なら小数点付きで書く
2. **`generate.py` の対応表に足す**
   - ROS2 で使う → `ros_sections()` の該当ノードのリストに `("ros側の名前", p.get("section.param"))` を1行
   - JS で使う → `render_js()` の該当 `export const` に1行
3. **受け側にパラメータを宣言する** (ROS2 なら `declare_parameter`。JS なら `generated/robotParams.js` から import)
4. `python3 sharmech/params/generate.py` → `--check` → ビルド・テスト
5. §6 の一覧表と、そのパラメータを説明するノードの `docs/*.md` に追記する

`generate.py` の対応表に無いキーは、正本に書いても**どこにも出ていかない**
(警告も出ない)。追加時は 2 を忘れないこと。

---

## 8. 確認コマンド (何かおかしいと思ったらまずこれ)

```bash
cd ~/catchrobo_ros2_ws/src

# 生成物が正本と一致しているか (1つでも古ければ exit 1 と STALE の一覧)
python3 sharmech/params/generate.py --check

# ROS2 が実際にどの値で動いているか (生成物と同じはず)
ros2 param get /motion_generator_node workspace_x_min
ros2 param get /game_state_manager_node box_top_z_m
ros2 param dump /game_state_manager_node          # 全部

# 正本の書式が壊れていないか (generate.py は書式が不正なら何も書かずに exit 1)
python3 sharmech/params/generate.py --check      # "ERROR: ... が不正:" と原因の一覧
```

生成物が無い状態で launch すると `No such file or directory: .../robot_geometry.generated.yaml`
で**起動しない** (黙って C++ の既定値で動くことはない)。JS 側も import に失敗する。
「起動しなくなった」は生成し忘れをまず疑う。

---

## 9. よくある間違い

| 間違い | 何が起きるか | 正しくは |
|---|---|---|
| `config.yaml` に数値を書く | 生成物が後から読まれて上書きするので**効かない** | `robot_geometry.yaml` に書く |
| 生成物 (`*.generated.yaml` / `generated/robotParams.js`) を直接編集 | 次の `generate.py` で消える。3つの値がずれる | 同上 |
| 正本を編集して `generate.py` を走らせない | 全部が古い値のまま | 編集したら必ず生成。`--check` で確認 |
| `ros2 param set` で合わせて満足する | 再起動で戻る | 決まった値を正本に書く (§3) |
| `value: 0` (小数点なし) | ROS2 が整数と解釈し、double のパラメータに入らず起動失敗 | `0.0`。`generate.py` が書式エラーで止めてくれる |
| ピボット間距離をそのまま `pivot_half_separation_m` に入れる | リンクの幾何が崩れ IK が解けない/到達範囲が半分になる | **÷2** (`measurement_checklist.md` §1) |
| `unmeasured` の値を「それらしい値」に変える | 未実測なのに実機で使える値に見える | 測るか解く。`status` も変える (§4) |
| 生成物をコミットしない | 他の人の環境・clone 先で古い値になる | 3リポジトリでそれぞれコミット (§2) |
| 片方のリポジトリだけ再生成する | `generate.py` は3つ同時に書くので通常起きないが、他リポジトリが無い環境では skip される | `generate.py` のログに `skip` が出ていないか見る |

---

## 10. AI (Claude Code) に頼むときの言い方

このワークスペースで作業する Claude Code は CLAUDE.md からこの仕組みを知っているので、
**変えたい値と理由**だけ伝えれば、正本の編集 → 生成 → `--check` → 確認まで自分でやる。

```
robot_geometry.yaml の shooting_box.slot_gap_y_m を 0.020 にして (缶が当たるため)、生成して check まで通して
```

```
ターンテーブル軸の位置を実測した。x=0.000, y=0.012。robot_geometry.yaml に measured で入れて生成し、
測り方は「ノギス。設置エリア中心の墨出し線から軸中心まで」
```

**AI に「それらしい値を決めて」とは言わない。** `unmeasured` の値を AI が決めることは
CLAUDE.md で禁止してあり、断られる。値は人が測るか、実機のデータから解く
(将来はフィッティングのスクリプトをここに足す予定)。

3リポジトリ間で数値が合っているか不安なときは `/integration_check` が
`generate.py --check` から始める手順になっている。
