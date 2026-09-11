#!/usr/bin/env python3
"""
robot_geometry.yaml (正本) から各リポジトリ向けの設定ファイルを生成する。

生成先:
  <このリポジトリ>/sharmech_bringup/config/robot_geometry.generated.yaml
  ~/catchrobo_sim_threejs/src/generated/robotParams.js
  ~/catchrobo_webxr_controller/src/generated/robotParams.js

使い方:
  python3 sharmech/params/generate.py            # 生成して書き込む
  python3 sharmech/params/generate.py --check    # 生成物が正本と一致しているか確認する (書き込まない)
  python3 sharmech/params/generate.py --ros-only # 他リポジトリには書かない

ROS2 に依存しない (PyYAML のみ)。手順の全体は sharmech/docs/parameter_tuning.md。

--check の終了コード: 0 = 全て最新 / 1 = 生成物が古い・無い、または正本が不正。
生成物が無い場合 ROS2 側は launch が失敗し (<param from> が無いファイルを指す)、
JS 側は import に失敗するので、黙って古い値で動くことはない。
"""
import argparse
import os
import sys
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parents[1]          # .../src/sharmech
SOURCE = REPO_ROOT / "params" / "robot_geometry.yaml"
ROS_OUT = REPO_ROOT / "sharmech_bringup" / "config" / "robot_geometry.generated.yaml"
JS_OUTS = [
    Path.home() / "catchrobo_sim_threejs" / "src" / "generated" / "robotParams.js",
    Path.home() / "catchrobo_webxr_controller" / "src" / "generated" / "robotParams.js",
]

VALID_STATUS = {"unmeasured", "estimate", "cad", "measured", "fitted", "decision"}
FLOAT_UNITS = {"m", "rad", "s"}
HEADER_NOTE = (
    "このファイルは自動生成 (sharmech/params/generate.py)。直接編集しないこと。\n"
    "値を変えるときは sharmech/params/robot_geometry.yaml を編集して再生成する。\n"
    "手順: sharmech/docs/parameter_tuning.md"
)


# ---------------------------------------------------------------------------
# 読み込みと検証
# ---------------------------------------------------------------------------
class Params:
    """正本を読み、`get('section.param')` で値を、`status()` で信頼度を返す。"""

    def __init__(self, path: Path):
        with open(path, encoding="utf-8") as f:
            self.raw = yaml.safe_load(f)
        if self.raw.get("schema_version") != 1:
            raise ValueError(f"{path}: schema_version が 1 ではない")
        self.errors = []
        self.unmeasured = []
        self._validate()

    def _validate(self):
        for section, params in self.raw.items():
            if section == "schema_version":
                continue
            if not isinstance(params, dict):
                self.errors.append(f"{section}: セクションが辞書ではない")
                continue
            for name, entry in params.items():
                key = f"{section}.{name}"
                if not isinstance(entry, dict) or "value" not in entry:
                    self.errors.append(f"{key}: value が無い")
                    continue
                status = entry.get("status")
                if status not in VALID_STATUS:
                    self.errors.append(
                        f"{key}: status '{status}' は不正 ({'/'.join(sorted(VALID_STATUS))})")
                unit = entry.get("unit")
                if unit is None:
                    self.errors.append(f"{key}: unit が無い")
                if "source" not in entry:
                    self.errors.append(f"{key}: source が無い (由来を書くこと)")
                self._check_numeric(key, entry["value"], unit)
                if status == "unmeasured":
                    self.unmeasured.append(key)

    def _check_numeric(self, key, value, unit):
        # ROS2 は 0 を整数、0.0 を double と解釈する。double 宣言のパラメータに整数を
        # 渡すと起動に失敗するので、m/rad/s の値は小数点付きを強制する
        values = value if isinstance(value, list) else [value]
        for v in values:
            if isinstance(v, bool) or not isinstance(v, (int, float)):
                self.errors.append(f"{key}: 数値ではない ({v!r})")
            elif unit in FLOAT_UNITS and isinstance(v, int):
                self.errors.append(
                    f"{key}: 単位 {unit} の値 {v} は小数点付きで書くこと ({float(v)!r})")
            elif unit == "count" and isinstance(v, float):
                self.errors.append(f"{key}: 単位 count の値 {v} は整数で書くこと")

    def get(self, key: str):
        section, name = key.split(".")
        return self.raw[section][name]["value"]

    def status(self, key: str) -> str:
        section, name = key.split(".")
        return self.raw[section][name]["status"]

    def keys(self):
        for section, params in self.raw.items():
            if section == "schema_version":
                continue
            for name in params:
                yield f"{section}.{name}"


# ---------------------------------------------------------------------------
# 書式
# ---------------------------------------------------------------------------
def fmt(v) -> str:
    """YAML/JS 共通の数値リテラル。float は必ず小数点付き、list は [a, b]。"""
    if isinstance(v, list):
        return "[" + ", ".join(fmt(x) for x in v) + "]"
    if isinstance(v, float):
        return repr(v)
    return str(v)


def comment_block(text: str, prefix: str) -> str:
    return "\n".join(f"{prefix} {line}".rstrip() for line in text.splitlines()) + "\n"


# ---------------------------------------------------------------------------
# ROS2 (各ノードの ros__parameters 上書き)
# ---------------------------------------------------------------------------
# 正本のキー → ROS2 パラメータ名。ノードごとにまとめる。
# 値の変換が要るものは (key, 変換関数) で書く。
def ros_sections(p: Params):
    origin = [
        ("field_origin_offset_x_m", p.get("field_origin_offset.x_m")),
        ("field_origin_offset_y_m", p.get("field_origin_offset.y_m")),
        ("field_origin_offset_z_m", p.get("field_origin_offset.z_m")),
    ]
    return [
        ("field_geometry", [
            ("normal_work_rows", p.get("work_placement.normal_rows")),
            ("normal_work_cols", p.get("work_placement.normal_cols")),
            ("common_work_count", p.get("work_placement.common_count")),
            ("normal_work_pitch_x_m", p.get("work_placement.normal_pitch_x_m")),
            ("normal_work_pitch_y_m", p.get("work_placement.normal_pitch_y_m")),
            ("common_work_pitch_m", p.get("work_placement.common_pitch_m")),
            ("first_work_x_m", p.get("work_placement.first_x_m")),
            ("first_work_y_m", p.get("work_placement.first_y_m")),
            ("common_work_x_m", p.get("work_placement.common_x_m")),
            ("common_work_first_y_m", p.get("work_placement.common_first_y_m")),
            ("first_work_z_m", p.get("work_placement.first_z_m")),
        ] + origin[:2]),
        ("motion_generator_node", [
            ("workspace_x_min", p.get("workspace.x_min_m")),
            ("workspace_x_max", p.get("workspace.x_max_m")),
            ("workspace_y_min", p.get("workspace.y_min_m")),
            ("workspace_y_max", p.get("workspace.y_max_m")),
            ("workspace_z_min", p.get("workspace.z_min_m")),
            ("workspace_z_max", p.get("workspace.z_max_m")),
        ] + origin),
        ("game_state_manager_node", [
            ("box_center_x_red", p.get("shooting_box.center_x_red")),
            ("box_center_y_red", p.get("shooting_box.center_y_red")),
            ("box_center_x_blue", p.get("shooting_box.center_x_blue")),
            ("box_center_y_blue", p.get("shooting_box.center_y_blue")),
            ("box_inner_size_x_m", p.get("shooting_box.inner_size_x_m")),
            ("box_inner_size_y_m", p.get("shooting_box.inner_size_y_m")),
            ("slot_cols_x", p.get("shooting_box.slot_cols_x")),
            ("slot_rows_y", p.get("shooting_box.slot_rows_y")),
            # 缶の直径は半径の2倍。丸めは float の表現誤差を消すため
            ("cylinder_diameter_m", round(2.0 * p.get("cylinder.radius_m"), 6)),
            ("slot_gap_x_m", p.get("shooting_box.slot_gap_x_m")),
            ("slot_gap_y_m", p.get("shooting_box.slot_gap_y_m")),
            ("box_top_z_m", p.get("shooting_box.top_z_m")),
            ("slot_release_below_box_top_m", p.get("shooting_box.release_below_top_m")),
            ("approach_clearance_above_box_top_m",
             p.get("shooting_box.approach_clearance_above_top_m")),
            ("transport_clearance_above_box_top_m",
             p.get("shooting_box.transport_clearance_above_top_m")),
            ("retract_clearance_above_box_top_m",
             p.get("shooting_box.retract_clearance_above_top_m")),
            # 掴みに降りる先の絶対 z (pick_request の z を常に上書き)
            ("pick_z_m", p.get("work_placement.pick_z_m")),
            # 初期位置 (起動時と /catchrobo/game/reset の行き先)。**極座標のまま渡し**、
            # ノード側が turntable_axis_* を原点として直交座標へ直す
            ("init_pose_r_red", p.get("init_pose.r_red")),
            ("init_pose_theta_red", p.get("init_pose.theta_red")),
            ("init_pose_z_red", p.get("init_pose.z_red")),
            ("init_pose_r_blue", p.get("init_pose.r_blue")),
            ("init_pose_theta_blue", p.get("init_pose.theta_blue")),
            ("init_pose_z_blue", p.get("init_pose.z_blue")),
            # 上の極座標の原点。hardware_bridge_node へ渡すのと同じ値
            ("turntable_axis_x_m", p.get("kinematics.turntable_axis_x_m")),
            ("turntable_axis_y_m", p.get("kinematics.turntable_axis_y_m")),
        ] + origin),
        # UDP 極座標 (r, θ) の原点 = ターンテーブル軸。送受信の両方でこの値を使う
        ("hardware_bridge_node", [
            ("turntable_axis_x_m", p.get("kinematics.turntable_axis_x_m")),
            ("turntable_axis_y_m", p.get("kinematics.turntable_axis_y_m")),
        ]),
        ("kinematics_node", [
            (name, p.get(f"kinematics.{name}")) for name in (
                "shoulder_pivot_half_separation_m",
                "shoulder_proximal_link_length_m",
                "shoulder_distal_link_length_m",
                "knee_pivot_half_separation_m",
                "knee_proximal_link_length_m",
                "knee_distal_link_length_m",
                "turntable_axis_x_m",
                "turntable_axis_y_m",
                "knee_base_height_m",
            )
        ]),
    ]


def render_ros(p: Params) -> str:
    out = comment_block(HEADER_NOTE, "#")
    out += "#\n# sharmech.launch.xml が config.yaml の後に読み込み、同名パラメータを上書きする。\n"
    out += "# 各パラメータの意味は robot_geometry.yaml の note と各ノードの docs/*.md を参照。\n"
    for node, items in ros_sections(p):
        out += f"\n{node}:\n  ros__parameters:\n"
        for name, value in items:
            out += f"    {name}: {fmt(value)}\n"
    return out


# ---------------------------------------------------------------------------
# JS (シミュレータ・VR クライアント共通。同じ内容を両方に置く)
# ---------------------------------------------------------------------------
def render_js(p: Params) -> str:
    r = p.get("cylinder.radius_m")
    length = p.get("cylinder.length_m")
    cols = p.get("work_placement.normal_cols")
    pitch_y = p.get("work_placement.normal_pitch_y_m")
    first_x = p.get("work_placement.first_x_m")
    first_y = p.get("work_placement.first_y_m")
    common_x = p.get("work_placement.common_x_m")
    # webxr の REAL_FIELD (ワークエリアの外形) は初期配置 + 円柱寸法から決まる派生値。
    # 以前は手計算で入れていて、半径を変えると width も手で直す必要があった
    real_field = {
        "xMin": round(first_x - length / 2.0, 6),
        "length": round((common_x - first_x) + length, 6),
        "yMin": round(first_y + (cols - 1) * pitch_y - r, 6),
        "width": round(abs((cols - 1) * pitch_y) + 2.0 * r, 6),
    }

    def obj(pairs, indent="  "):
        close = indent[:-2]
        return "{\n" + "".join(f"{indent}{k}: {fmt(v)},\n" for k, v in pairs) + close + "}"

    kin = {
        "shoulder": obj([
            ("pivotHalfSeparation", p.get("kinematics.shoulder_pivot_half_separation_m")),
            ("proximalLinkLength", p.get("kinematics.shoulder_proximal_link_length_m")),
            ("distalLinkLength", p.get("kinematics.shoulder_distal_link_length_m")),
        ], "    "),
        "knee": obj([
            ("pivotHalfSeparation", p.get("kinematics.knee_pivot_half_separation_m")),
            ("proximalLinkLength", p.get("kinematics.knee_proximal_link_length_m")),
            ("distalLinkLength", p.get("kinematics.knee_distal_link_length_m")),
        ], "    "),
    }

    status_lines = "".join(f"  '{k}': '{p.status(k)}',\n" for k in p.keys())

    return f"""{comment_block(HEADER_NOTE, '//')}//
// 正本 robot_geometry.yaml の値をそのまま写したもの。単位は m / rad (ROS 標準)。
// STATUS が 'unmeasured' の値は仮値であり、表示用にはそれぞれのリポジトリ側で
// プレースホルダに置き換えてよい (isPlaceholder を使う)。実機を動かす値には使わないこと。

/** 5軸パラレルリンクの幾何 (ROS2 kinematics_node と同じ値) */
export const PARALLEL_ARM = {{
  shoulder: {kin['shoulder']},
  knee: {kin['knee']},
  turntableAxisX: {fmt(p.get('kinematics.turntable_axis_x_m'))},
  turntableAxisY: {fmt(p.get('kinematics.turntable_axis_y_m'))},
  kneeBaseHeight: {fmt(p.get('kinematics.knee_base_height_m'))},
}};

/** 作業領域 (motion_generator_node のクランプ範囲と同じ値) */
export const WORKSPACE = {obj([
    ("xMin", p.get("workspace.x_min_m")), ("xMax", p.get("workspace.x_max_m")),
    ("yMin", p.get("workspace.y_min_m")), ("yMax", p.get("workspace.y_max_m")),
    ("zMin", p.get("workspace.z_min_m")), ("zMax", p.get("workspace.z_max_m")),
])};

/** ワーク (円柱) の寸法 */
export const CYLINDER = {obj([("radius", r), ("length", length)])};

/**
 * ワークエリアの外形 (webxr の REAL_FIELD 相当。field 座標系)。
 * 初期配置 (work_placement) と円柱寸法からの派生値
 */
export const REAL_FIELD = {obj(list(real_field.items()))};

/** ワークの初期配置 (field 座標系) */
export const WORK_PLACEMENT = {obj([
    ("normalRows", p.get("work_placement.normal_rows")),
    ("normalCols", p.get("work_placement.normal_cols")),
    ("commonCount", p.get("work_placement.common_count")),
    ("normalPitchX", p.get("work_placement.normal_pitch_x_m")),
    ("normalPitchY", p.get("work_placement.normal_pitch_y_m")),
    ("commonPitch", p.get("work_placement.common_pitch_m")),
    ("firstX", first_x), ("firstY", first_y),
    ("commonX", common_x), ("commonFirstY", p.get("work_placement.common_first_y_m")),
    ("firstZ", p.get("work_placement.first_z_m")),
])};

/** シューティングボックス */
export const SHOOTING_BOX = {obj([
    ("centerXRed", p.get("shooting_box.center_x_red")),
    ("centerYRed", p.get("shooting_box.center_y_red")),
    ("centerXBlue", p.get("shooting_box.center_x_blue")),
    ("centerYBlue", p.get("shooting_box.center_y_blue")),
    ("innerSizeX", p.get("shooting_box.inner_size_x_m")),
    ("innerSizeY", p.get("shooting_box.inner_size_y_m")),
    ("slotColsX", p.get("shooting_box.slot_cols_x")),
    ("slotRowsY", p.get("shooting_box.slot_rows_y")),
    ("slotGapX", p.get("shooting_box.slot_gap_x_m")),
    ("slotGapY", p.get("shooting_box.slot_gap_y_m")),
    ("topZ", p.get("shooting_box.top_z_m")),
    ("releaseBelowTop", p.get("shooting_box.release_below_top_m")),
    ("approachClearanceAboveTop", p.get("shooting_box.approach_clearance_above_top_m")),
    ("transportClearanceAboveTop", p.get("shooting_box.transport_clearance_above_top_m")),
    ("retractClearanceAboveTop", p.get("shooting_box.retract_clearance_above_top_m")),
])};

/** 本番設置での原点ズレ補正 */
export const FIELD_ORIGIN_OFFSET = {obj([
    ("x", p.get("field_origin_offset.x_m")),
    ("y", p.get("field_origin_offset.y_m")),
    ("z", p.get("field_origin_offset.z_m")),
])};

/** 各値の信頼度 (robot_geometry.yaml の status)。キーは 'section.param' */
export const STATUS = {{
{status_lines}}};

/** その値が未実測の仮値 (実機には使えない) なら true */
export function isPlaceholder(key) {{
  return STATUS[key] === 'unmeasured';
}}
"""


# ---------------------------------------------------------------------------
# 実行
# ---------------------------------------------------------------------------
def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--check", action="store_true",
                    help="書き込まず、生成物が正本と一致しているかだけ確認する")
    ap.add_argument("--ros-only", action="store_true",
                    help="他リポジトリ (sim/webxr) には書かない")
    args = ap.parse_args()

    try:
        p = Params(SOURCE)
    except Exception as e:  # noqa: BLE001
        print(f"[generate] ERROR: {SOURCE} を読めない: {e}", file=sys.stderr)
        return 1
    if p.errors:
        print(f"[generate] ERROR: {SOURCE.relative_to(REPO_ROOT.parent)} が不正:",
              file=sys.stderr)
        for e in p.errors:
            print(f"  - {e}", file=sys.stderr)
        return 1

    targets = [(ROS_OUT, render_ros(p))]
    if not args.ros_only:
        js = render_js(p)
        for out in JS_OUTS:
            if out.parents[1].is_dir():          # <repo>/src が存在するときだけ
                targets.append((out, js))
            else:
                print(f"[generate] skip (リポジトリが無い): {out}", file=sys.stderr)

    stale = []
    for path, content in targets:
        current = path.read_text(encoding="utf-8") if path.exists() else None
        if current == content:
            print(f"[generate] up to date: {path}")
            continue
        if args.check:
            stale.append(path)
            print(f"[generate] STALE: {path} ({'無い' if current is None else '正本と不一致'})")
        else:
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(content, encoding="utf-8")
            print(f"[generate] wrote: {path}")

    if p.unmeasured:
        print("[generate] 注意: 以下は未実測の仮値 (status: unmeasured)。実機には使えない:")
        for k in p.unmeasured:
            print(f"  - {k} = {fmt(p.get(k))}")

    if args.check and stale:
        print(f"[generate] {len(stale)} 件が古い。`python3 {os.path.relpath(__file__)}` で再生成すること",
              file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
