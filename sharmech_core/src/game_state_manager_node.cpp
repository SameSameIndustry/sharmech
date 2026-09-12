#include "sharmech_core/game_state_manager_node.hpp"
#include "sharmech_core/utility/orientation_utils.hpp"
#include "sharmech_core/utility/polar_utils.hpp"

#include <rclcpp_components/register_node_macro.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <cmath>
#include <stdexcept>
#include <string>

namespace sharmech_core
{

namespace
{

geometry_msgs::msg::PoseStamped toPoseStampedMsg(const CartesianState & s)
{
  geometry_msgs::msg::PoseStamped msg;
  msg.header.frame_id = "field";
  msg.pose.position.x = s.x;
  msg.pose.position.y = s.y;
  msg.pose.position.z = s.z;
  tf2::Quaternion q;
  q.setRPY(0.0, s.pitch, s.yaw);
  msg.pose.orientation = tf2::toMsg(q);
  return msg;
}

}  // namespace

GameStateManagerNode::GameStateManagerNode(const rclcpp::NodeOptions & options)
: Node("game_state_manager_node", options)
{
  // フィールドの色によってシューティングボックスのスロット座標が変わるため、
  // 独断で既定値を決めず launch 引数での明示指定を必須にする
  // (config.yaml 側にも既定値を置かない。sharmech.launch.xml の field_color 引数を参照)
  const std::string field_color = declare_parameter("field_color", std::string(""));
  if (field_color != "red" && field_color != "blue") {
    RCLCPP_FATAL(
      get_logger(),
      "field_color must be \"red\" or \"blue\" (got \"%s\"). "
      "launch引数 field_color:=red|blue で明示してください",
      field_color.c_str());
    throw std::invalid_argument("field_color must be 'red' or 'blue'");
  }

  field_color_ = field_color;
  declareParameters(field_color);

  // 起動時は「上書き無し」で組み立てる。ここで throw する不整合 (placement_order が
  // 空/範囲外など) は起動を止める。実行中の ros2 param set では throw せず
  // reason を返して却下する (下記 onSetParameters)
  GameStateMachine::Config config = buildConfig({});
  machine_ = std::make_unique<GameStateMachine>(config);

  // 実行中のパラメータ変更を即反映する (再起動なしで現場合わせできるようにする)。
  // Humble には post-set コールバックが無いため、この on-set コールバックの中で
  // 検証と適用の両方を行う
  param_callback_handle_ = add_on_set_parameters_callback(
    std::bind(&GameStateManagerNode::onSetParameters, this, std::placeholders::_1));

  pick_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "/catchrobo/game/pick_request", 10,
    std::bind(&GameStateManagerNode::onPickRequest, this, std::placeholders::_1));
  // VRが仮想フィールドの指定箱にワークを離すたびに届く通算個数 (1始まり)。
  // 「置きに行くべきスロット座標のキュー」として扱う (下記 onBoxCount)
  box_count_sub_ = create_subscription<std_msgs::msg::Int32>(
    "/catchrobo/game/box_count", 10,
    std::bind(&GameStateManagerNode::onBoxCount, this, std::placeholders::_1));
  status_sub_ = create_subscription<sharmech_msgs::msg::MotionStatus>(
    "/catchrobo/arm/status", rclcpp::QoS(10).transient_local(),   // depth は publisher 側と揃える
    std::bind(&GameStateManagerNode::onArmStatus, this, std::placeholders::_1));

  // デバッグ用。任意のステートへ飛ばして、その状態の振る舞いだけを確認できる
  change_state_sub_ = create_subscription<std_msgs::msg::String>(
    "/catchrobo/debug/change_state", 10,
    std::bind(&GameStateManagerNode::onChangeStateRequest, this, std::placeholders::_1));

  // VRが使えない場合の脱出ハッチ。joy_teleop_node がDualSenseの特定ボタン
  // 同時押し(L1+R1+L3+R3)を検知して publish する
  // 微調整の確定。PS4の確定ボタン (joy_teleop_node) と VR のサムズアップが
  // どちらもここへ publish する契約
  confirm_sub_ = create_subscription<std_msgs::msg::Empty>(
    "/catchrobo/game/confirm", 10,
    std::bind(&GameStateManagerNode::onConfirm, this, std::placeholders::_1));

  // 微調整で操縦者がジョグした結果を追うため、現在の目標姿勢を購読する。
  // 100Hz だが姿勢を1つ保持するだけなので負荷は無視できる
  command_cartesian_sub_ = create_subscription<sharmech_msgs::msg::CartesianCommand>(
    "/catchrobo/command/cartesian", 10,
    std::bind(&GameStateManagerNode::onCommandCartesian, this, std::placeholders::_1));

  toggle_manual_control_sub_ = create_subscription<std_msgs::msg::Empty>(
    "/catchrobo/game/toggle_manual_control", 10,
    std::bind(&GameStateManagerNode::onToggleManualControl, this, std::placeholders::_1));

  // 状態のリセット要求。VR (メニューのボタン) から届く。
  // どの状態からでも INIT へ入り、初期位置へ戻してから待機状態に復帰する
  reset_sub_ = create_subscription<std_msgs::msg::Empty>(
    "/catchrobo/game/reset", 10,
    std::bind(&GameStateManagerNode::onResetRequest, this, std::placeholders::_1));

  target_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
    "/catchrobo/arm/target_pose", 10);
  gripper_pub_ = create_publisher<std_msgs::msg::Bool>(
    "/catchrobo/arm/gripper", 10);
  orient_vertical_pub_ = create_publisher<std_msgs::msg::Bool>(
    "/catchrobo/arm/orient_vertical", 10);
  workspace_clamp_pub_ = create_publisher<sharmech_msgs::msg::WorkspaceClamp>(
    "/catchrobo/game/workspace_clamp", 10);
  jog_limit_pub_ = create_publisher<sharmech_msgs::msg::JogLimit>(
    "/catchrobo/game/jog_limit", 10);
  // INIT の z 速度上限を ros2 param set 相当で motion_generator_node へ入れる (syncInitSpeedLimit)
  motion_params_client_ = std::make_shared<rclcpp::AsyncParametersClient>(
    this, "motion_generator_node");
  // latched: 後から接続したVRクライアント・観測用ダッシュボードにも現在状態が即座に届く
  state_pub_ = create_publisher<std_msgs::msg::String>(
    "/catchrobo/game/state", rclcpp::QoS(1).transient_local());

  const double state_publish_rate = get_parameter("state_publish_rate").as_double();
  const auto period = std::chrono::duration<double>(1.0 / state_publish_rate);
  timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&GameStateManagerNode::onTimer, this));

  RCLCPP_INFO(
    get_logger(),
    "game_state_manager_node started (field_color=%s, %zu slots, %zu-step placement_order, "
    "init_pose=(%.3f, %.3f, %.3f), init_on_startup=%s)",
    field_color.c_str(), config.slots.size(), config.placement_order.size(),
    config.init_pose.x, config.init_pose.y, config.init_pose.z,
    config.init_on_startup ? "true" : "false");
  logSlotGeometry(config, {});
}

// パラメータの宣言。値の取得は buildConfig() 側で行い、ここでは既定値だけを与える
// (実行中の ros2 param set で組み直せるよう、宣言と読み取りを分けてある)
void GameStateManagerNode::declareParameters(const std::string & color_suffix)
{
  declare_parameter("placement_order", std::vector<int64_t>{});
  declare_parameter("state_publish_rate", 10.0);

  // --- 本番設置での原点ズレ補正 (フィールド全体を平行移動する) ---
  // sharmech/docs/field_dimensions.md 参照。motion_generator_node と同じ意味・同じ値を
  // 使う想定 (現場合わせで両方に入れる)。Z は箱の高さ基準ごと持ち上げ下げしたいとき用
  declare_parameter("field_origin_offset_x_m", 0.0);
  declare_parameter("field_origin_offset_y_m", 0.0);
  declare_parameter("field_origin_offset_z_m", 0.0);

  // --- 箱の位置 (箱を動かしたらここを変える) ---
  // 各箱の中心X [m]。長さがそのまま箱の数になる
  declare_parameter("box_center_x_" + color_suffix, std::vector<double>{});
  // 箱の中心Y [m] (全箱共通)
  declare_parameter("box_center_y_" + color_suffix, 0.0);

  // --- 箱の内寸 (はみ出し警告に使うだけで、座標計算には使わない) ---
  declare_parameter("box_inner_size_x_m", 0.138);
  declare_parameter("box_inner_size_y_m", 0.255);

  // --- 箱内のスロット格子 ---
  declare_parameter("slot_cols_x", 2);   // X方向(箱の短辺)の列数
  declare_parameter("slot_rows_y", 3);   // Y方向(箱の長辺)の行数
  declare_parameter("cylinder_diameter_m", 0.071);
  // ★当日ここを調整する。隣り合う缶の「隙間」[m] (中心間距離 = 直径 + 隙間)。
  // 缶同士が当たるなら増やす。箱からはみ出すなら減らす (負値も許す = 干渉している状態)
  declare_parameter("slot_gap_x_m", -0.002);
  declare_parameter("slot_gap_y_m", 0.014);

  // --- 高さ (基準面からの相対で与える) ---
  // 当日実測するのはこの box_top_z_m 1個だけで済むようにしてある
  declare_parameter("box_top_z_m", 0.156);
  declare_parameter("slot_release_below_box_top_m", 0.106);
  declare_parameter("approach_clearance_above_box_top_m", 0.044);
  declare_parameter("transport_clearance_above_box_top_m", 0.044);
  declare_parameter("retract_clearance_above_box_top_m", 0.044);
  // 掴みに降りる先の絶対 z。pick_request の z は常にこれで上書きする
  // (VR は缶オブジェクトの原点 = 底面 z=0 を送ってくるため。2026-09-11 ユーザー指示)
  declare_parameter("pick_z_m", 0.20);

  declare_parameter("slot_clamp_margin_m", 0.03);
  declare_parameter("require_manual_confirm", true);
  // 微調整中 (ADJUSTING_PICK / ADJUSTING_PLACE) のジョグ速度上限 [m/s]。
  // この2状態の間だけ motion_generator_node へ /catchrobo/game/jog_limit で伝え、
  // 抜けたら reset を送って起動時の上限へ戻す。
  //
  // **掴む/離す直前は缶に一番近く、行き過ぎるとワーク破損 (競技で-1点) に直結する。**
  // 既定 0.05 は motion_generator_node の v_max (0.10) の半分。
  // a_max=0.2 なので 0.25 秒で上限に達し、以降は等速になる —— 「倒した時間に
  // 比例して進む」予測しやすい挙動になり、離した後の滑りも 6mm 程度に収まる
  // (上限を上げると、倒し続けている間ずっと加速し続けて行き過ぎやすくなる)。
  // 実機の手感に合わせて ros2 param set で調整すること
  declare_parameter("adjusting_jog_v_max", 0.05);
  declare_parameter("grasp_dwell_sec", 0.3);
  declare_parameter("orient_dwell_sec", 0.5);

  // --- 初期位置 (起動時・reset・フィードバック途絶で戻る先) ---
  // robot_geometry.yaml の init_pose から生成される。UDP と同じ極座標
  // (原点 = ターンテーブル軸、θ は +X から時計回り正、z はベース座標系) で赤・青別に持ち、
  // buildConfig() が turntable_axis_x/y_m を原点として直交座標へ直す
  declare_parameter("init_pose_r_" + color_suffix, 0.0);
  declare_parameter("init_pose_theta_" + color_suffix, 0.0);
  declare_parameter("init_pose_z_" + color_suffix, 0.0);
  declare_parameter("turntable_axis_x_m", 0.0);
  declare_parameter("turntable_axis_y_m", 0.0);
  // true: 起動直後 INIT から始まり、MCU 同期後に初期位置へ動く。起動時にだけ読む
  declare_parameter("init_on_startup", true);
  // INIT で動作許可が出てから初期位置へのゴールを出すまでの待ち [s]
  declare_parameter("init_delay_sec", 3.0);
  // INIT (初期位置への直線 1 本) の間だけ motion_generator_node の v_max_z に入れる
  // z 速度の上限 [m/s]。肘/膝機構は可動上限 (workspace z_max 0.2098) の近くを動くので、
  // 初期位置へ戻るときだけ z をゆっくりにする (ユーザー指示 2026-09-12)。r/θ はそのまま。
  // 0 なら z の別上限なし。INIT を抜けたら motion_generator_node 側を 0 に戻す
  declare_parameter("init_v_max_z", 0.02);
}

// overrides に載っているものはそ担ってるよねの値を、載っていないものは現在値を使って設定を組む。
// overrides は ros2 param set の「これから設定される値」(Humble には post-set
// コールバックが無いため、適用前の値をここで先取りして組み立てる)
GameStateMachine::Config GameStateManagerNode::buildConfig(
  const std::vector<rclcpp::Parameter> & overrides) const
{
  const auto find = [&overrides](const std::string & name) -> const rclcpp::Parameter * {
      for (const auto & p : overrides) {
        if (p.get_name() == name) {return &p;}
      }
      return nullptr;
    };
  const auto dbl = [&](const std::string & name) {
      const auto * o = find(name);
      return o ? o->as_double() : get_parameter(name).as_double();
    };
  const auto integer = [&](const std::string & name) {
      const auto * o = find(name);
      return o ? o->as_int() : get_parameter(name).as_int();
    };
  const auto boolean = [&](const std::string & name) {
      const auto * o = find(name);
      return o ? o->as_bool() : get_parameter(name).as_bool();
    };
  const auto dbl_array = [&](const std::string & name) {
      const auto * o = find(name);
      return o ? o->as_double_array() : get_parameter(name).as_double_array();
    };
  const auto int_array = [&](const std::string & name) {
      const auto * o = find(name);
      return o ? o->as_integer_array() : get_parameter(name).as_integer_array();
    };

  const double offset_x = dbl("field_origin_offset_x_m");
  const double offset_y = dbl("field_origin_offset_y_m");
  const double offset_z = dbl("field_origin_offset_z_m");

  // 高さはすべて box_top_z_m (箱の上端) からの相対で決める。
  // 当日は box_top_z_m だけ実測して入れれば、離す高さも各退避高さもまとめて追従する
  const double box_top_z = dbl("box_top_z_m");
  const double slot_z = box_top_z - dbl("slot_release_below_box_top_m") + offset_z;

  GameStateMachine::Config config;
  config.slots = generateSlots(
    dbl_array("box_center_x_" + field_color_),
    dbl("box_center_y_" + field_color_),
    static_cast<int>(integer("slot_cols_x")),
    static_cast<int>(integer("slot_rows_y")),
    dbl("cylinder_diameter_m") + dbl("slot_gap_x_m"),
    dbl("cylinder_diameter_m") + dbl("slot_gap_y_m"),
    slot_z);
  for (auto & slot : config.slots) {
    slot.x += offset_x;
    slot.y += offset_y;
  }

  const auto order_i64 = int_array("placement_order");
  config.placement_order.assign(order_i64.begin(), order_i64.end());

  config.slot_clamp_margin_m = dbl("slot_clamp_margin_m");
  config.require_manual_confirm = boolean("require_manual_confirm");
  config.approach_clearance_z =
    box_top_z + dbl("approach_clearance_above_box_top_m") + offset_z;
  config.transport_clearance_z =
    box_top_z + dbl("transport_clearance_above_box_top_m") + offset_z;
  config.retract_clearance_z =
    box_top_z + dbl("retract_clearance_above_box_top_m") + offset_z;
  config.pick_z = dbl("pick_z_m") + offset_z;
  config.grasp_dwell_sec = dbl("grasp_dwell_sec");
  config.orient_dwell_sec = dbl("orient_dwell_sec");

  // 初期位置: 極座標 → 直交座標。field_origin_offset は掛けない
  // (ロボット自身に対する姿勢で、フィールドの設置誤差とは無関係)
  const double init_r = dbl("init_pose_r_" + field_color_);
  const double init_theta = dbl("init_pose_theta_" + field_color_);
  if (!(init_r > 0.0) || !std::isfinite(init_theta)) {
    throw std::invalid_argument(
            "init_pose_r_" + field_color_ + " must be positive (got " +
            std::to_string(init_r) + "); robot_geometry.generated.yaml が古いか未生成");
  }
  config.init_pose.x = PolarUtils::toX(init_r, init_theta, dbl("turntable_axis_x_m"));
  config.init_pose.y = PolarUtils::toY(init_r, init_theta, dbl("turntable_axis_y_m"));
  config.init_pose.z = dbl("init_pose_z_" + field_color_);
  config.init_on_startup = boolean("init_on_startup");
  config.init_delay_sec = dbl("init_delay_sec");
  if (config.init_delay_sec < 0.0) {
    throw std::invalid_argument("init_delay_sec must be >= 0");
  }

  if (config.slots.empty()) {
    throw std::invalid_argument(
            "no slots generated; box_center_x_" + field_color_ +
            " が空か、slot_cols_x / slot_rows_y が 0 以下です");
  }
  if (config.placement_order.empty()) {
    throw std::invalid_argument("placement_order is empty; check config.yaml");
  }
  for (const int id : config.placement_order) {
    if (id < 0 || static_cast<std::size_t>(id) >= config.slots.size()) {
      throw std::invalid_argument(
              "placement_order contains out-of-range slot id: " + std::to_string(id) +
              " (slots.size()=" + std::to_string(config.slots.size()) + ")");
    }
  }
  return config;
}

// 箱の中心と格子ピッチからスロット座標を作る。
// スロットIDの順番は「箱ごとに、X列を外側・Y行を内側」で 0 から通し番号
// (箱0の(x0,y0),(x0,y1),(x0,y2),(x1,y0)... → 箱1の…)。
// placement_order はこのIDを並べたもの
std::vector<CartesianState> GameStateManagerNode::generateSlots(
  const std::vector<double> & box_center_x, double box_center_y,
  int cols_x, int rows_y, double pitch_x, double pitch_y, double slot_z)
{
  std::vector<CartesianState> slots;
  if (cols_x <= 0 || rows_y <= 0) {return slots;}
  slots.reserve(box_center_x.size() * static_cast<std::size_t>(cols_x * rows_y));

  for (const double cx : box_center_x) {
    for (int ix = 0; ix < cols_x; ++ix) {
      // 格子は箱の中心に対して左右(上下)対称に置く
      const double dx = (static_cast<double>(ix) - (cols_x - 1) / 2.0) * pitch_x;
      for (int iy = 0; iy < rows_y; ++iy) {
        const double dy = (static_cast<double>(iy) - (rows_y - 1) / 2.0) * pitch_y;
        CartesianState slot;
        slot.x = cx + dx;
        slot.y = box_center_y + dy;
        slot.z = slot_z;
        slots.push_back(slot);
      }
    }
  }
  return slots;
}

// 生成したスロットが箱に収まっているかを起動時とパラメータ変更時に知らせる。
// 当日「缶が入らない」に現場で気づけるようにするための警告
void GameStateManagerNode::logSlotGeometry(
  const GameStateMachine::Config & config,
  const std::vector<rclcpp::Parameter> & overrides) const
{
  // buildConfig と同じく「これから設定される値」を優先して見る
  // (Humble には post-set コールバックが無く、適用前に呼ばれるため)
  const auto dbl = [&](const std::string & name) {
      for (const auto & p : overrides) {
        if (p.get_name() == name) {return p.as_double();}
      }
      return get_parameter(name).as_double();
    };
  const auto integer = [&](const std::string & name) {
      for (const auto & p : overrides) {
        if (p.get_name() == name) {return p.as_int();}
      }
      return get_parameter(name).as_int();
    };
  const double diameter = dbl("cylinder_diameter_m");
  const int cols = static_cast<int>(integer("slot_cols_x"));
  const int rows = static_cast<int>(integer("slot_rows_y"));
  const double gap_x = dbl("slot_gap_x_m");
  const double gap_y = dbl("slot_gap_y_m");
  const double inner_x = dbl("box_inner_size_x_m");
  const double inner_y = dbl("box_inner_size_y_m");

  // 格子全体が占める幅 = (缶の数-1)*中心間距離 + 缶1個分
  const double span_x = (cols - 1) * (diameter + gap_x) + diameter;
  const double span_y = (rows - 1) * (diameter + gap_y) + diameter;

  RCLCPP_INFO(
    get_logger(),
    "slot grid: %dx%d/box, diameter=%.0fmm, gap=(%.1f, %.1f)mm, "
    "span=(%.1f, %.1f)mm vs box inner=(%.1f, %.1f)mm, slot_z=%.3fm",
    cols, rows, diameter * 1000.0, gap_x * 1000.0, gap_y * 1000.0,
    span_x * 1000.0, span_y * 1000.0, inner_x * 1000.0, inner_y * 1000.0,
    config.slots.empty() ? 0.0 : config.slots.front().z);

  if (gap_x < 0.0 || gap_y < 0.0) {
    RCLCPP_WARN(
      get_logger(),
      "隣り合う缶が重なっています (gap_x=%.1fmm, gap_y=%.1fmm)。"
      "slot_gap_x_m / slot_gap_y_m を増やすか、並べ方を見直してください",
      gap_x * 1000.0, gap_y * 1000.0);
  }
  if (span_x > inner_x || span_y > inner_y) {
    RCLCPP_WARN(
      get_logger(),
      "スロット格子が箱の内寸をはみ出しています "
      "(span=(%.1f, %.1f)mm > inner=(%.1f, %.1f)mm)。"
      "slot_gap_x_m / slot_gap_y_m を減らすか、slot_cols_x / slot_rows_y を見直してください",
      span_x * 1000.0, span_y * 1000.0, inner_x * 1000.0, inner_y * 1000.0);
  }
}

// 実行中のパラメータ変更。組み直せたものだけを適用し、駄目なら理由を返して却下する。
// **却下してもロボットは動き続ける** (直前の設定のまま) ので、現場で値を打ち間違えても
// シーケンスが壊れない
rcl_interfaces::msg::SetParametersResult GameStateManagerNode::onSetParameters(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  try {
    GameStateMachine::Config config = buildConfig(parameters);
    machine_->setConfig(config);
    result.successful = true;
    RCLCPP_INFO(
      get_logger(), "parameters updated at runtime (%zu changed); %zu slots",
      parameters.size(), config.slots.size());
    logSlotGeometry(config, parameters);
  } catch (const std::exception & e) {
    result.successful = false;
    result.reason = e.what();
    RCLCPP_WARN(get_logger(), "parameter update rejected: %s", e.what());
  }
  return result;
}

void GameStateManagerNode::onPickRequest(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  const auto pitch_yaw = OrientationUtils::toPitchYaw(msg->pose.orientation);
  CartesianState pose;
  pose.x = msg->pose.position.x;
  pose.y = msg->pose.position.y;
  pose.z = msg->pose.position.z;
  pose.pitch = pitch_yaw.pitch;
  pose.yaw = pitch_yaw.yaw;
  machine_->onPickPoseReceived(pose);
  publishPendingOutputs();
}

// VRの指定箱にワークを離した通算個数。box_count = N は
// 「placement_order[N-1] のスロットへ置きに行く」ことを意味する
// (0->1 なら placement_order[0])。**受け取った値が常に正本**なので、飛び・減少・
// 0リセットも警告を出したうえでその値に追従する (詳細は GameStateMachine::onBoxCount)。
//
// このイベント自体は動き出しの契機ではなく、キューに積むだけ。実際に動くのは
// GRASPING の dwell 経過後にキューが空でないときで、そこから TRANSPORTING →
// PLACING → RETRACTING まで自動で進む (place_request 相当の指示も box_count が兼ねる)
void GameStateManagerNode::onBoxCount(const std_msgs::msg::Int32::SharedPtr msg)
{
  const int count = msg->data;
  const int capacity = static_cast<int>(machine_->placementCount());
  if (count < 0 || count > capacity) {
    RCLCPP_WARN(
      get_logger(),
      "box_count %d is out of range [0, %d]; clamped", count, capacity);
  } else if (count != prev_box_count_ + 1) {
    RCLCPP_WARN(
      get_logger(),
      "box_count changed %d -> %d (expected +1); following the received value as-is",
      prev_box_count_, count);
  }
  prev_box_count_ = count;

  machine_->onBoxCount(count);
  // キュー投入で GRASPING → TRANSPORTING に進める場合があるので、
  // 次のタイマー周期を待たずにここで一度評価する
  machine_->tick(now().seconds());
  publishPendingOutputs();
  publishState();
}

void GameStateManagerNode::onArmStatus(const sharmech_msgs::msg::MotionStatus::SharedPtr msg)
{
  if (msg->last_result == prev_last_result_) {return;}
  prev_last_result_ = msg->last_result;

  if (msg->last_result == sharmech_msgs::msg::MotionStatus::RESULT_SUCCEEDED) {
    machine_->onGoalReached(now().seconds());
  } else if (msg->last_result == sharmech_msgs::msg::MotionStatus::RESULT_REJECTED ||
    msg->last_result == sharmech_msgs::msg::MotionStatus::RESULT_ABORTED)
  {
    RCLCPP_WARN(
      get_logger(),
      "Automated goal was not accepted (last_result=%u, %s); returning to WAITING_FOR_PICK",
      msg->last_result, msg->message.c_str());
    machine_->onGoalRejectedOrAborted();
  }
  publishPendingOutputs();
}

// デバッグ用のステート強制遷移。
// 状態だけを書き換え、その状態の目標姿勢は配信しない (GameStateMachine::forceState)。
// 未知の状態名は無視して警告する
void GameStateManagerNode::onChangeStateRequest(
  const std_msgs::msg::String::SharedPtr msg)
{
  const auto requested = gameStateFromString(msg->data);
  if (!requested) {
    RCLCPP_WARN(
      get_logger(),
      "Unknown state name for /catchrobo/debug/change_state: '%s'", msg->data.c_str());
    return;
  }

  const auto previous = machine_->state();
  machine_->forceState(*requested, now().seconds());
  RCLCPP_WARN(
    get_logger(),
    "Game state forced by debug topic: %s -> %s (no goal is published)",
    toString(previous).c_str(), toString(*requested).c_str());
  publishState();
}

// VRが使えない場合の脱出ハッチ。どの状態からでもトグルできる。
// 副作用コマンドは出ないが、作業領域クランプだけは必ずデフォルトへ戻る
// (GameStateMachine::toggleManualControl 参照)
void GameStateManagerNode::onToggleManualControl(const std_msgs::msg::Empty::SharedPtr)
{
  const auto previous = machine_->state();
  machine_->toggleManualControl(now().seconds());
  RCLCPP_WARN(
    get_logger(),
    "Manual control toggled: %s -> %s",
    toString(previous).c_str(), toString(machine_->state()).c_str());
  publishPendingOutputs();
  publishState();
}

// 状態のリセット要求 (/catchrobo/game/reset)。どの状態からでも INIT へ入り、
// 初期位置 (init_pose) へのゴールを出す (動作許可がまだなら立ち上がりで出す)。
// 到達 (last_result = SUCCEEDED) したら WAITING_FOR_PICK へ戻る。
// **配置の進み具合 (box_count のキュー) は消さない** ——
// 正本は VR 側の通算カウントなので、こちらだけ巻き戻すと食い違う
// (GameStateMachine::requestInit() のコメント参照)
void GameStateManagerNode::onResetRequest(const std_msgs::msg::Empty::SharedPtr)
{
  const auto previous = machine_->state();
  machine_->requestInit(now().seconds());
  RCLCPP_WARN(
    get_logger(),
    "Game state reset requested: %s -> INIT (moving to init_pose)",
    toString(previous).c_str());
  publishPendingOutputs();
  publishState();
}

// 微調整の確定。ADJUSTING_PICK / ADJUSTING_PLACE 以外では何も起きない
void GameStateManagerNode::onConfirm(const std_msgs::msg::Empty::SharedPtr)
{
  const auto previous = machine_->state();
  machine_->onConfirm(now().seconds());
  if (machine_->state() != previous) {
    RCLCPP_INFO(
      get_logger(), "Confirmed by operator: %s -> %s",
      toString(previous).c_str(), toString(machine_->state()).c_str());
  } else {
    RCLCPP_DEBUG(get_logger(), "Confirm ignored (state=%s)", toString(previous).c_str());
  }
  publishPendingOutputs();
  publishState();
}

// 現在の目標姿勢と動作許可。姿勢は控えるだけ (微調整後の垂直移動の起点に使う)。
// 動作許可の立ち上がり (MCU の実姿勢へ同期済み) は INIT が初期位置へ動き出す契機、
// 立ち下がり (MCU 未初期化・フィードバック途絶) はどの状態からでも INIT へ入る契機
// (GameStateMachine::onMotionEnabled)
void GameStateManagerNode::onCommandCartesian(
  const sharmech_msgs::msg::CartesianCommand::SharedPtr msg)
{
  const auto pitch_yaw = OrientationUtils::toPitchYaw(msg->pose.orientation);
  CartesianState pose;
  pose.x = msg->pose.position.x;
  pose.y = msg->pose.position.y;
  pose.z = msg->pose.position.z;
  pose.pitch = pitch_yaw.pitch;
  pose.yaw = pitch_yaw.yaw;
  machine_->onCurrentPose(pose);

  const auto previous = machine_->state();
  const bool was_enabled = machine_->motionEnabled();
  machine_->onMotionEnabled(msg->enable, now().seconds());
  if (msg->enable != was_enabled) {
    // 動作許可の立ち上がり = motion_generator_node が (再) 起動して同期した合図。
    // パラメータが既定 (v_max_z=0) に戻っている可能性があるので入れ直す
    if (msg->enable) {sent_v_max_z_.reset();}
    RCLCPP_WARN(
      get_logger(), "Motion %s by motion_generator_node: %s -> %s%s",
      msg->enable ? "enabled" : "disabled",
      toString(previous).c_str(), toString(machine_->state()).c_str(),
      msg->enable && machine_->state() == GameState::kInit ?
      " (moving to init_pose after init_delay_sec)" : "");
    publishPendingOutputs();
    publishState();
  }
}

void GameStateManagerNode::onTimer()
{
  machine_->tick(now().seconds());
  publishPendingOutputs();
  publishState();
}

void GameStateManagerNode::publishPendingOutputs()
{
  // 順序が重要: 作業領域クランプを先に送ってから、その後にゴールを送る。
  // 逆だとPLACINGのゴールが縮小前の(広い)クランプで一瞬評価される可能性がある
  if (machine_->hasPendingWorkspaceClamp()) {
    const auto clamp = machine_->consumePendingWorkspaceClamp();
    sharmech_msgs::msg::WorkspaceClamp msg;
    msg.reset = clamp.reset;
    msg.x_min = clamp.x_min;
    msg.x_max = clamp.x_max;
    msg.y_min = clamp.y_min;
    msg.y_max = clamp.y_max;
    msg.z_min = clamp.z_min;
    msg.z_max = clamp.z_max;
    workspace_clamp_pub_->publish(msg);
  }
  if (machine_->hasPendingGripper()) {
    std_msgs::msg::Bool msg;
    msg.data = machine_->consumePendingGripper();
    gripper_pub_->publish(msg);
  }
  if (machine_->hasPendingOrientVertical()) {
    std_msgs::msg::Bool msg;
    msg.data = machine_->consumePendingOrientVertical();
    orient_vertical_pub_->publish(msg);
  }
  if (machine_->hasPendingGoal()) {
    target_pose_pub_->publish(toPoseStampedMsg(machine_->consumePendingGoal()));
  }
}

void GameStateManagerNode::publishState()
{
  std_msgs::msg::String msg;
  msg.data = toString(machine_->state());
  state_pub_->publish(msg);
  publishJogLimitIfChanged();
  syncInitSpeedLimit();
}

// INIT の間だけ motion_generator_node の v_max_z を init_v_max_z に絞る。
//
// 専用トピック (jog_limit のような) を増やさず、`ros2 param set /motion_generator_node
// v_max_z` と同じことをパラメータクライアントで行う (ユーザー判断 2026-09-12)。
// INIT のゴールは動作許可から init_delay_sec (既定 3s) 後に出るので、その前に
// 届いている。応答は待たない (motion_generator_node 側が受理時にログを出す)。
// 相手のサービスがまだ無い起動直後は送らず、次の tick (state_publish_rate) で再試行する
void GameStateManagerNode::syncInitSpeedLimit()
{
  const double desired = machine_->state() == GameState::kInit ?
    get_parameter("init_v_max_z").as_double() : 0.0;
  if (sent_v_max_z_ && *sent_v_max_z_ == desired) {return;}
  if (!motion_params_client_->service_is_ready()) {return;}
  motion_params_client_->set_parameters({rclcpp::Parameter("v_max_z", desired)});
  sent_v_max_z_ = desired;
  RCLCPP_INFO(
    get_logger(), "motion_generator_node v_max_z -> %.3f m/s (state=%s)",
    desired, toString(machine_->state()).c_str());
}

// 状態ごとにジョグの扱いを決めて motion_generator_node へ伝える。
//
// **状態を知っているのはこのノードだけ、という役割分担を守るための経路。**
// 以前は WebXR クライアントが ROS2 の状態名一覧を持ち、状態ごとに
// 「ジョグを出す / 出さない」「速度に倍率を掛ける」を判断していたが、
// ROS2 側が状態を増やすたびにクライアントの配列を手で直す必要があり、
// 実際に ADJUSTING_* が漏れて微調整が効かなくなる事故が起きた (2026-09-08)。
// ここで一元化したことで、VR も PS4 もシミュレータも同じ挙動になる。
//
//   自動シーケンス動作中        → block (操縦者は触らない)
//   微調整待ち (ADJUSTING_*)    → 上限を adjusting_jog_v_max まで絞る
//   待機・自由操作・完了        → reset (起動時の jog_v_max)
//
// **GRASPING / ORIENTING も block に含める。** ゴールを持たない待機状態
// (グリッパを閉じる/缶を縦にする時間を待つだけ) なので、goal_priority による
// 「ゴール実行中は Twist を無視」の保護が効かず、ジョグが素通りしてしまう
void GameStateManagerNode::publishJogLimitIfChanged()
{
  const auto state = machine_->state();
  JogPolicy policy;
  switch (state) {
    case GameState::kAdjustingPick:
    case GameState::kAdjustingPlace:
      policy.v_max = get_parameter("adjusting_jog_v_max").as_double();
      break;
    case GameState::kWaitingForPick:
    case GameState::kManualControl:
    case GameState::kComplete:
      policy.reset = true;
      break;
    default:
      // kApproaching / kApproachDescend / kGrasping / kTransportLift /
      // kTransporting / kOrienting / kPlacing / kRetracting / kInit
      policy.block = true;
      break;
  }
  if (policy == last_jog_policy_) {return;}   // 変化したときだけ送る

  sharmech_msgs::msg::JogLimit msg;
  msg.reset = policy.reset;
  msg.block = policy.block;
  msg.v_max = policy.v_max;
  jog_limit_pub_->publish(msg);
  last_jog_policy_ = policy;

  const std::string what = policy.reset ? std::string("reset") :
    policy.block ? std::string("blocked") :
    ("-> " + std::to_string(policy.v_max) + " m/s");
  const std::string state_name = toString(state);
  RCLCPP_INFO(
    get_logger(), "Jog %s (state=%s)", what.c_str(), state_name.c_str());
}

}  // namespace sharmech_core

RCLCPP_COMPONENTS_REGISTER_NODE(sharmech_core::GameStateManagerNode)
