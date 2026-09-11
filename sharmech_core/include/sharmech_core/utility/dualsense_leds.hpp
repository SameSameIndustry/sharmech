#ifndef SHARMECH_CORE__UTILITY__DUALSENSE_LEDS_HPP_
#define SHARMECH_CORE__UTILITY__DUALSENSE_LEDS_HPP_

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

namespace sharmech_core
{

// DualSense (PS5 コントローラ) の LED を Linux の sysfs 経由で操作する
//
// カーネルの hid-playstation ドライバがコントローラ 1 台につき次の LED クラスデバイスを作る:
//   /sys/class/leds/inputN:rgb:indicator     ライトバー (multicolor LED。multi_intensity="r g b")
//   /sys/class/leds/inputN:white:player-1..5 タッチパッド下の白いプレイヤーインジケータ 5 個
// `N` は接続ごとに変わるので名前ではなく `device` シンボリックリンクの
// ベンダ ID (Sony = 054C) で見分ける。USB (0003:054C:...) と Bluetooth (0005:054C:...) の
// どちらでも同じ形になる。
//
// `joy_node` (SDL2) は `/joy/set_feedback` で振動しか扱えず LED は触れないため、
// joy_teleop_node がここを通して直接書く。SDL2 は hidraw を開けない (root 権限) ので
// evdev 経由になり、カーネルドライバの LED 状態と競合しない。振動 (rumble) の
// 出力レポートも LED のフィールドを含まないので、振動で表示が消えることはない。
//
// **書き込みには権限が要る。** 既定では root しか書けないので
// `sharmech_bringup/udev/90-dualsense-leds.rules` を入れておくこと。
//
// **ライトバーの sysfs 値は初回書き込みまで実機と食い違う。** ドライバは接続時に
// 実機を青 (0,0,128) にするが multi_intensity は 0 0 0 のまま (Linux 6.8 の
// hid-playstation.c で確認)。そのため復元にはスナップショットではなく
// 「通常時の色」を呼び出し側が明示的に持つ (joy_teleop_node の lightbar_normal_rgb)。
// プレイヤー LED の brightness はドライバの状態そのものなのでスナップショットで戻せる。
//
// Static・ヘッダーオンリー・インスタンス生成禁止 (開発規約)。状態はノード側が持つ。
class DualSenseLeds
{
public:
  DualSenseLeds() = delete;

  static constexpr const char * kDefaultSysfsRoot = "/sys/class/leds";

  // 見つかった LED のパス。複数台繋がっていれば全部入る (区別しない)
  struct Devices
  {
    std::vector<std::filesystem::path> lightbars;   // inputN:rgb:indicator
    std::vector<std::filesystem::path> players;     // inputN:white:player-1..5

    bool found() const {return !lightbars.empty() || !players.empty();}
  };

  // 復元用のプレイヤー LED の状態 (players と同じ並び)
  struct PlayerSnapshot
  {
    std::vector<std::filesystem::path> paths;
    std::vector<int> brightness;
  };

  // sysfs から DualSense の LED を探す。root が無ければ空を返す (例外は投げない)
  static Devices discover(const std::filesystem::path & sysfs_root = kDefaultSysfsRoot)
  {
    Devices devices;
    std::error_code ec;
    if (!std::filesystem::is_directory(sysfs_root, ec)) {return devices;}
    for (const auto & entry : std::filesystem::directory_iterator(sysfs_root, ec)) {
      const std::string name = entry.path().filename().string();
      const bool is_lightbar = endsWith(name, ":rgb:indicator");
      const bool is_player = name.find(":white:player-") != std::string::npos;
      if (!is_lightbar && !is_player) {continue;}
      if (!isSonyDevice(entry.path())) {continue;}
      (is_lightbar ? devices.lightbars : devices.players).push_back(entry.path());
    }
    // directory_iterator の順序は不定なので名前順に揃える (player-1..5 の並び)
    std::sort(devices.lightbars.begin(), devices.lightbars.end());
    std::sort(devices.players.begin(), devices.players.end());
    return devices;
  }

  // ライトバーの色を設定する (0〜255)。明るさ (brightness) は触らない
  static bool setLightbarColor(
    const Devices & devices, int r, int g, int b, std::string * error = nullptr)
  {
    std::ostringstream rgb;
    rgb << clamp255(r) << ' ' << clamp255(g) << ' ' << clamp255(b);
    bool ok = true;
    for (const auto & path : devices.lightbars) {
      ok = writeFile(path / "multi_intensity", rgb.str(), error) && ok;
    }
    return ok;
  }

  // ライトバーの点灯/消灯 (色は multi_intensity のまま)。on は max_brightness (255)
  static bool setLightbarOn(const Devices & devices, bool on, std::string * error = nullptr)
  {
    return setBrightness(devices.lightbars, on, error);
  }

  // プレイヤー LED 5 個まとめて点灯/消灯。on は max_brightness (1)。
  // 点滅は setLightbarOn と合わせて交互に呼ぶ
  static bool setPlayersOn(const Devices & devices, bool on, std::string * error = nullptr)
  {
    return setBrightness(devices.players, on, error);
  }

  // 見つけたパスがまだ存在するか (コントローラが切断されると sysfs ごと消える)
  static bool stillPresent(const Devices & devices)
  {
    std::error_code ec;
    for (const auto & path : devices.lightbars) {
      if (!std::filesystem::exists(path, ec)) {return false;}
    }
    for (const auto & path : devices.players) {
      if (!std::filesystem::exists(path, ec)) {return false;}
    }
    return true;
  }

  // プレイヤー LED の現在値を控える (MANUAL_CONTROL から戻るときの復元用)
  static PlayerSnapshot snapshotPlayers(const Devices & devices)
  {
    PlayerSnapshot snap;
    for (const auto & path : devices.players) {
      const auto value = readInt(path / "brightness");
      if (!value) {continue;}
      snap.paths.push_back(path);
      snap.brightness.push_back(*value);
    }
    return snap;
  }

  static bool restorePlayers(const PlayerSnapshot & snap, std::string * error = nullptr)
  {
    bool ok = true;
    for (size_t i = 0; i < snap.paths.size(); ++i) {
      ok = writeFile(
        snap.paths[i] / "brightness", std::to_string(snap.brightness[i]), error) && ok;
    }
    return ok;
  }

  // --- 下請け (テストから見えるよう public) ---

  static bool writeFile(
    const std::filesystem::path & path, const std::string & value, std::string * error)
  {
    std::ofstream ofs(path);
    if (!ofs) {
      if (error) {*error = "cannot open " + path.string() + " for writing (permission?)";}
      return false;
    }
    ofs << value;
    if (!ofs) {
      if (error) {*error = "write failed: " + path.string();}
      return false;
    }
    return true;
  }

  static std::optional<int> readInt(const std::filesystem::path & path)
  {
    std::ifstream ifs(path);
    int value = 0;
    if (!(ifs >> value)) {return std::nullopt;}
    return value;
  }

private:
  static bool endsWith(const std::string & s, const std::string & suffix)
  {
    return s.size() >= suffix.size() &&
           s.compare(s.size() - suffix.size(), suffix.size(), suffix) == 0;
  }

  // `device` リンクの先が "…/0005:054C:0CE6.0002" のような HID デバイス名になっている。
  // Sony のベンダ ID 054C を含むものだけを DualSense (Edge 含む) とみなす
  static bool isSonyDevice(const std::filesystem::path & led_dir)
  {
    std::error_code ec;
    const auto target = std::filesystem::read_symlink(led_dir / "device", ec);
    if (ec) {return false;}
    return target.string().find(":054C:") != std::string::npos;
  }

  static std::string readMaxBrightness(const std::filesystem::path & led_dir)
  {
    const auto value = readInt(led_dir / "max_brightness");
    return std::to_string(value.value_or(1));
  }

  static bool setBrightness(
    const std::vector<std::filesystem::path> & led_dirs, bool on, std::string * error)
  {
    bool ok = true;
    for (const auto & path : led_dirs) {
      ok = writeFile(path / "brightness", on ? readMaxBrightness(path) : "0", error) && ok;
    }
    return ok;
  }

  static int clamp255(int v) {return std::clamp(v, 0, 255);}
};

}  // namespace sharmech_core

#endif  // SHARMECH_CORE__UTILITY__DUALSENSE_LEDS_HPP_
