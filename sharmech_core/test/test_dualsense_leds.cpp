// DualSenseLeds (utility/dualsense_leds.hpp) の sysfs 操作を偽のディレクトリで検証する。
// 実機の /sys/class/leds は root 権限が要るので、同じ形のツリーを一時ディレクトリに作る。
#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <string>

#include "sharmech_core/utility/dualsense_leds.hpp"

namespace fs = std::filesystem;
using sharmech_core::DualSenseLeds;

namespace
{

std::string readAll(const fs::path & path)
{
  std::ifstream ifs(path);
  std::string s;
  std::getline(ifs, s);
  return s;
}

void writeAll(const fs::path & path, const std::string & value)
{
  std::ofstream ofs(path);
  ofs << value;
}

// hid-playstation が作るのと同じ構造を作る。device リンクの先は実在しなくてよい
// (discover はリンク文字列しか見ない)
class FakeSysfs : public ::testing::Test
{
protected:
  void SetUp() override
  {
    root_ = fs::temp_directory_path() / ("dualsense_leds_test_" + std::to_string(getpid()));
    fs::remove_all(root_);
    fs::create_directories(root_);
  }
  void TearDown() override {fs::remove_all(root_);}

  // Bluetooth 接続の DualSense 1 台分 (inputN は接続ごとに変わる)
  void addDualSense(int input_no, const std::string & hid = "0005:054C:0CE6.0002")
  {
    const std::string prefix = "input" + std::to_string(input_no);
    addLed(prefix + ":rgb:indicator", hid, "255", true);
    for (int i = 1; i <= 5; ++i) {
      // カーネルの player 1 パターンは中央 (player-3) だけ点灯
      addLed(prefix + ":white:player-" + std::to_string(i), hid, "1", false, i == 3 ? "1" : "0");
    }
  }

  void addLed(
    const std::string & name, const std::string & hid, const std::string & max_brightness,
    bool multicolor, const std::string & brightness = "255")
  {
    const fs::path dir = root_ / name;
    fs::create_directories(dir);
    writeAll(dir / "brightness", brightness);
    writeAll(dir / "max_brightness", max_brightness);
    if (multicolor) {
      writeAll(dir / "multi_index", "red green blue");
      writeAll(dir / "multi_intensity", "0 0 0");   // ドライバは初回書き込みまで 0 0 0
    }
    fs::create_symlink("../../../" + hid, dir / "device");
  }

  fs::path root_;
};

}  // namespace

TEST_F(FakeSysfs, DiscoverFindsLightbarAndFivePlayerLedsInOrder)
{
  addDualSense(21);
  // 無関係な LED (ThinkPad 等) は拾わない
  addLed("tpacpi::power", "platform", "1", false);
  // 名前は合うがベンダが違うものも拾わない
  addLed("input9:rgb:indicator", "0003:046D:C33F.0001", "255", true);

  const auto devices = DualSenseLeds::discover(root_);
  ASSERT_TRUE(devices.found());
  ASSERT_EQ(devices.lightbars.size(), 1u);
  EXPECT_EQ(devices.lightbars[0].filename(), "input21:rgb:indicator");
  ASSERT_EQ(devices.players.size(), 5u);
  for (int i = 0; i < 5; ++i) {
    EXPECT_EQ(devices.players[i].filename(), "input21:white:player-" + std::to_string(i + 1));
  }
}

TEST_F(FakeSysfs, DiscoverReturnsEmptyWhenNothingConnected)
{
  EXPECT_FALSE(DualSenseLeds::discover(root_).found());
  EXPECT_FALSE(DualSenseLeds::discover(root_ / "does_not_exist").found());
}

TEST_F(FakeSysfs, WhiteBlinkWritesColorAndTogglesAllBrightness)
{
  addDualSense(21);
  const auto devices = DualSenseLeds::discover(root_);
  const fs::path bar = root_ / "input21:rgb:indicator";

  EXPECT_TRUE(DualSenseLeds::setLightbarColor(devices, 255, 255, 255));
  EXPECT_EQ(readAll(bar / "multi_intensity"), "255 255 255");

  EXPECT_TRUE(DualSenseLeds::setLightbarOn(devices, true));
  EXPECT_TRUE(DualSenseLeds::setPlayersOn(devices, true));
  EXPECT_EQ(readAll(bar / "brightness"), "255");   // max_brightness
  for (int i = 1; i <= 5; ++i) {
    EXPECT_EQ(readAll(root_ / ("input21:white:player-" + std::to_string(i)) / "brightness"), "1");
  }

  EXPECT_TRUE(DualSenseLeds::setLightbarOn(devices, false));
  EXPECT_TRUE(DualSenseLeds::setPlayersOn(devices, false));
  EXPECT_EQ(readAll(bar / "brightness"), "0");
  for (int i = 1; i <= 5; ++i) {
    EXPECT_EQ(readAll(root_ / ("input21:white:player-" + std::to_string(i)) / "brightness"), "0");
  }
  // 色は消灯しても保持される (点灯し直すと白のまま)
  EXPECT_EQ(readAll(bar / "multi_intensity"), "255 255 255");
}

TEST_F(FakeSysfs, ColorIsClampedTo0To255)
{
  addDualSense(21);
  const auto devices = DualSenseLeds::discover(root_);
  EXPECT_TRUE(DualSenseLeds::setLightbarColor(devices, -5, 300, 128));
  EXPECT_EQ(readAll(root_ / "input21:rgb:indicator" / "multi_intensity"), "0 255 128");
}

TEST_F(FakeSysfs, SnapshotAndRestoreBringsPlayerPatternBack)
{
  addDualSense(21);
  const auto devices = DualSenseLeds::discover(root_);
  const auto snapshot = DualSenseLeds::snapshotPlayers(devices);
  ASSERT_EQ(snapshot.paths.size(), 5u);

  DualSenseLeds::setPlayersOn(devices, true);   // 点滅で全部触る
  DualSenseLeds::setPlayersOn(devices, false);
  EXPECT_TRUE(DualSenseLeds::restorePlayers(snapshot));
  for (int i = 1; i <= 5; ++i) {
    EXPECT_EQ(
      readAll(root_ / ("input21:white:player-" + std::to_string(i)) / "brightness"),
      i == 3 ? "1" : "0");
  }
}

TEST_F(FakeSysfs, StillPresentDetectsDisconnect)
{
  addDualSense(21);
  const auto devices = DualSenseLeds::discover(root_);
  EXPECT_TRUE(DualSenseLeds::stillPresent(devices));
  fs::remove_all(root_ / "input21:white:player-4");   // 切断で sysfs ごと消える
  EXPECT_FALSE(DualSenseLeds::stillPresent(devices));
  // 再接続は別の inputN で現れる
  addDualSense(25);
  const auto again = DualSenseLeds::discover(root_);
  ASSERT_EQ(again.lightbars.size(), 2u);   // 古い方の残骸 + 新しい方 (実機では古い方は消える)
  EXPECT_EQ(again.lightbars[1].filename(), "input25:rgb:indicator");
}

TEST_F(FakeSysfs, WriteFailureReportsPathAndReturnsFalse)
{
  addDualSense(21);
  auto devices = DualSenseLeds::discover(root_);
  devices.lightbars.push_back(root_ / "input99:rgb:indicator");   // 存在しない
  std::string error;
  EXPECT_FALSE(DualSenseLeds::setLightbarColor(devices, 1, 2, 3, &error));
  EXPECT_NE(error.find("input99:rgb:indicator/multi_intensity"), std::string::npos);
  // 存在する方には書けている (1 つ失敗しても残りは処理する)
  EXPECT_EQ(readAll(root_ / "input21:rgb:indicator" / "multi_intensity"), "1 2 3");
  // 何も無ければ (未接続) 失敗ではない
  EXPECT_TRUE(DualSenseLeds::setLightbarColor(DualSenseLeds::Devices{}, 1, 2, 3));
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
