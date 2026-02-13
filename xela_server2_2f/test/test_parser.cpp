#include <fstream>
#include <sstream>
#include <string>

#include "gtest/gtest.h"
#include "rclcpp/time.hpp"

#include "xela_server2_2f/frame_ids.hpp"
#include "xela_server2_2f/parser.hpp"

namespace xela_server2_2f {
namespace {

FrameConfig MakeFrameConfig(const std::string &prefix) {
  FrameConfig config;
  config.frame_prefix = prefix;
  return config;
}

std::string MakeBasicJson(int sensor_id, const std::string &model, bool include_taxels) {
  std::ostringstream out;
  out << "{\n";
  out << "  \"message\": 123,\n";
  out << "  \"" << sensor_id << "\": {\n";
  out << "    \"time\": 1.5,\n";
  out << "    \"sensor\": \"" << sensor_id << "\",\n";
  out << "    \"model\": \"" << model << "\",\n";
  if (include_taxels) {
    out << "    \"taxels\": 2,\n";
  }
  out << "    \"integer\": [1,2,3,4,5,6],\n";
  out << "    \"calibrated\": [0.1,0.2,0.3,0.4,0.5,0.6],\n";
  out << "    \"temp\": [300.15,273.15]\n";
  out << "  }\n";
  out << "}\n";
  return out.str();
}

}  // namespace

TEST(Parser, GeneratesFrameIdsAndTemps) {
  FrameConfig frame_config = MakeFrameConfig("x_taxel_0");
  ModelFrameConfig model_config;
  model_config.gen_frame_list = true;
  model_config.taxel_count = 2U;
  frame_config.models.emplace("uSPa46", model_config);

  ParseConfig parse_config;
  parse_config.use_ros_time_for_sensor_time = false;
  parse_config.header_frame_id = "";

  xela_taxel_msgs::msg::XTaxelSensorTArray out;
  std::string error;
  auto status = ParseJsonToMessage(MakeBasicJson(1, "uSPa46", true), frame_config,
                                   rclcpp::Time(0, 0, RCL_SYSTEM_TIME),
                                   parse_config, out, error);

  EXPECT_EQ(status, ParseStatus::kOk) << error;
  ASSERT_EQ(out.x_modules.size(), 1U);
  const auto &module = out.x_modules.front();
  ASSERT_EQ(module.frame_ids.size(), 2U);
  EXPECT_EQ(module.frame_ids[0], "x_taxel_0_01_uSPa46_01_joint");
  EXPECT_EQ(module.frame_ids[1], "x_taxel_0_01_uSPa46_02_joint");

  ASSERT_EQ(module.taxels.size(), 2U);
  EXPECT_EQ(module.taxels[0].x, 1U);
  EXPECT_EQ(module.taxels[0].y, 2U);
  EXPECT_EQ(module.taxels[0].z, 3U);

  ASSERT_EQ(module.temps.size(), 2U);
  EXPECT_NEAR(module.temps[0], 27.0F, 1e-3F);
  EXPECT_NEAR(module.temps[1], 0.0F, 1e-3F);
}

TEST(Parser, UsesFrameIdList) {
  FrameConfig frame_config = MakeFrameConfig("prefix");
  ModelFrameConfig model_config;
  model_config.gen_frame_list = false;
  model_config.names = {"foo_joint", "bar_joint"};
  frame_config.models.emplace("uSPaDS", model_config);

  ParseConfig parse_config;
  parse_config.use_ros_time_for_sensor_time = false;
  parse_config.header_frame_id = "";

  xela_taxel_msgs::msg::XTaxelSensorTArray out;
  std::string error;
  auto status = ParseJsonToMessage(MakeBasicJson(2, "uSPaDS", false), frame_config,
                                   rclcpp::Time(0, 0, RCL_SYSTEM_TIME),
                                   parse_config, out, error);

  EXPECT_EQ(status, ParseStatus::kOk) << error;
  ASSERT_EQ(out.x_modules.size(), 1U);
  const auto &module = out.x_modules.front();
  ASSERT_EQ(module.frame_ids.size(), 2U);
  EXPECT_EQ(module.frame_ids[0], "prefix_02_foo_joint");
  EXPECT_EQ(module.frame_ids[1], "prefix_02_bar_joint");
}

TEST(Parser, UsesGenModelForGeneratedFrames) {
  FrameConfig frame_config = MakeFrameConfig("x_taxel_0");
  ModelFrameConfig model_config;
  model_config.gen_frame_list = true;
  model_config.gen_model = "uSPa46";
  model_config.taxel_count = 2U;
  frame_config.models.emplace("uSPa46_v2", model_config);

  ParseConfig parse_config;
  parse_config.use_ros_time_for_sensor_time = false;
  parse_config.header_frame_id = "";

  xela_taxel_msgs::msg::XTaxelSensorTArray out;
  std::string error;
  auto status = ParseJsonToMessage(MakeBasicJson(1, "uSPa46_v2", true), frame_config,
                                   rclcpp::Time(0, 0, RCL_SYSTEM_TIME),
                                   parse_config, out, error);

  EXPECT_EQ(status, ParseStatus::kOk) << error;
  ASSERT_EQ(out.x_modules.size(), 1U);
  const auto &module = out.x_modules.front();
  ASSERT_EQ(module.frame_ids.size(), 2U);
  EXPECT_EQ(module.frame_ids[0], "x_taxel_0_01_uSPa46_01_joint");
  EXPECT_EQ(module.frame_ids[1], "x_taxel_0_01_uSPa46_02_joint");
}

TEST(FrameConfig, LoadsAliasesAndGlobal) {
  const std::string path = "/tmp/xela_server2_2f_config_test.yaml";
  std::ofstream file(path);
  ASSERT_TRUE(file.is_open());
  file << "global:\n";
  file << "  frame_prefix: x_taxel_0\n\n";
  file << "\"uSPa46,uSPa44\":\n";
  file << "  gen_frame_list: true\n";
  file << "  taxel_count: 2\n\n";
  file << "uSPaDS:\n";
  file << "  gen_frame_list: false\n";
  file << "  frame_ids:\n";
  file << "    - a\n";
  file << "    - b\n";
  file.close();

  std::string error;
  FrameConfig config = LoadFrameConfig(path, error);
  ASSERT_TRUE(error.empty()) << error;
  EXPECT_EQ(config.frame_prefix, "x_taxel_0");
  EXPECT_EQ(config.models.size(), 3U);
  ASSERT_TRUE(config.models.find("uSPa46") != config.models.end());
  ASSERT_TRUE(config.models.find("uSPa44") != config.models.end());
  ASSERT_TRUE(config.models.find("uSPaDS") != config.models.end());
  EXPECT_TRUE(config.models.at("uSPa46").gen_frame_list);
  EXPECT_EQ(config.models.at("uSPaDS").names.size(), 2U);
}

}  // namespace xela_server2_2f
