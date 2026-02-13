#pragma once

#include <string>
#include <vector>

#include "rclcpp/time.hpp"
#include "xela_server2_2f/frame_ids.hpp"
#include "xela_taxel_msgs/msg/x_taxel_sensor_t_array.hpp"

namespace xela_server2_2f {

struct ParseConfig {
  bool use_ros_time_for_sensor_time;
  std::string header_frame_id;
};

enum class ParseStatus {
  kOk,
  kWarn,
  kIgnore,
  kError
};

ParseStatus ParseJsonToMessage(
    const std::string &json_text,
    const FrameConfig &frame_config,
    const rclcpp::Time &now,
    const ParseConfig &config,
    xela_taxel_msgs::msg::XTaxelSensorTArray &out,
    std::string &error);

}  // namespace xela_server2_2f
