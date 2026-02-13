#pragma once

#include <string>

#include "rclcpp/time.hpp"
#include "xela_server2_ah/ah_joint_map.hpp"
#include "xela_taxel_msgs/msg/x_taxel_sensor_t_array.hpp"

namespace xela_server2_ah {

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
    const JointMap &joint_map,
    const rclcpp::Time &now,
    const ParseConfig &config,
    xela_taxel_msgs::msg::XTaxelSensorTArray &out,
    std::string &error);

}  // namespace xela_server2_ah
