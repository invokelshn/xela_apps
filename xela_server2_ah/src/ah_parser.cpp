#include "xela_server2_ah/ah_parser.hpp"

#include <algorithm>
#include <cctype>
#include <limits>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "nlohmann/json.hpp"
#include "xela_server_ros2/msg/forces.hpp"
#include "xela_server_ros2/msg/taxel.hpp"
#include "xela_taxel_msgs/msg/x_taxel_sensor_t.hpp"

namespace xela_server2_ah {
namespace {

bool IsNumericKey(const std::string &key) {
  if (key.empty()) {
    return false;
  }
  return std::all_of(key.begin(), key.end(), [](unsigned char ch) {
    return std::isdigit(ch) != 0;
  });
}

bool ParseIntegerArray(const nlohmann::json &arr, std::vector<uint16_t> &values,
                       std::string &error) {
  values.clear();
  if (!arr.is_array()) {
    error = "integer is not an array";
    return false;
  }
  values.reserve(arr.size());
  try {
    for (const auto &item : arr) {
      int64_t value = 0;
      if (item.is_number_integer()) {
        value = item.get<int64_t>();
      } else if (item.is_number_unsigned()) {
        value = static_cast<int64_t>(item.get<uint64_t>());
      } else {
        error = "integer array contains non-numeric value";
        return false;
      }
      if (value < 0 || value > std::numeric_limits<uint16_t>::max()) {
        error = "integer value out of range";
        return false;
      }
      values.push_back(static_cast<uint16_t>(value));
    }
  } catch (const std::exception &e) {
    error = std::string("integer parse error: ") + e.what();
    return false;
  }
  return true;
}

bool ParseCalibratedArray(const nlohmann::json &arr, std::vector<float> &values,
                          std::string &error) {
  values.clear();
  if (!arr.is_array()) {
    error = "calibrated is not an array";
    return false;
  }
  values.reserve(arr.size());
  try {
    for (const auto &item : arr) {
      values.push_back(static_cast<float>(item.get<double>()));
    }
  } catch (const std::exception &e) {
    error = std::string("calibrated parse error: ") + e.what();
    return false;
  }
  return true;
}

bool ParseTempArray(const nlohmann::json &arr, std::vector<float> &values, std::string &error) {
  values.clear();
  if (!arr.is_array()) {
    error = "temp is not an array";
    return false;
  }
  values.reserve(arr.size());
  try {
    for (const auto &item : arr) {
      values.push_back(static_cast<float>(item.get<double>()));
    }
  } catch (const std::exception &e) {
    error = std::string("temp parse error: ") + e.what();
    return false;
  }
  return true;
}

void AppendWarning(std::vector<std::string> &warnings, const std::string &message) {
  warnings.push_back(message);
}

std::string JoinWarnings(const std::vector<std::string> &warnings) {
  std::ostringstream out;
  for (size_t i = 0; i < warnings.size(); ++i) {
    if (i > 0) {
      out << "; ";
    }
    out << warnings[i];
  }
  return out.str();
}

}  // namespace

ParseStatus ParseJsonToMessage(
    const std::string &json_text,
    const JointMap &joint_map,
    const rclcpp::Time &now,
    const ParseConfig &config,
    xela_taxel_msgs::msg::XTaxelSensorTArray &out,
    std::string &error) {
  nlohmann::json data;
  try {
    data = nlohmann::json::parse(json_text);
  } catch (const std::exception &e) {
    error = std::string("JSON parse error: ") + e.what();
    return ParseStatus::kError;
  }

  if (!data.contains("message")) {
    error = "missing message field";
    return ParseStatus::kError;
  }

  if (data["message"].is_string() && data["message"].get<std::string>() == "Welcome") {
    return ParseStatus::kIgnore;
  }

  uint32_t message_value = 0;
  try {
    if (data["message"].is_number_unsigned()) {
      message_value = data["message"].get<uint32_t>();
    } else if (data["message"].is_number_integer()) {
      int64_t signed_val = data["message"].get<int64_t>();
      if (signed_val < 0 || signed_val > std::numeric_limits<uint32_t>::max()) {
        error = "message value out of range";
        return ParseStatus::kError;
      }
      message_value = static_cast<uint32_t>(signed_val);
    } else {
      error = "message field is not numeric";
      return ParseStatus::kError;
    }
  } catch (const std::exception &e) {
    error = std::string("message parse error: ") + e.what();
    return ParseStatus::kError;
  }

  std::vector<std::pair<int, nlohmann::json>> sensors;
  for (auto it = data.begin(); it != data.end(); ++it) {
    if (!IsNumericKey(it.key())) {
      continue;
    }
    try {
      int key_num = std::stoi(it.key());
      sensors.emplace_back(key_num, it.value());
    } catch (const std::exception &) {
      continue;
    }
  }

  if (sensors.empty()) {
    error = "no sensor entries found";
    return ParseStatus::kError;
  }

  std::sort(sensors.begin(), sensors.end(), [](const auto &a, const auto &b) {
    return a.first < b.first;
  });

  std::vector<std::string> warnings;

  if (data.contains("sensors") && data["sensors"].is_number_integer()) {
    int expected = data["sensors"].get<int>();
    if (expected >= 0 && static_cast<size_t>(expected) != sensors.size()) {
      std::ostringstream warn;
      warn << "sensors count mismatch (expected " << expected << ", found "
           << sensors.size() << ")";
      AppendWarning(warnings, warn.str());
    }
  }

  if (sensors.size() > 1U) {
    AppendWarning(warnings, "multiple sensors present; using the first one only");
  }

  const auto &sensor = sensors.front().second;
  if (!sensor.is_object()) {
    error = "sensor entry is not an object";
    return ParseStatus::kError;
  }

  for (const auto &required : {"time", "model", "integer", "temp"}) {
    if (!sensor.contains(required)) {
      error = std::string("missing sensor field: ") + required;
      return ParseStatus::kError;
    }
  }

  double sensor_time = 0.0;
  if (config.use_ros_time_for_sensor_time) {
    sensor_time = now.seconds();
  } else {
    try {
      sensor_time = sensor["time"].get<double>();
    } catch (const std::exception &e) {
      error = std::string("time parse error: ") + e.what();
      return ParseStatus::kError;
    }
  }

  try {
    (void)sensor["model"].get<std::string>();
  } catch (const std::exception &e) {
    error = std::string("model parse error: ") + e.what();
    return ParseStatus::kError;
  }

  size_t expected_taxels = joint_map.flat.size();
  if (sensor.contains("taxels")) {
    try {
      int taxels = sensor["taxels"].get<int>();
      if (taxels > 0 && static_cast<size_t>(taxels) != expected_taxels) {
        std::ostringstream warn;
        warn << "taxels mismatch (expected " << expected_taxels << ", got " << taxels << ")";
        AppendWarning(warnings, warn.str());
      }
    } catch (const std::exception &e) {
      error = std::string("taxels parse error: ") + e.what();
      return ParseStatus::kError;
    }
  }

  std::vector<uint16_t> integer_values;
  if (!ParseIntegerArray(sensor["integer"], integer_values, error)) {
    return ParseStatus::kError;
  }

  std::vector<float> calibrated_values;
  if (sensor.contains("calibrated")) {
    if (!ParseCalibratedArray(sensor["calibrated"], calibrated_values, error)) {
      return ParseStatus::kError;
    }
  }

  std::vector<float> temp_values;
  if (!ParseTempArray(sensor["temp"], temp_values, error)) {
    return ParseStatus::kError;
  }

  if (integer_values.size() % 3U != 0U) {
    AppendWarning(warnings, "integer length not multiple of 3; extra values ignored");
  }
  if (!calibrated_values.empty() && calibrated_values.size() % 3U != 0U) {
    AppendWarning(warnings, "calibrated length not multiple of 3; extra values ignored");
  }

  size_t integer_count = integer_values.size() / 3U;
  size_t calibrated_count = calibrated_values.size() / 3U;
  size_t temp_count = temp_values.size();

  if (integer_count != expected_taxels) {
    std::ostringstream warn;
    warn << "integer count mismatch (expected " << expected_taxels << ", got "
         << integer_count << ")";
    AppendWarning(warnings, warn.str());
  }
  if (!calibrated_values.empty() && calibrated_count != expected_taxels) {
    std::ostringstream warn;
    warn << "calibrated count mismatch (expected " << expected_taxels << ", got "
         << calibrated_count << ")";
    AppendWarning(warnings, warn.str());
  }
  if (temp_count != expected_taxels) {
    std::ostringstream warn;
    warn << "temp count mismatch (expected " << expected_taxels << ", got "
         << temp_count << ")";
    AppendWarning(warnings, warn.str());
  }

  xela_taxel_msgs::msg::XTaxelSensorTArray msg;
  msg.header.stamp = now;
  msg.header.frame_id = config.header_frame_id;
  msg.md_frame_ids.reserve(joint_map.modules.size());
  msg.x_modules.reserve(joint_map.modules.size());

  for (const auto &module_map : joint_map.modules) {
    if (module_map.sensor_pos < 0 || module_map.sensor_pos > 255) {
      std::ostringstream warn;
      warn << "sensor_pos out of range: " << module_map.sensor_pos;
      AppendWarning(warnings, warn.str());
      continue;
    }

    xela_taxel_msgs::msg::XTaxelSensorT module;
    module.message = message_value;
    module.time = sensor_time;
    module.model = module_map.model;
    module.sensor_pos = static_cast<uint8_t>(module_map.sensor_pos);

    std::vector<std::string> frame_ids;
    frame_ids.reserve(module_map.frame_ids.size());
    std::vector<xela_server_ros2::msg::Taxel> taxels;
    taxels.reserve(module_map.frame_ids.size());
    std::vector<xela_server_ros2::msg::Forces> forces;
    if (!calibrated_values.empty()) {
      forces.reserve(module_map.frame_ids.size());
    }
    std::vector<float> temps;
    temps.reserve(module_map.frame_ids.size());

    size_t skipped = 0U;
    for (size_t i = 0; i < module_map.flat_indices.size(); ++i) {
      size_t flat_index = module_map.flat_indices[i];
      bool has_integer = flat_index < integer_count;
      bool has_temp = flat_index < temp_count;
      bool has_calibrated = calibrated_values.empty() || flat_index < calibrated_count;

      if (!has_integer || !has_temp || !has_calibrated) {
        ++skipped;
        continue;
      }

      frame_ids.push_back(module_map.frame_ids[i]);

      xela_server_ros2::msg::Taxel taxel;
      size_t base = flat_index * 3U;
      taxel.x = integer_values[base + 0U];
      taxel.y = integer_values[base + 1U];
      taxel.z = integer_values[base + 2U];
      taxels.push_back(taxel);

      if (!calibrated_values.empty()) {
        xela_server_ros2::msg::Forces force;
        force.x = calibrated_values[base + 0U];
        force.y = calibrated_values[base + 1U];
        force.z = calibrated_values[base + 2U];
        forces.push_back(force);
      }

      temps.push_back(static_cast<float>(temp_values[flat_index] - 273.15));
    }

    if (frame_ids.empty()) {
      AppendWarning(warnings, "no usable taxels for sensor_pos " + std::to_string(module_map.sensor_pos));
      continue;
    }

    if (skipped > 0U) {
      std::ostringstream warn;
      warn << "sensor_pos " << module_map.sensor_pos << " skipped " << skipped
           << " taxels due to missing data";
      AppendWarning(warnings, warn.str());
    }

    module.frame_ids = std::move(frame_ids);
    module.taxels = std::move(taxels);
    module.temps = std::move(temps);
    if (!calibrated_values.empty()) {
      module.forces = std::move(forces);
    }

    msg.md_frame_ids.push_back(module.model);
    msg.x_modules.push_back(module);
  }

  if (msg.x_modules.empty()) {
    error = warnings.empty() ? "no valid modules" : JoinWarnings(warnings);
    return ParseStatus::kIgnore;
  }

  out = std::move(msg);
  if (!warnings.empty()) {
    error = JoinWarnings(warnings);
    return ParseStatus::kWarn;
  }
  return ParseStatus::kOk;
}

}  // namespace xela_server2_ah
