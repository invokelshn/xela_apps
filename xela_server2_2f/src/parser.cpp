#include "xela_server2_2f/parser.hpp"

#include <algorithm>
#include <cctype>
#include <iomanip>
#include <limits>
#include <sstream>
#include <utility>

#include "nlohmann/json.hpp"
#include "xela_server_ros2/msg/forces.hpp"
#include "xela_server_ros2/msg/taxel.hpp"
#include "xela_taxel_msgs/msg/x_taxel_sensor_t.hpp"

namespace xela_server2_2f {
namespace {

bool IsNumericKey(const std::string &key) {
  if (key.empty()) {
    return false;
  }
  return std::all_of(key.begin(), key.end(), [](unsigned char ch) {
    return std::isdigit(ch) != 0;
  });
}

bool ParseIntegerArray(const nlohmann::json &arr, std::vector<uint16_t> &values, std::string &error) {
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

bool ParseCalibratedArray(const nlohmann::json &arr, std::vector<float> &values, std::string &error) {
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

bool ParseTempsArray(const nlohmann::json &arr, std::vector<float> &values, std::string &error) {
  values.clear();
  if (!arr.is_array()) {
    error = "temp is not an array";
    return false;
  }
  values.reserve(arr.size());
  try {
    for (const auto &item : arr) {
      double kelvin = item.get<double>();
      values.push_back(static_cast<float>(kelvin - 273.15));
    }
  } catch (const std::exception &e) {
    error = std::string("temp parse error: ") + e.what();
    return false;
  }
  return true;
}

bool ParseSensorPos(const nlohmann::json &value, uint8_t &sensor_pos, std::string &error) {
  int parsed = 0;
  try {
    if (value.is_string()) {
      parsed = std::stoi(value.get<std::string>());
    } else if (value.is_number_integer()) {
      parsed = value.get<int>();
    } else if (value.is_number_unsigned()) {
      parsed = static_cast<int>(value.get<unsigned int>());
    } else {
      error = "sensor field is not a number or string";
      return false;
    }
  } catch (const std::exception &e) {
    error = std::string("sensor parse error: ") + e.what();
    return false;
  }

  if (parsed < 0 || parsed > std::numeric_limits<uint8_t>::max()) {
    error = "sensor value out of range";
    return false;
  }
  sensor_pos = static_cast<uint8_t>(parsed);
  return true;
}

size_t ParseTaxelCount(const nlohmann::json &sensor, std::string &error) {
  if (!sensor.contains("taxels")) {
    return 0U;
  }
  try {
    int count = sensor["taxels"].get<int>();
    if (count <= 0) {
      error = "taxels must be positive";
      return 0U;
    }
    return static_cast<size_t>(count);
  } catch (const std::exception &e) {
    error = std::string("taxels parse error: ") + e.what();
    return 0U;
  }
}

std::string FormatSensorId(uint8_t sensor_pos) {
  std::ostringstream out;
  out << std::setw(2) << std::setfill('0') << static_cast<int>(sensor_pos);
  return out.str();
}

std::string BuildGeneratedFrameId(const std::string &model, size_t index) {
  std::ostringstream out;
  out << model << "_" << std::setw(2) << std::setfill('0') << (index + 1U) << "_joint";
  return out.str();
}

std::string BuildFrameId(const std::string &frame_prefix,
                         const std::string &sensor_id,
                         const std::string &base) {
  if (frame_prefix.empty()) {
    return sensor_id + "_" + base;
  }
  return frame_prefix + "_" + sensor_id + "_" + base;
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
    const FrameConfig &frame_config,
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

  xela_taxel_msgs::msg::XTaxelSensorTArray msg;
  msg.header.stamp = now;
  msg.header.frame_id = config.header_frame_id;
  msg.md_frame_ids.reserve(sensors.size());
  msg.x_modules.reserve(sensors.size());

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

  for (const auto &sensor_entry : sensors) {
    const auto &sensor = sensor_entry.second;
    if (!sensor.is_object()) {
      error = "sensor entry is not an object";
      return ParseStatus::kError;
    }
    for (const auto &required : {"time", "sensor", "model", "integer", "temp"}) {
      if (!sensor.contains(required)) {
        error = std::string("missing sensor field: ") + required;
        return ParseStatus::kError;
      }
    }

    xela_taxel_msgs::msg::XTaxelSensorT module;
    module.message = message_value;

    if (config.use_ros_time_for_sensor_time) {
      module.time = now.seconds();
    } else {
      try {
        module.time = sensor["time"].get<double>();
      } catch (const std::exception &e) {
        error = std::string("time parse error: ") + e.what();
        return ParseStatus::kError;
      }
    }

    try {
      module.model = sensor["model"].get<std::string>();
    } catch (const std::exception &e) {
      error = std::string("model parse error: ") + e.what();
      return ParseStatus::kError;
    }

    auto model_it = frame_config.models.find(module.model);
    if (model_it == frame_config.models.end()) {
      AppendWarning(warnings, "missing model config: " + module.model);
      continue;
    }

    const auto &model_config = model_it->second;
    std::string count_error;
    size_t expected_count = ParseTaxelCount(sensor, count_error);
    if (!count_error.empty()) {
      error = count_error;
      return ParseStatus::kError;
    }

    if (expected_count == 0U && model_config.taxel_count.has_value()) {
      expected_count = *model_config.taxel_count;
    }

    if (expected_count == 0U && !model_config.gen_frame_list) {
      expected_count = model_config.names.size();
    }

    if (expected_count == 0U) {
      AppendWarning(warnings, "missing taxel_count for model: " + module.model);
      continue;
    }

    uint8_t sensor_pos = 0;
    if (!ParseSensorPos(sensor["sensor"], sensor_pos, error)) {
      return ParseStatus::kError;
    }
    module.sensor_pos = sensor_pos;

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
    if (!ParseTempsArray(sensor["temp"], temp_values, error)) {
      return ParseStatus::kError;
    }

    size_t data_count = integer_values.size() / 3U;
    size_t calibrated_count = calibrated_values.size() / 3U;
    if (integer_values.size() % 3U != 0U) {
      AppendWarning(warnings, "integer length not multiple of 3 for model: " + module.model);
    }
    if (!calibrated_values.empty() && calibrated_values.size() % 3U != 0U) {
      AppendWarning(warnings, "calibrated length not multiple of 3 for model: " + module.model);
    }

    size_t name_count = model_config.gen_frame_list ? expected_count : model_config.names.size();
    if (!model_config.gen_frame_list && name_count == 0U) {
      AppendWarning(warnings, "empty frame_ids for model: " + module.model);
      continue;
    }

    size_t temp_count = temp_values.size();
    size_t available_count = std::min({expected_count, name_count, data_count, temp_count});
    if (!calibrated_values.empty()) {
      available_count = std::min(available_count, calibrated_count);
    }
    if (available_count == 0U) {
      AppendWarning(warnings, "no usable taxels for model: " + module.model);
      continue;
    }

    if (data_count != expected_count || name_count != expected_count || temp_count != expected_count ||
        (!calibrated_values.empty() && calibrated_count != expected_count)) {
      std::ostringstream warn;
      warn << "count mismatch for model " << module.model << " (expected " << expected_count
           << ", data " << data_count;
      if (!calibrated_values.empty()) {
        warn << ", calibrated " << calibrated_count;
      }
      warn << ", frame_ids " << name_count << ", temps " << temp_count
           << ") using " << available_count;
      AppendWarning(warnings, warn.str());
    }

    std::string sensor_id = FormatSensorId(sensor_pos);
    module.frame_ids.reserve(available_count);
    if (model_config.gen_frame_list) {
      const std::string &gen_model = model_config.gen_model.value_or(module.model);
      for (size_t i = 0; i < available_count; ++i) {
        module.frame_ids.push_back(BuildFrameId(frame_config.frame_prefix, sensor_id,
                                                BuildGeneratedFrameId(gen_model, i)));
      }
    } else {
      for (size_t i = 0; i < available_count; ++i) {
        module.frame_ids.push_back(BuildFrameId(frame_config.frame_prefix, sensor_id,
                                                model_config.names[i]));
      }
    }

    module.taxels.resize(available_count);
    for (size_t i = 0; i < available_count; ++i) {
      xela_server_ros2::msg::Taxel taxel;
      taxel.x = integer_values[i * 3U + 0U];
      taxel.y = integer_values[i * 3U + 1U];
      taxel.z = integer_values[i * 3U + 2U];
      module.taxels[i] = taxel;
    }

    if (!calibrated_values.empty()) {
      module.forces.resize(available_count);
      for (size_t i = 0; i < available_count; ++i) {
        xela_server_ros2::msg::Forces force;
        force.x = calibrated_values[i * 3U + 0U];
        force.y = calibrated_values[i * 3U + 1U];
        force.z = calibrated_values[i * 3U + 2U];
        module.forces[i] = force;
      }
    }

    module.temps.resize(available_count);
    for (size_t i = 0; i < available_count; ++i) {
      module.temps[i] = temp_values[i];
    }

    msg.md_frame_ids.push_back(module.model);
    msg.x_modules.push_back(module);
  }

  if (msg.x_modules.empty()) {
    error = warnings.empty() ? "no valid sensors" : JoinWarnings(warnings);
    return ParseStatus::kIgnore;
  }

  out = std::move(msg);
  if (!warnings.empty()) {
    error = JoinWarnings(warnings);
    return ParseStatus::kWarn;
  }
  return ParseStatus::kOk;
}

}  // namespace xela_server2_2f
