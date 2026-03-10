#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cctype>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <yaml-cpp/yaml.h>

#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <xela_taxel_msgs/msg/x_taxel_sensor_t_array.hpp>

namespace {

std::string to_lower_copy(std::string value)
{
  std::transform(
    value.begin(), value.end(), value.begin(),
    [](unsigned char c) {return static_cast<char>(std::tolower(c));});
  return value;
}

std::string trim_copy(const std::string & value)
{
  std::size_t begin = 0;
  while (begin < value.size() && std::isspace(static_cast<unsigned char>(value[begin]))) {
    ++begin;
  }
  std::size_t end = value.size();
  while (end > begin && std::isspace(static_cast<unsigned char>(value[end - 1]))) {
    --end;
  }
  return value.substr(begin, end - begin);
}

bool ends_with(const std::string & value, const std::string & suffix)
{
  if (value.size() < suffix.size()) {
    return false;
  }
  return value.compare(value.size() - suffix.size(), suffix.size(), suffix) == 0;
}

std::string to_link_frame(const std::string & frame_id)
{
  if (ends_with(frame_id, "_joint")) {
    return frame_id.substr(0, frame_id.size() - 6) + "_link";
  }
  return frame_id;
}

std::string strip_leading_slash(const std::string & frame_id)
{
  if (!frame_id.empty() && frame_id.front() == '/') {
    return frame_id.substr(1);
  }
  return frame_id;
}

std::string json_escape(const std::string & value)
{
  std::string out;
  out.reserve(value.size());
  for (const char c : value) {
    switch (c) {
      case '\\':
        out += "\\\\";
        break;
      case '"':
        out += "\\\"";
        break;
      case '\n':
        out += "\\n";
        break;
      case '\r':
        out += "\\r";
        break;
      case '\t':
        out += "\\t";
        break;
      default:
        out.push_back(c);
        break;
    }
  }
  return out;
}

bool is_json_object_text(const std::string & value)
{
  const auto text = trim_copy(value);
  if (text.size() < 2) {
    return false;
  }
  return text.front() == '{' && text.back() == '}';
}

bool parameter_to_bool(const rclcpp::Parameter & param, bool & out)
{
  switch (param.get_type()) {
    case rclcpp::ParameterType::PARAMETER_BOOL:
      out = param.as_bool();
      return true;
    case rclcpp::ParameterType::PARAMETER_INTEGER:
      out = param.as_int() != 0;
      return true;
    case rclcpp::ParameterType::PARAMETER_DOUBLE:
      out = std::abs(param.as_double()) > 1e-12;
      return true;
    case rclcpp::ParameterType::PARAMETER_STRING: {
      const auto text = to_lower_copy(trim_copy(param.as_string()));
      out = (text == "1" || text == "true" || text == "yes" || text == "on");
      return true;
    }
    default:
      return false;
  }
}

bool parameter_to_double(const rclcpp::Parameter & param, double & out)
{
  switch (param.get_type()) {
    case rclcpp::ParameterType::PARAMETER_DOUBLE:
      out = param.as_double();
      return true;
    case rclcpp::ParameterType::PARAMETER_INTEGER:
      out = static_cast<double>(param.as_int());
      return true;
    case rclcpp::ParameterType::PARAMETER_STRING:
      try {
        out = std::stod(trim_copy(param.as_string()));
        return true;
      } catch (...) {
        return false;
      }
    default:
      return false;
  }
}

enum class SensorGroup {
  NONE,
  THUMB,
  INDEX,
  MIDDLE,
  RING,
  PALM
};

std::string extractModuleId(const std::string& joint_name) {
  // Format: x_taxel_0_{module_id}_...
  std::istringstream iss(joint_name);
  std::string token;
  int count = 0;
  while (std::getline(iss, token, '_')) {
    if (count == 3) {
      if (token.size() == 2 && isdigit(token[0]) && isdigit(token[1])) {
        return token;
      }
    }
    count++;
  }
  return "";
}

SensorGroup identifyGroup(const std::string& joint_name) {
  std::string mod = extractModuleId(joint_name);
  if (mod == "00" || mod == "01" || mod == "02" || mod == "03") return SensorGroup::INDEX;
  if (mod == "10" || mod == "11" || mod == "12" || mod == "13") return SensorGroup::MIDDLE;
  if (mod == "20" || mod == "21" || mod == "22" || mod == "23") return SensorGroup::RING;
  if (mod == "30" || mod == "31" || mod == "32" || mod == "33") return SensorGroup::THUMB;
  if (mod == "51" || mod == "52" || mod == "53") return SensorGroup::PALM;
  return SensorGroup::NONE;
}



}  // namespace

class XelaTaxelWebBridgeAhNode : public rclcpp::Node {
public:
  XelaTaxelWebBridgeAhNode()
  : Node("xela_taxel_web_bridge_cpp")
  {
    declare_parameters();
    load_parameters();
    load_mapping();
    load_pattern();
    recompute_runtime_state();
    configure_tf(emit_urdf_points_);

    pub_ = this->create_publisher<std_msgs::msg::String>(out_topic_, 10);
    sub_ = this->create_subscription<xela_taxel_msgs::msg::XTaxelSensorTArray>(
      in_topic_, 10,
      std::bind(&XelaTaxelWebBridgeAhNode::on_taxel_array, this, std::placeholders::_1));
    if (include_slip_meta_) {
      slip_diagnostics_sub_ = this->create_subscription<std_msgs::msg::String>(
        slip_diagnostics_topic_,
        10,
        std::bind(&XelaTaxelWebBridgeAhNode::on_slip_diagnostics, this, std::placeholders::_1));
    }
    if (include_grasp_event_meta_) {
      grasp_event_sub_ = this->create_subscription<std_msgs::msg::String>(
        grasp_event_topic_,
        10,
        std::bind(&XelaTaxelWebBridgeAhNode::on_grasp_event, this, std::placeholders::_1));
    }
    on_set_parameters_handle_ = this->add_on_set_parameters_callback(
      std::bind(&XelaTaxelWebBridgeAhNode::on_set_parameters, this, std::placeholders::_1));

    last_publish_steady_ = std::chrono::steady_clock::now();

    RCLCPP_INFO(
      this->get_logger(),
      "xela_taxel_web_bridge_cpp started: in=%s out=%s hand_side=%s slip_meta=%s diag_topic=%s grasp_meta=%s grasp_topic=%s",
      in_topic_.c_str(),
      out_topic_.c_str(),
      resolved_hand_side_.c_str(),
      include_slip_meta_ ? "true" : "false",
      slip_diagnostics_topic_.c_str(),
      include_grasp_event_meta_ ? "true" : "false",
      grasp_event_topic_.c_str());
  }

private:
  struct GridPoint
  {
    std::string module;
    std::size_t sensor_index{0};
    double x{0.0};
    double y{0.0};
    double fx{0.0};
    double fy{0.0};
    double fz{0.0};
    double norm{0.0};
  };

  struct UrdfPoint
  {
    std::string module;
    std::size_t sensor_index{0};
    std::string frame_id;
    double x{0.0};
    double y{0.0};
    double z{0.0};
    double fx{0.0};
    double fy{0.0};
    double fz{0.0};
    double norm{0.0};
  };

  struct TfCacheEntry
  {
    double x{0.0};
    double y{0.0};
    double z{0.0};
    std::chrono::steady_clock::time_point stamp;
  };

  struct Vec3
  {
    double x{0.0};
    double y{0.0};
    double z{0.0};
  };

  struct Point2
  {
    double x{0.0};
    double y{0.0};
  };

  struct BaselineState
  {
    bool ready{false};
    bool started{false};
    std::chrono::steady_clock::time_point start;
    std::size_t force_samples{0};
    std::size_t taxel_samples{0};
    std::vector<Vec3> force_sum;
    std::vector<Vec3> taxel_sum;
    std::vector<Vec3> force_base;
    std::vector<Vec3> taxel_base;
  };

  void declare_parameters()
  {
    this->declare_parameter<std::string>("in_topic", "/x_taxel_ah");
    this->declare_parameter<std::string>("out_topic", "/x_taxel_ah/web_state");
    this->declare_parameter<std::string>("viz_mode", "grid");
    this->declare_parameter<std::string>("model_name", "XR23AHLCPP");
    this->declare_parameter<std::string>("fixed_frame", "world");

    this->declare_parameter<std::string>("mapping_yaml", "");
    this->declare_parameter<std::string>("pattern_yaml", "");
    this->declare_parameter<std::string>("hand_side", "auto");
    this->declare_parameter<double>("cell_size", 0.01);
    this->declare_parameter<double>("origin_x", 0.0);
    this->declare_parameter<double>("origin_y", 0.0);
    this->declare_parameter<std::vector<int64_t>>("grid_separator_cols_left", std::vector<int64_t>{});
    this->declare_parameter<std::vector<int64_t>>("grid_separator_cols_right", std::vector<int64_t>{});

    this->declare_parameter<double>("force_scale", 1.0);
    this->declare_parameter<bool>("use_fz_only", false);
    this->declare_parameter<bool>("use_axis_normalization", true);
    this->declare_parameter<double>("xy_force_range", 1350.0);
    this->declare_parameter<double>("z_force_range", 16000.0);
    this->declare_parameter<bool>("use_taxels_when_no_forces", true);
    this->declare_parameter<double>("xy_taxel_range", 1350.0);
    this->declare_parameter<double>("z_taxel_range", 10085.0);
    this->declare_parameter<double>("baseline_duration_sec", 2.0);
    this->declare_parameter<double>("baseline_deadband_xy", 10.0);
    this->declare_parameter<double>("baseline_deadband_z", 60.0);
    this->declare_parameter<double>("baseline_deadband_taxel_xy", 10.0);
    this->declare_parameter<double>("baseline_deadband_taxel_z", 60.0);

    this->declare_parameter<double>("grid_force_x_sign", 1.0);
    this->declare_parameter<double>("grid_force_y_sign", 1.0);
    this->declare_parameter<double>("urdf_force_x_sign", 1.0);
    this->declare_parameter<double>("urdf_force_y_sign", 1.0);

    this->declare_parameter<bool>("show_thumb", true);
    this->declare_parameter<bool>("show_index", true);
    this->declare_parameter<bool>("show_middle", true);
    this->declare_parameter<bool>("show_ring", true);
    this->declare_parameter<bool>("show_palm", true);

    for (const char * mod : {"01", "02", "03", "10", "11", "12", "13", "20", "21", "22", "23", "30", "31", "32", "33", "51", "52", "53"}) {
        this->declare_parameter<bool>(std::string("show_module_") + mod, true);
    }
    this->declare_parameter<double>("max_publish_rate_hz", 20.0);
    this->declare_parameter<bool>("emit_urdf_points", false);
    this->declare_parameter<bool>("freeze_urdf_positions", true);
    this->declare_parameter<double>("tf_cache_ttl_sec", 0.5);
    this->declare_parameter<bool>("include_slip_meta", true);
    this->declare_parameter<std::string>(
      "slip_diagnostics_topic", "/xela_atag_slip_detector/diagnostics");
    this->declare_parameter<double>("slip_meta_timeout_sec", 2.0);
    this->declare_parameter<bool>("include_grasp_event_meta", true);
    this->declare_parameter<std::string>("grasp_event_topic", "/atag/grasp_event");
    this->declare_parameter<double>("grasp_event_timeout_sec", 3.0);
  }

  std::string get_string_param(const std::string & name, const std::string & fallback)
  {
    const auto param = this->get_parameter(name);
    if (param.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
      return param.as_string();
    }
    return fallback;
  }

  double get_double_param(const std::string & name, double fallback)
  {
    const auto param = this->get_parameter(name);
    double value = fallback;
    if (parameter_to_double(param, value)) {
      return value;
    }
    return fallback;
  }

  bool get_bool_param(const std::string & name, bool fallback)
  {
    const auto param = this->get_parameter(name);
    bool value = fallback;
    if (parameter_to_bool(param, value)) {
      return value;
    }
    return fallback;
  }

  std::vector<int64_t> get_int_array_param(const std::string & name)
  {
    const auto param = this->get_parameter(name);
    if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY) {
      return param.as_integer_array();
    }
    return {};
  }

  std::string resolve_hand_side() const
  {
    const auto side = to_lower_copy(trim_copy(hand_side_param_));
    if (side == "left" || side == "l") {
      return "left";
    }
    if (side == "right" || side == "r") {
      return "right";
    }

    const auto model = to_lower_copy(trim_copy(model_name_));
    if (model == "xr23ahrcpp" || model == "xr23ahrcpp_right") {
      return "right";
    }
    return "left";
  }

  void load_parameters()
  {
    in_topic_ = get_string_param("in_topic", "/x_taxel_ah");
    out_topic_ = get_string_param("out_topic", "/x_taxel_ah/web_state");
    viz_mode_ = get_string_param("viz_mode", "grid");
    model_name_ = get_string_param("model_name", "XR23AHLCPP");
    fixed_frame_ = get_string_param("fixed_frame", "world");

    mapping_yaml_ = get_string_param("mapping_yaml", "");
    pattern_yaml_ = get_string_param("pattern_yaml", "");
    hand_side_param_ = get_string_param("hand_side", "auto");

    cell_size_ = std::max(1e-6, get_double_param("cell_size", 0.01));
    origin_x_ = get_double_param("origin_x", 0.0);
    origin_y_ = get_double_param("origin_y", 0.0);
    grid_separator_cols_left_ = get_int_array_param("grid_separator_cols_left");
    grid_separator_cols_right_ = get_int_array_param("grid_separator_cols_right");

    force_scale_ = get_double_param("force_scale", 1.0);
    use_fz_only_ = get_bool_param("use_fz_only", false);
    use_axis_normalization_ = get_bool_param("use_axis_normalization", true);
    xy_force_range_ = std::max(1e-9, get_double_param("xy_force_range", 1350.0));
    z_force_range_ = std::max(1e-9, get_double_param("z_force_range", 16000.0));
    use_taxels_when_no_forces_ = get_bool_param("use_taxels_when_no_forces", true);
    xy_taxel_range_ = std::max(1e-9, get_double_param("xy_taxel_range", 1350.0));
    z_taxel_range_ = std::max(1e-9, get_double_param("z_taxel_range", 10085.0));
    baseline_duration_sec_ = std::max(0.0, get_double_param("baseline_duration_sec", 2.0));
    baseline_deadband_xy_ = std::max(0.0, get_double_param("baseline_deadband_xy", 10.0));
    baseline_deadband_z_ = std::max(0.0, get_double_param("baseline_deadband_z", 60.0));
    baseline_deadband_taxel_xy_ = std::max(0.0, get_double_param("baseline_deadband_taxel_xy", 10.0));
    baseline_deadband_taxel_z_ = std::max(0.0, get_double_param("baseline_deadband_taxel_z", 60.0));

    grid_force_x_sign_ = get_double_param("grid_force_x_sign", 1.0);
    grid_force_y_sign_ = get_double_param("grid_force_y_sign", 1.0);
    urdf_force_x_sign_ = get_double_param("urdf_force_x_sign", 1.0);
    urdf_force_y_sign_ = get_double_param("urdf_force_y_sign", 1.0);

    show_thumb_ = get_bool_param("show_thumb", true);
    show_index_ = get_bool_param("show_index", true);
    show_middle_ = get_bool_param("show_middle", true);
    show_ring_ = get_bool_param("show_ring", true);
    show_palm_ = get_bool_param("show_palm", true);

    for (auto& pair : show_modules_) {
        pair.second = get_bool_param("show_module_" + pair.first, true);
    }

    max_publish_rate_hz_ = get_double_param("max_publish_rate_hz", 20.0);
    emit_urdf_points_ = get_bool_param("emit_urdf_points", false);
    freeze_urdf_positions_ = get_bool_param("freeze_urdf_positions", true);
    tf_cache_ttl_sec_ = std::max(0.0, get_double_param("tf_cache_ttl_sec", 0.5));
    include_slip_meta_ = get_bool_param("include_slip_meta", true);
    slip_diagnostics_topic_ = get_string_param(
      "slip_diagnostics_topic", "/xela_atag_slip_detector/diagnostics");
    slip_meta_timeout_sec_ = std::max(0.0, get_double_param("slip_meta_timeout_sec", 2.0));
    include_grasp_event_meta_ = get_bool_param("include_grasp_event_meta", true);
    grasp_event_topic_ = get_string_param("grasp_event_topic", "/atag/grasp_event");
    grasp_event_timeout_sec_ = std::max(0.0, get_double_param("grasp_event_timeout_sec", 3.0));

    resolved_hand_side_ = resolve_hand_side();
  }

  void load_mapping()
  {
    if (mapping_yaml_.empty()) {
      throw std::runtime_error("mapping_yaml is empty");
    }

    std::string prefix_from = "x_taxel_0_";
    std::string prefix_to = "x_taxel_0_";
    if (resolved_hand_side_ == "right") {
      prefix_from = "x_taxel_0_";
      prefix_to = "x_taxel_1_";
    } else {
      prefix_from = "x_taxel_1_";
      prefix_to = "x_taxel_0_";
    }

    YAML::Node root = YAML::LoadFile(mapping_yaml_);
    YAML::Node map_node = root["taxel_joint_map"];
    if (!map_node) {
      map_node = root["server_model_joint_map"];
    }
    if (!map_node || !map_node.IsMap()) {
      throw std::runtime_error("taxel_joint_map/server_model_joint_map not found in mapping_yaml");
    }

    joint_to_index_.clear();
    std::size_t max_index = 0;
    for (const auto & it : map_node) {
      const int idx = it.first.as<int>();
      if (idx < 0) {
        continue;
      }
      auto joint = it.second.as<std::string>();
      SensorGroup group = identifyGroup(joint);
      std::string mod_id = extractModuleId(joint);
      
      if (joint.rfind(prefix_from, 0) == 0) {
        joint.replace(0, prefix_from.size(), prefix_to);
      }
      joint_to_index_[joint] = static_cast<std::size_t>(idx);
      max_index = std::max(max_index, static_cast<std::size_t>(idx));
      
      if (index_to_group_.size() <= static_cast<std::size_t>(idx)) {
        index_to_group_.resize(static_cast<std::size_t>(idx) + 1, SensorGroup::NONE);
      }
      index_to_group_[static_cast<std::size_t>(idx)] = group;

      if (index_to_module_.size() <= static_cast<std::size_t>(idx)) {
        index_to_module_.resize(static_cast<std::size_t>(idx) + 1, "");
      }
      index_to_module_[static_cast<std::size_t>(idx)] = mod_id;
    }
    data_size_ = max_index + 1;
    if (data_size_ == 0) {
      throw std::runtime_error("mapping_yaml has no valid indices");
    }
    RCLCPP_INFO(
      this->get_logger(),
      "Loaded mapping_yaml=%s with %zu indices for hand_side=%s",
      mapping_yaml_.c_str(),
      data_size_,
      resolved_hand_side_.c_str());
  }

  void load_pattern()
  {
    if (pattern_yaml_.empty()) {
      throw std::runtime_error("pattern_yaml is empty");
    }

    YAML::Node root = YAML::LoadFile(pattern_yaml_);
    YAML::Node pattern = root["pattern"];
    if (!pattern) {
      throw std::runtime_error("pattern section not found in pattern_yaml");
    }
    const int rows = pattern["rows"].as<int>();
    const int cols = pattern["cols"].as<int>();
    const YAML::Node index_map = pattern["index_map"];
    if (!index_map || !index_map.IsSequence()) {
      throw std::runtime_error("index_map not found or invalid in pattern_yaml");
    }
    if (rows <= 0 || cols <= 0) {
      throw std::runtime_error("pattern rows/cols invalid");
    }

    grid_rows_ = rows;
    grid_cols_ = cols;
    index_positions_.assign(data_size_, {});
    index_has_pos_.assign(data_size_, false);

    for (int r = 0; r < grid_rows_ && r < static_cast<int>(index_map.size()); ++r) {
      const auto row = index_map[r];
      if (!row || !row.IsSequence()) {
        continue;
      }
      for (int c = 0; c < grid_cols_ && c < static_cast<int>(row.size()); ++c) {
        const int idx = row[c].as<int>();
        if (idx < 0) {
          continue;
        }
        if (static_cast<std::size_t>(idx) >= data_size_) {
          RCLCPP_WARN(
            this->get_logger(),
            "Pattern index %d out of range (size=%zu); ignored",
            idx,
            data_size_);
          continue;
        }
        if (index_has_pos_[static_cast<std::size_t>(idx)]) {
          continue;
        }
        Point2 p;
        p.x = origin_x_ + (static_cast<double>(c) + 0.5) * cell_size_;
        p.y = origin_y_ - (static_cast<double>(r) + 0.5) * cell_size_;
        index_positions_[static_cast<std::size_t>(idx)] = p;
        index_has_pos_[static_cast<std::size_t>(idx)] = true;
      }
    }

    std::size_t active_count = 0;
    for (const auto has_pos : index_has_pos_) {
      if (has_pos) {
        ++active_count;
      }
    }
    RCLCPP_INFO(
      this->get_logger(),
      "Loaded pattern_yaml=%s with grid=%dx%d active_indices=%zu",
      pattern_yaml_.c_str(),
      grid_rows_,
      grid_cols_,
      active_count);
  }

  void recompute_runtime_state()
  {
    viz_mode_ = to_lower_copy(trim_copy(viz_mode_));
    if (viz_mode_ != "grid" && viz_mode_ != "urdf") {
      viz_mode_ = "grid";
    }
    if (viz_mode_ == "urdf") {
      emit_urdf_points_ = true;
    }

    if (max_publish_rate_hz_ > 0.0) {
      publish_period_sec_ = 1.0 / max_publish_rate_hz_;
    } else {
      publish_period_sec_ = 0.0;
    }
    fixed_frame_ = trim_copy(fixed_frame_);
    if (fixed_frame_.empty()) {
      fixed_frame_ = "world";
    }

    resolved_hand_side_ = resolve_hand_side();
    module_label_ = resolved_hand_side_ == "right" ? "right" : "left";
  }

  void configure_tf(const bool enable)
  {
    if (!enable || tf_buffer_) {
      return;
    }
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock(), tf2::durationFromSec(30.0));
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_, true);
  }

  double normalize_force(
    const double fx,
    const double fy,
    const double fz,
    const double xy_range,
    const double z_range) const
  {
    if (use_fz_only_) {
      return std::min(std::abs(fz) / std::max(z_range, 1e-9), 1.0);
    }

    if (use_axis_normalization_) {
      const double z_clamped = std::max(0.0, fz);
      const double nx = fx / std::max(xy_range, 1e-9);
      const double ny = fy / std::max(xy_range, 1e-9);
      const double nz = z_clamped / std::max(z_range, 1e-9);
      return std::min(std::sqrt(nx * nx + ny * ny + nz * nz), 1.0);
    }

    const double mag = std::sqrt(fx * fx + fy * fy + fz * fz);
    const double denom = std::sqrt(
      std::max(xy_range, 1e-9) * std::max(xy_range, 1e-9) * 2.0 +
      std::max(z_range, 1e-9) * std::max(z_range, 1e-9));
    return std::min(mag / std::max(denom, 1e-9), 1.0);
  }

  std::optional<std::array<double, 3>> lookup_frame_xyz(const std::string & frame_id)
  {
    if (!emit_urdf_points_ || !tf_buffer_) {
      return std::nullopt;
    }

    const auto now = std::chrono::steady_clock::now();
    const auto cache_it = tf_cache_.find(frame_id);
    if (cache_it != tf_cache_.end()) {
      if (freeze_urdf_positions_) {
        return std::array<double, 3>{cache_it->second.x, cache_it->second.y, cache_it->second.z};
      }
      if (tf_cache_ttl_sec_ <= 0.0) {
        return std::array<double, 3>{cache_it->second.x, cache_it->second.y, cache_it->second.z};
      }
      const auto age = std::chrono::duration<double>(now - cache_it->second.stamp).count();
      if (age < tf_cache_ttl_sec_) {
        return std::array<double, 3>{cache_it->second.x, cache_it->second.y, cache_it->second.z};
      }
    }

    try {
      const auto tf_msg = tf_buffer_->lookupTransform(fixed_frame_, frame_id, tf2::TimePointZero);
      TfCacheEntry entry;
      entry.x = static_cast<double>(tf_msg.transform.translation.x);
      entry.y = static_cast<double>(tf_msg.transform.translation.y);
      entry.z = static_cast<double>(tf_msg.transform.translation.z);
      entry.stamp = now;
      tf_cache_[frame_id] = entry;
      return std::array<double, 3>{entry.x, entry.y, entry.z};
    } catch (const tf2::TransformException &) {
      if (cache_it != tf_cache_.end()) {
        return std::array<double, 3>{cache_it->second.x, cache_it->second.y, cache_it->second.z};
      }
      return std::nullopt;
    }
  }

  void ensure_baseline_size(const std::size_t size)
  {
    if (baseline_.force_base.size() == size && baseline_.taxel_base.size() == size) {
      return;
    }
    baseline_.ready = false;
    baseline_.started = false;
    baseline_.force_samples = 0;
    baseline_.taxel_samples = 0;
    baseline_.force_sum.assign(size, {});
    baseline_.taxel_sum.assign(size, {});
    baseline_.force_base.assign(size, {});
    baseline_.taxel_base.assign(size, {});
  }

  void update_baseline(
    const std::vector<Vec3> & current,
    const std::vector<bool> & present,
    const std::vector<bool> & source_is_taxel,
    const std::chrono::steady_clock::time_point & now)
  {
    if (baseline_duration_sec_ <= 0.0) {
      baseline_.ready = true;
      return;
    }

    ensure_baseline_size(current.size());
    if (!baseline_.started) {
      baseline_.start = now;
      baseline_.started = true;
    }
    if (baseline_.ready) {
      return;
    }

    double elapsed = std::chrono::duration<double>(now - baseline_.start).count();
    if (elapsed < 0.0) {
      baseline_.start = now;
      elapsed = 0.0;
    }

    if (elapsed <= baseline_duration_sec_) {
      bool saw_force = false;
      bool saw_taxel = false;
      for (std::size_t idx = 0; idx < current.size(); ++idx) {
        if (!present[idx]) {
          continue;
        }
        if (source_is_taxel[idx]) {
          baseline_.taxel_sum[idx].x += current[idx].x;
          baseline_.taxel_sum[idx].y += current[idx].y;
          baseline_.taxel_sum[idx].z += current[idx].z;
          saw_taxel = true;
        } else {
          baseline_.force_sum[idx].x += current[idx].x;
          baseline_.force_sum[idx].y += current[idx].y;
          baseline_.force_sum[idx].z += current[idx].z;
          saw_force = true;
        }
      }
      if (saw_force) {
        baseline_.force_samples += 1;
      }
      if (saw_taxel) {
        baseline_.taxel_samples += 1;
      }
    }

    if (elapsed >= baseline_duration_sec_) {
      if (baseline_.force_samples > 0) {
        for (std::size_t idx = 0; idx < baseline_.force_base.size(); ++idx) {
          baseline_.force_base[idx].x =
            baseline_.force_sum[idx].x / static_cast<double>(baseline_.force_samples);
          baseline_.force_base[idx].y =
            baseline_.force_sum[idx].y / static_cast<double>(baseline_.force_samples);
          baseline_.force_base[idx].z =
            baseline_.force_sum[idx].z / static_cast<double>(baseline_.force_samples);
        }
      }
      if (baseline_.taxel_samples > 0) {
        for (std::size_t idx = 0; idx < baseline_.taxel_base.size(); ++idx) {
          baseline_.taxel_base[idx].x =
            baseline_.taxel_sum[idx].x / static_cast<double>(baseline_.taxel_samples);
          baseline_.taxel_base[idx].y =
            baseline_.taxel_sum[idx].y / static_cast<double>(baseline_.taxel_samples);
          baseline_.taxel_base[idx].z =
            baseline_.taxel_sum[idx].z / static_cast<double>(baseline_.taxel_samples);
        }
      }
      baseline_.ready = true;
    }
  }

  std::optional<std::size_t> lookup_index_for_frame(const std::string & frame_id) const
  {
    auto it = joint_to_index_.find(frame_id);
    if (it != joint_to_index_.end()) {
      return it->second;
    }

    const auto normalized = strip_leading_slash(frame_id);
    it = joint_to_index_.find(normalized);
    if (it != joint_to_index_.end()) {
      return it->second;
    }
    return std::nullopt;
  }

  bool isIndexVisible(size_t idx) const {
    if (idx >= index_to_group_.size() || idx >= index_to_module_.size()) return true;

    std::string mod = index_to_module_[idx];
    if (!mod.empty()) {
      auto it = show_modules_.find(mod);
      if (it != show_modules_.end() && !it->second) {
          return false;
      }
    }

    SensorGroup g = index_to_group_[idx];
    switch (g) {
      case SensorGroup::THUMB:  return show_thumb_;
      case SensorGroup::INDEX:  return show_index_;
      case SensorGroup::MIDDLE: return show_middle_;
      case SensorGroup::RING:   return show_ring_;
      case SensorGroup::PALM:   return show_palm_;
      default: return true;
    }
  }

  void on_slip_diagnostics(const std_msgs::msg::String::SharedPtr msg)
  {
    if (!include_slip_meta_ || !msg) {
      return;
    }

    const auto payload = trim_copy(msg->data);
    if (!is_json_object_text(payload)) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        3000,
        "Ignoring malformed slip diagnostics JSON payload");
      return;
    }

    std::lock_guard<std::mutex> lock(slip_meta_mutex_);
    latest_slip_diagnostics_json_ = payload;
    latest_slip_diagnostics_stamp_ = this->now();
  }

  void on_grasp_event(const std_msgs::msg::String::SharedPtr msg)
  {
    if (!include_grasp_event_meta_ || !msg) {
      return;
    }

    const auto payload = trim_copy(msg->data);
    if (!is_json_object_text(payload)) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        3000,
        "Ignoring malformed grasp event JSON payload");
      return;
    }

    std::lock_guard<std::mutex> lock(grasp_event_mutex_);
    latest_grasp_event_json_ = payload;
    latest_grasp_event_stamp_ = this->now();
  }

  bool get_recent_slip_json(std::string & payload_out, double & age_ms_out) const
  {
    if (!include_slip_meta_) {
      return false;
    }

    std::lock_guard<std::mutex> lock(slip_meta_mutex_);
    if (
      latest_slip_diagnostics_json_.empty() ||
      latest_slip_diagnostics_stamp_.nanoseconds() <= 0)
    {
      return false;
    }

    const double age_sec = std::fabs((this->now() - latest_slip_diagnostics_stamp_).seconds());
    if (age_sec > slip_meta_timeout_sec_) {
      return false;
    }

    payload_out = latest_slip_diagnostics_json_;
    age_ms_out = age_sec * 1000.0;
    return true;
  }

  bool get_recent_grasp_event_json(std::string & payload_out, double & age_ms_out) const
  {
    if (!include_grasp_event_meta_) {
      return false;
    }

    std::lock_guard<std::mutex> lock(grasp_event_mutex_);
    if (
      latest_grasp_event_json_.empty() ||
      latest_grasp_event_stamp_.nanoseconds() <= 0)
    {
      return false;
    }

    const double age_sec = std::fabs((this->now() - latest_grasp_event_stamp_).seconds());
    if (age_sec > grasp_event_timeout_sec_) {
      return false;
    }

    payload_out = latest_grasp_event_json_;
    age_ms_out = age_sec * 1000.0;
    return true;
  }

  void on_taxel_array(const xela_taxel_msgs::msg::XTaxelSensorTArray::SharedPtr msg)
  {
    if (data_size_ == 0) {
      return;
    }

    const auto now = std::chrono::steady_clock::now();
    if (publish_period_sec_ > 0.0) {
      const auto elapsed = std::chrono::duration<double>(now - last_publish_steady_).count();
      if (elapsed < publish_period_sec_) {
        return;
      }
    }
    last_publish_steady_ = now;

    const std::size_t modules_count = msg->x_modules.size();
    if (modules_count == 0) {
      return;
    }

    std::vector<Vec3> current(data_size_, Vec3{});
    std::vector<bool> present(data_size_, false);
    std::vector<bool> source_is_taxel(data_size_, false);
    std::vector<std::string> frame_for_index(data_size_);
    bool used_forces = false;
    bool used_taxels = false;

    for (const auto & module : msg->x_modules) {
      const bool use_taxel_data =
        use_taxels_when_no_forces_ && module.forces.empty() && !module.taxels.empty();
      const std::size_t count = std::min(
        module.frame_ids.size(),
        use_taxel_data ? module.taxels.size() : module.forces.size());
      used_taxels = used_taxels || use_taxel_data;
      used_forces = used_forces || !use_taxel_data;

      for (std::size_t j = 0; j < count; ++j) {
        const auto idx_opt = lookup_index_for_frame(module.frame_ids[j]);
        if (!idx_opt.has_value()) {
          continue;
        }
        const auto idx = idx_opt.value();
        if (idx >= data_size_) {
          continue;
        }

        Vec3 v;
        if (use_taxel_data) {
          v.x = static_cast<double>(module.taxels[j].x);
          v.y = static_cast<double>(module.taxels[j].y);
          v.z = static_cast<double>(module.taxels[j].z);
        } else {
          v.x = static_cast<double>(module.forces[j].x);
          v.y = static_cast<double>(module.forces[j].y);
          v.z = static_cast<double>(module.forces[j].z);
        }
        current[idx] = v;
        present[idx] = true;
        source_is_taxel[idx] = use_taxel_data;
        if (frame_for_index[idx].empty()) {
          frame_for_index[idx] = module.frame_ids[j];
        }
      }
    }

    update_baseline(current, present, source_is_taxel, now);

    std::vector<GridPoint> grid_points;
    std::vector<UrdfPoint> urdf_points;
    grid_points.reserve(data_size_);
    if (emit_urdf_points_) {
      urdf_points.reserve(data_size_);
    }
    std::size_t urdf_lookup_attempts = 0;
    std::size_t urdf_lookup_successes = 0;
    std::string first_missing_frame;

    for (std::size_t idx = 0; idx < data_size_; ++idx) {
      if (!present[idx] || !index_has_pos_[idx]) {
        continue;
      }
      
      if (!isIndexVisible(idx)) {
        continue;
      }

      double fx = current[idx].x;
      double fy = current[idx].y;
      double fz = current[idx].z;

      if (baseline_duration_sec_ > 0.0 && !baseline_.ready) {
        fx = 0.0;
        fy = 0.0;
        fz = 0.0;
      } else if (source_is_taxel[idx]) {
        fx -= baseline_.taxel_base[idx].x;
        fy -= baseline_.taxel_base[idx].y;
        fz -= baseline_.taxel_base[idx].z;
      } else {
        fx -= baseline_.force_base[idx].x;
        fy -= baseline_.force_base[idx].y;
        fz -= baseline_.force_base[idx].z;
      }

      const double deadband_xy =
        source_is_taxel[idx] ? baseline_deadband_taxel_xy_ : baseline_deadband_xy_;
      const double deadband_z =
        source_is_taxel[idx] ? baseline_deadband_taxel_z_ : baseline_deadband_z_;
      if (std::abs(fx) < deadband_xy) {
        fx = 0.0;
      }
      if (std::abs(fy) < deadband_xy) {
        fy = 0.0;
      }
      if (std::abs(fz) < deadband_z) {
        fz = 0.0;
      }

      fx *= force_scale_;
      fy *= force_scale_;
      fz *= force_scale_;

      const double xy_range = source_is_taxel[idx] ? xy_taxel_range_ : xy_force_range_;
      const double z_range = source_is_taxel[idx] ? z_taxel_range_ : z_force_range_;

      const double g_fx = fx * grid_force_x_sign_;
      const double g_fy = fy * grid_force_y_sign_;
      GridPoint gp;
      gp.module = module_label_;
      gp.sensor_index = idx;
      gp.x = index_positions_[idx].x;
      gp.y = index_positions_[idx].y;
      gp.fx = g_fx;
      gp.fy = g_fy;
      gp.fz = fz;
      gp.norm = normalize_force(g_fx, g_fy, fz, xy_range, z_range);
      grid_points.push_back(gp);

      if (emit_urdf_points_ && !frame_for_index[idx].empty()) {
        ++urdf_lookup_attempts;
        const auto source_frame = strip_leading_slash(frame_for_index[idx]);
        const auto link_frame = to_link_frame(source_frame);
        auto frame_id = link_frame;
        auto resolved_xyz = lookup_frame_xyz(frame_id);
        if (!resolved_xyz.has_value() && source_frame != link_frame) {
          frame_id = source_frame;
          resolved_xyz = lookup_frame_xyz(frame_id);
        }

        if (resolved_xyz.has_value()) {
          ++urdf_lookup_successes;
          const double u_fx = fx * urdf_force_x_sign_;
          const double u_fy = fy * urdf_force_y_sign_;
          UrdfPoint up;
          up.module = module_label_;
          up.sensor_index = idx;
          up.frame_id = frame_id;
          up.x = resolved_xyz.value()[0];
          up.y = resolved_xyz.value()[1];
          up.z = resolved_xyz.value()[2];
          up.fx = u_fx;
          up.fy = u_fy;
          up.fz = fz;
          up.norm = normalize_force(u_fx, u_fy, fz, xy_range, z_range);
          urdf_points.push_back(up);
        } else if (first_missing_frame.empty()) {
          first_missing_frame = frame_id;
        }
      }
    }

    if (emit_urdf_points_ && urdf_lookup_attempts > 0 && urdf_lookup_successes == 0) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 3000,
        "URDF TF lookup miss: fixed_frame=%s attempts=%zu sample_missing=%s",
        fixed_frame_.c_str(),
        urdf_lookup_attempts,
        first_missing_frame.empty() ? "<none>" : first_missing_frame.c_str());
    }

    const double stamp = static_cast<double>(msg->header.stamp.sec) +
      static_cast<double>(msg->header.stamp.nanosec) * 1e-9;
    const bool urdf_render_mode = viz_mode_ == "urdf" && emit_urdf_points_;
    double effective_xy_range = xy_force_range_;
    double effective_z_range = z_force_range_;
    if (used_taxels && !used_forces) {
      effective_xy_range = xy_taxel_range_;
      effective_z_range = z_taxel_range_;
    } else if (used_taxels && used_forces) {
      effective_xy_range = std::max(xy_force_range_, xy_taxel_range_);
      effective_z_range = std::max(z_force_range_, z_taxel_range_);
    }

    std::string slip_json;
    double slip_age_ms = 0.0;
    const bool has_slip = get_recent_slip_json(slip_json, slip_age_ms);
    std::string grasp_event_json;
    double grasp_event_age_ms = 0.0;
    const bool has_grasp_event = get_recent_grasp_event_json(grasp_event_json, grasp_event_age_ms);

    std::ostringstream json;
    auto append_int_array = [&json](const std::vector<int64_t> & values) {
      json << "[";
      for (std::size_t i = 0; i < values.size(); ++i) {
        if (i > 0) {
          json << ",";
        }
        json << values[i];
      }
      json << "]";
    };

    json << "{";
    json << "\"stamp\":" << stamp << ",";
    json << "\"fixed_frame\":\"" << json_escape(fixed_frame_) << "\",";

    json << "\"grid\":{";
    json << "\"rows\":" << grid_rows_ << ",";
    json << "\"cols\":" << grid_cols_ << ",";
    json << "\"cell_size\":" << cell_size_ << ",";
    json << "\"module_gap\":0.0,";
    json << "\"separator_cols_left\":";
    append_int_array(grid_separator_cols_left_);
    json << ",";
    json << "\"separator_cols_right\":";
    append_int_array(grid_separator_cols_right_);
    json << ",";
    json << "\"points\":[";
    for (std::size_t i = 0; i < grid_points.size(); ++i) {
      const auto & p = grid_points[i];
      if (i > 0) {
        json << ",";
      }
      json << "{";
      json << "\"module\":\"" << p.module << "\",";
      json << "\"sensor_index\":" << p.sensor_index << ",";
      json << "\"x\":" << p.x << ",";
      json << "\"y\":" << p.y << ",";
      json << "\"fx\":" << p.fx << ",";
      json << "\"fy\":" << p.fy << ",";
      json << "\"fz\":" << p.fz << ",";
      json << "\"norm\":" << p.norm;
      json << "}";
    }
    json << "]";
    json << "},";

    json << "\"urdf\":{";
    json << "\"points\":[";
    for (std::size_t i = 0; i < urdf_points.size(); ++i) {
      const auto & p = urdf_points[i];
      if (i > 0) {
        json << ",";
      }
      json << "{";
      json << "\"module\":\"" << p.module << "\",";
      json << "\"sensor_index\":" << p.sensor_index << ",";
      json << "\"frame_id\":\"" << json_escape(p.frame_id) << "\",";
      json << "\"x\":" << p.x << ",";
      json << "\"y\":" << p.y << ",";
      json << "\"z\":" << p.z << ",";
      json << "\"fx\":" << p.fx << ",";
      json << "\"fy\":" << p.fy << ",";
      json << "\"fz\":" << p.fz << ",";
      json << "\"norm\":" << p.norm;
      json << "}";
    }
    json << "]";
    json << "},";

    json << "\"meta\":{";
    json << "\"source_topic\":\"" << json_escape(in_topic_) << "\",";
    json << "\"module_count\":" << modules_count << ",";
    json << "\"requested_viz_mode\":\"" << viz_mode_ << "\",";
    json << "\"sidecar_render_mode\":\"" << (urdf_render_mode ? "urdf-fixed-3d" : "grid-2d") << "\",";
    json << "\"sidecar_grid_only\":" << (urdf_render_mode ? "false" : "true") << ",";
    json << "\"model_name\":\"" << json_escape(model_name_) << "\",";
    json << "\"hand_side\":\"" << resolved_hand_side_ << "\",";
    json << "\"emit_urdf_points\":" << (emit_urdf_points_ ? "true" : "false") << ",";
    json << "\"freeze_urdf_positions\":"
         << ((emit_urdf_points_ && freeze_urdf_positions_) ? "true" : "false") << ",";
    json << "\"urdf_points_count\":" << urdf_points.size() << ",";
    json << "\"urdf_lookup_attempts\":" << urdf_lookup_attempts << ",";
    json << "\"urdf_lookup_successes\":" << urdf_lookup_successes << ",";
    json << "\"xy_force_range\":" << effective_xy_range << ",";
    json << "\"z_force_range\":" << effective_z_range << ",";
    json << "\"use_fz_only\":" << (use_fz_only_ ? "true" : "false") << ",";
    json << "\"force_scale\":" << force_scale_;
    if (has_slip) {
      json << ",\"slip\":" << slip_json;
      json << ",\"slip_age_ms\":" << slip_age_ms;
    }
    if (has_grasp_event) {
      json << ",\"grasp_event\":" << grasp_event_json;
      json << ",\"grasp_event_age_ms\":" << grasp_event_age_ms;
    }
    json << "}";
    json << "}";

    std_msgs::msg::String out;
    out.data = json.str();
    pub_->publish(out);
  }

  rcl_interfaces::msg::SetParametersResult on_set_parameters(
    const std::vector<rclcpp::Parameter> & params)
  {
    auto result = rcl_interfaces::msg::SetParametersResult();
    result.successful = true;

    for (const auto & param : params) {
      const auto & name = param.get_name();
      if (name == "viz_mode") {
        if (param.get_type() != rclcpp::ParameterType::PARAMETER_STRING) {
          result.successful = false;
          result.reason = "viz_mode must be a string";
          return result;
        }
        auto mode = to_lower_copy(trim_copy(param.as_string()));
        if (mode != "grid" && mode != "urdf") {
          result.successful = false;
          result.reason = "viz_mode must be 'grid' or 'urdf'";
          return result;
        }
        viz_mode_ = mode;
      } else if (name == "model_name") {
        if (param.get_type() != rclcpp::ParameterType::PARAMETER_STRING) {
          result.successful = false;
          result.reason = "model_name must be a string";
          return result;
        }
        model_name_ = param.as_string();
      } else if (name == "emit_urdf_points") {
        bool value = emit_urdf_points_;
        if (!parameter_to_bool(param, value)) {
          result.successful = false;
          result.reason = "emit_urdf_points must be bool-compatible";
          return result;
        }
        emit_urdf_points_ = value;
      } else if (name == "freeze_urdf_positions") {
        bool value = freeze_urdf_positions_;
        if (!parameter_to_bool(param, value)) {
          result.successful = false;
          result.reason = "freeze_urdf_positions must be bool-compatible";
          return result;
        }
        freeze_urdf_positions_ = value;
      } else if (name == "max_publish_rate_hz") {
        double value = max_publish_rate_hz_;
        if (!parameter_to_double(param, value)) {
          result.successful = false;
          result.reason = "max_publish_rate_hz must be numeric";
          return result;
        }
        max_publish_rate_hz_ = value;
      } else if (name == "show_thumb") {
        bool value = show_thumb_;
        if (!parameter_to_bool(param, value)) {
          result.successful = false;
          result.reason = "show_thumb must be bool-compatible";
          return result;
        }
        show_thumb_ = value;
      } else if (name == "show_index") {
        bool value = show_index_;
        if (!parameter_to_bool(param, value)) {
          result.successful = false;
          result.reason = "show_index must be bool-compatible";
          return result;
        }
        show_index_ = value;
      } else if (name == "show_middle") {
        bool value = show_middle_;
        if (!parameter_to_bool(param, value)) {
          result.successful = false;
          result.reason = "show_middle must be bool-compatible";
          return result;
        }
        show_middle_ = value;
      } else if (name == "show_ring") {
        bool value = show_ring_;
        if (!parameter_to_bool(param, value)) {
          result.successful = false;
          result.reason = "show_ring must be bool-compatible";
          return result;
        }
        show_ring_ = value;
      } else if (name == "show_palm") {
        bool value = show_palm_;
        if (!parameter_to_bool(param, value)) {
          result.successful = false;
          result.reason = "show_palm must be bool-compatible";
          return result;
        }
        show_palm_ = value;
      } else if (name.find("show_module_") == 0) {
        std::string mod_id = name.substr(12);
        if (show_modules_.find(mod_id) != show_modules_.end()) {
          bool value = show_modules_[mod_id];
          if (!parameter_to_bool(param, value)) {
            result.successful = false;
            result.reason = "show_module_" + mod_id + " must be bool-compatible";
            return result;
          }
          show_modules_[mod_id] = value;
        }
      } else if (name == "fixed_frame") {
        if (param.get_type() != rclcpp::ParameterType::PARAMETER_STRING) {
          result.successful = false;
          result.reason = "fixed_frame must be a string";
          return result;
        }
        fixed_frame_ = param.as_string();
      }
    }

    recompute_runtime_state();
    configure_tf(emit_urdf_points_);

    return result;
  }

  std::string in_topic_;
  std::string out_topic_;
  std::string viz_mode_;
  std::string model_name_;
  std::string fixed_frame_;

  std::string mapping_yaml_;
  std::string pattern_yaml_;
  std::string hand_side_param_;
  std::string resolved_hand_side_;
  std::string module_label_{"left"};

  int grid_rows_{31};
  int grid_cols_{26};
  double cell_size_{0.01};
  double origin_x_{0.0};
  double origin_y_{0.0};
  std::vector<int64_t> grid_separator_cols_left_;
  std::vector<int64_t> grid_separator_cols_right_;

  std::unordered_map<std::string, std::size_t> joint_to_index_;
  std::vector<SensorGroup> index_to_group_;
  std::vector<std::string> index_to_module_;
  std::vector<Point2> index_positions_;
  std::vector<bool> index_has_pos_;
  std::size_t data_size_{0};

  std::unordered_map<std::string, bool> show_modules_ = {
    {"01", true}, {"02", true}, {"03", true},
    {"10", true}, {"11", true}, {"12", true}, {"13", true},
    {"20", true}, {"21", true}, {"22", true}, {"23", true},
    {"30", true}, {"31", true}, {"32", true}, {"33", true},
    {"51", true}, {"52", true}, {"53", true}
  };

  double force_scale_{1.0};
  bool use_fz_only_{false};
  bool use_axis_normalization_{true};
  double xy_force_range_{1350.0};
  double z_force_range_{16000.0};
  bool use_taxels_when_no_forces_{true};
  double xy_taxel_range_{1350.0};
  double z_taxel_range_{10085.0};
  double baseline_duration_sec_{2.0};
  double baseline_deadband_xy_{10.0};
  double baseline_deadband_z_{60.0};
  double baseline_deadband_taxel_xy_{10.0};
  double baseline_deadband_taxel_z_{60.0};

  double grid_force_x_sign_{1.0};
  double grid_force_y_sign_{1.0};
  double urdf_force_x_sign_{1.0};
  double urdf_force_y_sign_{1.0};

  bool show_thumb_{true};
  bool show_index_{true};
  bool show_middle_{true};
  bool show_ring_{true};
  bool show_palm_{true};

  double max_publish_rate_hz_{20.0};
  bool emit_urdf_points_{false};
  bool freeze_urdf_positions_{true};
  double tf_cache_ttl_sec_{0.5};
  bool include_slip_meta_{true};
  std::string slip_diagnostics_topic_{"/xela_atag_slip_detector/diagnostics"};
  double slip_meta_timeout_sec_{2.0};
  bool include_grasp_event_meta_{true};
  std::string grasp_event_topic_{"/atag/grasp_event"};
  double grasp_event_timeout_sec_{3.0};
  double publish_period_sec_{0.0};

  BaselineState baseline_;
  std::unordered_map<std::string, TfCacheEntry> tf_cache_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  std::chrono::steady_clock::time_point last_publish_steady_;
  mutable std::mutex slip_meta_mutex_;
  mutable std::mutex grasp_event_mutex_;
  std::string latest_slip_diagnostics_json_;
  rclcpp::Time latest_slip_diagnostics_stamp_{0, 0, RCL_ROS_TIME};
  std::string latest_grasp_event_json_;
  rclcpp::Time latest_grasp_event_stamp_{0, 0, RCL_ROS_TIME};

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::Subscription<xela_taxel_msgs::msg::XTaxelSensorTArray>::SharedPtr sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr slip_diagnostics_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr grasp_event_sub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_parameters_handle_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<XelaTaxelWebBridgeAhNode>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    fprintf(stderr, "Fatal error: %s\n", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
