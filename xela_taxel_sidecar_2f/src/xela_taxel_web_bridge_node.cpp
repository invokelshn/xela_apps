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

bool parameter_to_int(const rclcpp::Parameter & param, int & out)
{
  switch (param.get_type()) {
    case rclcpp::ParameterType::PARAMETER_INTEGER:
      out = static_cast<int>(param.as_int());
      return true;
    case rclcpp::ParameterType::PARAMETER_DOUBLE:
      out = static_cast<int>(std::llround(param.as_double()));
      return true;
    case rclcpp::ParameterType::PARAMETER_STRING:
      try {
        out = std::stoi(trim_copy(param.as_string()));
        return true;
      } catch (...) {
        return false;
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

const std::vector<int64_t> & default_usprds_left_index_map()
{
  static const std::vector<int64_t> map = {
    -1, 4, 0, 10, 14, 18, 22, 26,
    8, 5, 1, 11, 15, 19, 23, 27,
    9, 6, 2, 12, 16, 20, 24, 28,
    -1, 7, 3, 13, 17, 21, 25, 29,
  };
  return map;
}

const std::vector<int64_t> & default_usprds_right_index_map()
{
  static const std::vector<int64_t> map = {
    26, 22, 18, 14, 10, 0, 4, -1,
    27, 23, 19, 15, 11, 1, 5, 8,
    28, 24, 20, 16, 12, 2, 6, 9,
    29, 25, 21, 17, 13, 3, 7, -1,
  };
  return map;
}

}  // namespace

class XelaTaxelWebBridgeNode : public rclcpp::Node {
public:
  XelaTaxelWebBridgeNode()
  : Node("xela_taxel_web_bridge_cpp")
  {
    declare_parameters();
    load_parameters();
    recompute_runtime_state();
    configure_tf(emit_urdf_points_);

    pub_ = this->create_publisher<std_msgs::msg::String>(out_topic_, 10);
    sub_ = this->create_subscription<xela_taxel_msgs::msg::XTaxelSensorTArray>(
      in_topic_, 10,
      std::bind(&XelaTaxelWebBridgeNode::on_taxel_array, this, std::placeholders::_1));
    if (include_grasp_meta_) {
      grasp_telemetry_sub_ = this->create_subscription<std_msgs::msg::String>(
        grasp_telemetry_topic_, 10,
        std::bind(&XelaTaxelWebBridgeNode::on_grasp_telemetry, this, std::placeholders::_1));
      grasp_event_sub_ = this->create_subscription<std_msgs::msg::String>(
        grasp_event_topic_, 10,
        std::bind(&XelaTaxelWebBridgeNode::on_grasp_event, this, std::placeholders::_1));
    }
    on_set_parameters_handle_ = this->add_on_set_parameters_callback(
      std::bind(&XelaTaxelWebBridgeNode::on_set_parameters, this, std::placeholders::_1));

    last_publish_steady_ = std::chrono::steady_clock::now();

    RCLCPP_INFO(
      this->get_logger(),
      "xela_taxel_web_bridge_cpp started: in=%s out=%s fixed_frame=%s grasp_meta=%s telemetry_topic=%s event_topic=%s",
      in_topic_.c_str(),
      out_topic_.c_str(),
      fixed_frame_.c_str(),
      include_grasp_meta_ ? "true" : "false",
      grasp_telemetry_topic_.c_str(),
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

  struct BaselineState
  {
    bool ready{false};
    bool started{false};
    std::chrono::steady_clock::time_point start;
    std::size_t samples{0};
    std::vector<Vec3> force_sum;
    std::vector<Vec3> taxel_sum;
    std::vector<Vec3> force_base;
    std::vector<Vec3> taxel_base;
  };

  struct ModuleLayout
  {
    std::vector<std::pair<double, double>> index_centers;
    std::vector<bool> index_valid;
  };

  void declare_parameters()
  {
    this->declare_parameter<std::string>("in_topic", "/x_taxel_2f");
    this->declare_parameter<std::string>("out_topic", "/x_taxel_2f/web_state");
    this->declare_parameter<std::string>("viz_mode", "grid");
    this->declare_parameter<std::string>("model_name", "uSPr2F");
    this->declare_parameter<std::string>("fixed_frame", "world");

    this->declare_parameter<int>("left_module_index", 0);
    this->declare_parameter<int>("right_module_index", 1);

    this->declare_parameter<int>("grid_rows", 4);
    this->declare_parameter<int>("grid_cols", 6);
    this->declare_parameter<double>("cell_size", 0.015);
    this->declare_parameter<double>("module_gap", 0.04);
    this->declare_parameter<double>("origin_x", 0.0);
    this->declare_parameter<double>("origin_y", 0.0);
    this->declare_parameter<bool>("row_flip_right", true);
    this->declare_parameter<bool>("col_flip_right", true);
    this->declare_parameter<std::vector<int64_t>>("grid_index_map_left", std::vector<int64_t>{});
    this->declare_parameter<std::vector<int64_t>>("grid_index_map_right", std::vector<int64_t>{});
    this->declare_parameter<std::vector<int64_t>>("grid_separator_cols_left", std::vector<int64_t>{});
    this->declare_parameter<std::vector<int64_t>>("grid_separator_cols_right", std::vector<int64_t>{});

    this->declare_parameter<double>("force_scale", 1.0);
    this->declare_parameter<bool>("use_fz_only", false);
    this->declare_parameter<bool>("use_axis_normalization", true);
    this->declare_parameter<double>("xy_force_range", 0.8);
    this->declare_parameter<double>("z_force_range", 14.0);
    this->declare_parameter<bool>("use_taxels_when_no_forces", true);
    this->declare_parameter<double>("xy_taxel_range", 1350.0);
    this->declare_parameter<double>("z_taxel_range", 10085.0);
    this->declare_parameter<double>("baseline_duration_sec", 3.0);
    this->declare_parameter<double>("baseline_deadband_xy", 0.01);
    this->declare_parameter<double>("baseline_deadband_z", 0.05);
    this->declare_parameter<double>("baseline_deadband_taxel_xy", 10.0);
    this->declare_parameter<double>("baseline_deadband_taxel_z", 60.0);

    this->declare_parameter<double>("grid_left_force_x_sign", 1.0);
    this->declare_parameter<double>("grid_left_force_y_sign", -1.0);
    this->declare_parameter<double>("grid_right_force_x_sign", -1.0);
    this->declare_parameter<double>("grid_right_force_y_sign", -1.0);

    this->declare_parameter<double>("urdf_left_force_x_sign", 1.0);
    this->declare_parameter<double>("urdf_left_force_y_sign", 1.0);
    this->declare_parameter<double>("urdf_right_force_x_sign", 1.0);
    this->declare_parameter<double>("urdf_right_force_y_sign", 1.0);

    this->declare_parameter<double>("max_publish_rate_hz", 20.0);
    this->declare_parameter<bool>("emit_urdf_points", false);
    this->declare_parameter<bool>("freeze_urdf_positions", true);
    this->declare_parameter<double>("tf_cache_ttl_sec", 0.5);
    this->declare_parameter<bool>("include_grasp_meta", true);
    this->declare_parameter<std::string>("grasp_telemetry_topic", "/x_telemetry_2f/grasp_telemetry");
    this->declare_parameter<std::string>("grasp_event_topic", "/x_telemetry_2f/grasp_event");
    this->declare_parameter<double>("grasp_meta_timeout_sec", 2.0);
  }

  std::string get_string_param(const std::string & name, const std::string & fallback)
  {
    const auto param = this->get_parameter(name);
    if (param.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
      return param.as_string();
    }
    return fallback;
  }

  int get_int_param(const std::string & name, int fallback)
  {
    const auto param = this->get_parameter(name);
    int value = fallback;
    if (parameter_to_int(param, value)) {
      return value;
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

  void load_parameters()
  {
    in_topic_ = get_string_param("in_topic", "/x_taxel_2f");
    out_topic_ = get_string_param("out_topic", "/x_taxel_2f/web_state");
    viz_mode_ = get_string_param("viz_mode", "grid");
    model_name_ = get_string_param("model_name", "uSPr2F");
    fixed_frame_ = get_string_param("fixed_frame", "world");

    left_module_index_ = get_int_param("left_module_index", 0);
    right_module_index_ = get_int_param("right_module_index", 1);

    grid_rows_ = std::max(1, get_int_param("grid_rows", 4));
    grid_cols_ = std::max(1, get_int_param("grid_cols", 6));
    cell_size_ = get_double_param("cell_size", 0.015);
    module_gap_ = get_double_param("module_gap", 0.04);
    origin_x_ = get_double_param("origin_x", 0.0);
    origin_y_ = get_double_param("origin_y", 0.0);
    row_flip_right_ = get_bool_param("row_flip_right", true);
    col_flip_right_ = get_bool_param("col_flip_right", true);
    grid_index_map_left_ = get_int_array_param("grid_index_map_left");
    grid_index_map_right_ = get_int_array_param("grid_index_map_right");
    grid_separator_cols_left_ = get_int_array_param("grid_separator_cols_left");
    grid_separator_cols_right_ = get_int_array_param("grid_separator_cols_right");

    force_scale_ = get_double_param("force_scale", 1.0);
    use_fz_only_ = get_bool_param("use_fz_only", false);
    use_axis_normalization_ = get_bool_param("use_axis_normalization", true);
    xy_force_range_ = std::max(1e-9, get_double_param("xy_force_range", 0.8));
    z_force_range_ = std::max(1e-9, get_double_param("z_force_range", 14.0));
    use_taxels_when_no_forces_ = get_bool_param("use_taxels_when_no_forces", true);
    xy_taxel_range_ = std::max(1e-9, get_double_param("xy_taxel_range", 1350.0));
    z_taxel_range_ = std::max(1e-9, get_double_param("z_taxel_range", 10085.0));
    baseline_duration_sec_ = std::max(0.0, get_double_param("baseline_duration_sec", 3.0));
    baseline_deadband_xy_ = std::max(0.0, get_double_param("baseline_deadband_xy", 0.01));
    baseline_deadband_z_ = std::max(0.0, get_double_param("baseline_deadband_z", 0.05));
    baseline_deadband_taxel_xy_ = std::max(0.0, get_double_param("baseline_deadband_taxel_xy", 10.0));
    baseline_deadband_taxel_z_ = std::max(0.0, get_double_param("baseline_deadband_taxel_z", 60.0));

    grid_left_force_x_sign_ = get_double_param("grid_left_force_x_sign", 1.0);
    grid_left_force_y_sign_ = get_double_param("grid_left_force_y_sign", -1.0);
    grid_right_force_x_sign_ = get_double_param("grid_right_force_x_sign", -1.0);
    grid_right_force_y_sign_ = get_double_param("grid_right_force_y_sign", -1.0);

    urdf_left_force_x_sign_ = get_double_param("urdf_left_force_x_sign", 1.0);
    urdf_left_force_y_sign_ = get_double_param("urdf_left_force_y_sign", 1.0);
    urdf_right_force_x_sign_ = get_double_param("urdf_right_force_x_sign", 1.0);
    urdf_right_force_y_sign_ = get_double_param("urdf_right_force_y_sign", 1.0);

    max_publish_rate_hz_ = get_double_param("max_publish_rate_hz", 20.0);
    emit_urdf_points_ = get_bool_param("emit_urdf_points", false);
    freeze_urdf_positions_ = get_bool_param("freeze_urdf_positions", true);
    tf_cache_ttl_sec_ = std::max(0.0, get_double_param("tf_cache_ttl_sec", 0.5));
    include_grasp_meta_ = get_bool_param("include_grasp_meta", true);
    grasp_telemetry_topic_ = get_string_param("grasp_telemetry_topic", "/x_telemetry_2f/grasp_telemetry");
    grasp_event_topic_ = get_string_param("grasp_event_topic", "/x_telemetry_2f/grasp_event");
    grasp_meta_timeout_sec_ = std::max(0.0, get_double_param("grasp_meta_timeout_sec", 2.0));
  }

  ModuleLayout build_layout(
    const double base_x,
    const double base_y,
    const bool row_flip,
    const bool col_flip,
    const std::vector<int64_t> & index_map) const
  {
    std::vector<std::pair<double, double>> cell_centers;
    cell_centers.reserve(static_cast<std::size_t>(grid_rows_ * grid_cols_));
    for (int r = 0; r < grid_rows_; ++r) {
      for (int c = 0; c < grid_cols_; ++c) {
        const int draw_r = row_flip ? (grid_rows_ - 1 - r) : r;
        const int draw_c = col_flip ? (grid_cols_ - 1 - c) : c;
        const double x = base_x + (static_cast<double>(draw_c) + 0.5) * cell_size_;
        const double y = base_y - (static_cast<double>(draw_r) + 0.5) * cell_size_;
        cell_centers.emplace_back(x, y);
      }
    }

    ModuleLayout layout;
    if (index_map.empty()) {
      layout.index_centers = std::move(cell_centers);
      layout.index_valid.assign(layout.index_centers.size(), true);
      return layout;
    }

    if (index_map.size() != cell_centers.size()) {
      RCLCPP_WARN(
        this->get_logger(),
        "grid_index_map size mismatch: map=%zu expected=%zu. Falling back to sequential layout.",
        index_map.size(), cell_centers.size());
      layout.index_centers = std::move(cell_centers);
      layout.index_valid.assign(layout.index_centers.size(), true);
      return layout;
    }

    int64_t max_index = -1;
    for (const auto idx : index_map) {
      if (idx > max_index) {
        max_index = idx;
      }
    }
    if (max_index < 0) {
      return layout;
    }

    const std::size_t grid_size = static_cast<std::size_t>(max_index) + 1;
    layout.index_centers.assign(grid_size, {0.0, 0.0});
    layout.index_valid.assign(grid_size, false);

    for (std::size_t cell_idx = 0; cell_idx < index_map.size(); ++cell_idx) {
      const auto mapped_idx = index_map[cell_idx];
      if (mapped_idx < 0 || static_cast<std::size_t>(mapped_idx) >= grid_size) {
        continue;
      }
      layout.index_centers[static_cast<std::size_t>(mapped_idx)] = cell_centers[cell_idx];
      layout.index_valid[static_cast<std::size_t>(mapped_idx)] = true;
    }
    return layout;
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
    xy_force_range_ = std::max(1e-9, xy_force_range_);
    z_force_range_ = std::max(1e-9, z_force_range_);
    tf_cache_ttl_sec_ = std::max(0.0, tf_cache_ttl_sec_);

    publish_period_sec_ = (max_publish_rate_hz_ > 0.0) ? (1.0 / max_publish_rate_hz_) : 0.0;

    auto left_index_map = grid_index_map_left_;
    auto right_index_map = grid_index_map_right_;
    const auto normalized_model_name = to_lower_copy(trim_copy(model_name_));
    if (normalized_model_name == "usprds" && grid_rows_ == 4 && grid_cols_ == 8) {
      if (left_index_map.empty()) {
        left_index_map = default_usprds_left_index_map();
      }
      if (right_index_map.empty()) {
        right_index_map = default_usprds_right_index_map();
      }
      if (grid_separator_cols_left_.empty()) {
        grid_separator_cols_left_ = {3};
      }
      if (grid_separator_cols_right_.empty()) {
        grid_separator_cols_right_ = {5};
      }
    }

    layout_left_ = build_layout(origin_x_, origin_y_, false, false, left_index_map);
    layout_right_ = build_layout(
      origin_x_ + static_cast<double>(grid_cols_) * cell_size_ + module_gap_,
      origin_y_, row_flip_right_, col_flip_right_, right_index_map);
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
    const double clamped_xy_range = std::max(1e-9, xy_range);
    const double clamped_z_range = std::max(1e-9, z_range);
    if (use_fz_only_) {
      return std::min(std::abs(fz) / clamped_z_range, 1.0);
    }

    if (use_axis_normalization_) {
      const double z_clamped = std::max(0.0, fz);
      const double nx = fx / clamped_xy_range;
      const double ny = fy / clamped_xy_range;
      const double nz = z_clamped / clamped_z_range;
      return std::min(std::sqrt(nx * nx + ny * ny + nz * nz), 1.0);
    }

    const double mag = std::sqrt(fx * fx + fy * fy + fz * fz);
    const double denom = std::sqrt(
      clamped_xy_range * clamped_xy_range * 2.0 + clamped_z_range * clamped_z_range);
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

  void ensure_baseline_size(BaselineState & state, const std::size_t size)
  {
    if (state.force_base.size() == size && state.taxel_base.size() == size) {
      return;
    }
    state.ready = false;
    state.started = false;
    state.samples = 0;
    state.force_sum.assign(size, {});
    state.taxel_sum.assign(size, {});
    state.force_base.assign(size, {});
    state.taxel_base.assign(size, {});
  }

  void update_baseline(
    const xela_taxel_msgs::msg::XTaxelSensorT & module,
    const std::size_t expected_size,
    BaselineState & state,
    const std::chrono::steady_clock::time_point & now)
  {
    if (baseline_duration_sec_ <= 0.0) {
      state.ready = true;
      return;
    }

    ensure_baseline_size(state, expected_size);
    if (!state.started) {
      state.start = now;
      state.started = true;
    }
    if (state.ready) {
      return;
    }

    double elapsed = std::chrono::duration<double>(now - state.start).count();
    if (elapsed < 0.0) {
      state.start = now;
      elapsed = 0.0;
    }

    if (elapsed <= baseline_duration_sec_) {
      for (std::size_t idx = 0; idx < expected_size; ++idx) {
        if (idx < module.forces.size()) {
          state.force_sum[idx].x += static_cast<double>(module.forces[idx].x);
          state.force_sum[idx].y += static_cast<double>(module.forces[idx].y);
          state.force_sum[idx].z += static_cast<double>(module.forces[idx].z);
        }
        if (idx < module.taxels.size()) {
          state.taxel_sum[idx].x += static_cast<double>(module.taxels[idx].x);
          state.taxel_sum[idx].y += static_cast<double>(module.taxels[idx].y);
          state.taxel_sum[idx].z += static_cast<double>(module.taxels[idx].z);
        }
      }
      state.samples += 1;
    }

    if (elapsed >= baseline_duration_sec_) {
      if (state.samples > 0) {
        for (std::size_t idx = 0; idx < expected_size; ++idx) {
          state.force_base[idx].x = state.force_sum[idx].x / static_cast<double>(state.samples);
          state.force_base[idx].y = state.force_sum[idx].y / static_cast<double>(state.samples);
          state.force_base[idx].z = state.force_sum[idx].z / static_cast<double>(state.samples);
          state.taxel_base[idx].x = state.taxel_sum[idx].x / static_cast<double>(state.samples);
          state.taxel_base[idx].y = state.taxel_sum[idx].y / static_cast<double>(state.samples);
          state.taxel_base[idx].z = state.taxel_sum[idx].z / static_cast<double>(state.samples);
        }
      }
      state.ready = true;
    }
  }

  void append_module_payload(
    const xela_taxel_msgs::msg::XTaxelSensorT & module,
    const std::string & module_name,
    const ModuleLayout & layout,
    const BaselineState & baseline,
    const double grid_sign_x,
    const double grid_sign_y,
    const double urdf_sign_x,
    const double urdf_sign_y,
    bool & used_forces,
    bool & used_taxels,
    std::vector<GridPoint> & grid_points,
    std::vector<UrdfPoint> & urdf_points,
    std::size_t & urdf_lookup_attempts,
    std::size_t & urdf_lookup_successes,
    std::string & first_missing_frame)
  {
    const std::size_t force_count = module.forces.size();
    const std::size_t taxel_count = module.taxels.size();
    const std::size_t frame_count = module.frame_ids.size();
    const bool use_taxel_data = use_taxels_when_no_forces_ && force_count == 0 && taxel_count > 0;
    const std::size_t data_count = use_taxel_data ? taxel_count : force_count;
    const double xy_range = use_taxel_data ? xy_taxel_range_ : xy_force_range_;
    const double z_range = use_taxel_data ? z_taxel_range_ : z_force_range_;
    used_taxels = used_taxels || use_taxel_data;
    used_forces = used_forces || !use_taxel_data;

    for (std::size_t idx = 0; idx < layout.index_centers.size(); ++idx) {
      if (!layout.index_valid.empty() && !layout.index_valid[idx]) {
        continue;
      }
      if (idx >= data_count) {
        continue;
      }

      double fx = 0.0;
      double fy = 0.0;
      double fz = 0.0;
      if (use_taxel_data) {
        fx = static_cast<double>(module.taxels[idx].x);
        fy = static_cast<double>(module.taxels[idx].y);
        fz = static_cast<double>(module.taxels[idx].z);
      } else {
        fx = static_cast<double>(module.forces[idx].x);
        fy = static_cast<double>(module.forces[idx].y);
        fz = static_cast<double>(module.forces[idx].z);
      }

      if (baseline_duration_sec_ > 0.0 && !baseline.ready) {
        fx = 0.0;
        fy = 0.0;
        fz = 0.0;
      } else if (use_taxel_data && idx < baseline.taxel_base.size()) {
        fx -= baseline.taxel_base[idx].x;
        fy -= baseline.taxel_base[idx].y;
        fz -= baseline.taxel_base[idx].z;
      } else if (!use_taxel_data && idx < baseline.force_base.size()) {
        fx -= baseline.force_base[idx].x;
        fy -= baseline.force_base[idx].y;
        fz -= baseline.force_base[idx].z;
      }

      const double deadband_xy = use_taxel_data ? baseline_deadband_taxel_xy_ : baseline_deadband_xy_;
      const double deadband_z = use_taxel_data ? baseline_deadband_taxel_z_ : baseline_deadband_z_;
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

      const double g_fx = fx * grid_sign_x;
      const double g_fy = fy * grid_sign_y;
      GridPoint gp;
      gp.module = module_name;
      gp.sensor_index = idx;
      gp.x = layout.index_centers[idx].first;
      gp.y = layout.index_centers[idx].second;
      gp.fx = g_fx;
      gp.fy = g_fy;
      gp.fz = fz;
      gp.norm = normalize_force(g_fx, g_fy, fz, xy_range, z_range);
      grid_points.push_back(gp);

      if (emit_urdf_points_ && idx < frame_count) {
        ++urdf_lookup_attempts;
        const auto source_frame = strip_leading_slash(module.frame_ids[idx]);
        const auto link_frame = to_link_frame(source_frame);
        auto frame_id = link_frame;
        const auto xyz = lookup_frame_xyz(frame_id);
        auto resolved_xyz = xyz;
        if (!resolved_xyz.has_value() && link_frame != source_frame) {
          frame_id = source_frame;
          resolved_xyz = lookup_frame_xyz(frame_id);
        }

        if (resolved_xyz.has_value()) {
          ++urdf_lookup_successes;
          const double u_fx = fx * urdf_sign_x;
          const double u_fy = fy * urdf_sign_y;
          UrdfPoint up;
          up.module = module_name;
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
  }

  void on_grasp_telemetry(const std_msgs::msg::String::SharedPtr msg)
  {
    if (!include_grasp_meta_ || !msg) {
      return;
    }

    const auto payload = trim_copy(msg->data);
    if (!is_json_object_text(payload)) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 3000,
        "Ignoring malformed grasp telemetry JSON payload");
      return;
    }

    std::lock_guard<std::mutex> lock(grasp_meta_mutex_);
    latest_grasp_telemetry_json_ = payload;
    latest_grasp_telemetry_stamp_ = this->now();
  }

  void on_grasp_event(const std_msgs::msg::String::SharedPtr msg)
  {
    if (!include_grasp_meta_ || !msg) {
      return;
    }

    const auto payload = trim_copy(msg->data);
    if (!is_json_object_text(payload)) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 3000,
        "Ignoring malformed grasp event JSON payload");
      return;
    }

    std::lock_guard<std::mutex> lock(grasp_meta_mutex_);
    latest_grasp_event_json_ = payload;
    latest_grasp_event_stamp_ = this->now();
  }

  bool get_recent_grasp_json(const bool event_payload, std::string & payload_out, double & age_ms_out) const
  {
    if (!include_grasp_meta_) {
      return false;
    }

    std::lock_guard<std::mutex> lock(grasp_meta_mutex_);
    const auto & payload = event_payload ? latest_grasp_event_json_ : latest_grasp_telemetry_json_;
    const auto stamp = event_payload ? latest_grasp_event_stamp_ : latest_grasp_telemetry_stamp_;
    if (payload.empty() || stamp.nanoseconds() <= 0) {
      return false;
    }

    const double age_sec = std::fabs((this->now() - stamp).seconds());
    if (age_sec > grasp_meta_timeout_sec_) {
      return false;
    }

    payload_out = payload;
    age_ms_out = age_sec * 1000.0;
    return true;
  }

  void on_taxel_array(const xela_taxel_msgs::msg::XTaxelSensorTArray::SharedPtr msg)
  {
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

    std::vector<GridPoint> grid_points;
    std::vector<UrdfPoint> urdf_points;
    grid_points.reserve(layout_left_.index_centers.size() + layout_right_.index_centers.size());
    if (emit_urdf_points_) {
      urdf_points.reserve(layout_left_.index_centers.size() + layout_right_.index_centers.size());
    }
    std::size_t urdf_lookup_attempts = 0;
    std::size_t urdf_lookup_successes = 0;
    std::string first_missing_frame;
    bool used_forces = false;
    bool used_taxels = false;

    if (left_module_index_ >= 0 && static_cast<std::size_t>(left_module_index_) < modules_count) {
      update_baseline(
        msg->x_modules[static_cast<std::size_t>(left_module_index_)],
        layout_left_.index_centers.size(),
        baseline_left_,
        now);
    }
    if (right_module_index_ >= 0 && static_cast<std::size_t>(right_module_index_) < modules_count) {
      update_baseline(
        msg->x_modules[static_cast<std::size_t>(right_module_index_)],
        layout_right_.index_centers.size(),
        baseline_right_,
        now);
    }

    if (left_module_index_ >= 0 && static_cast<std::size_t>(left_module_index_) < modules_count) {
      append_module_payload(
        msg->x_modules[static_cast<std::size_t>(left_module_index_)], "left", layout_left_,
        baseline_left_,
        grid_left_force_x_sign_, grid_left_force_y_sign_,
        urdf_left_force_x_sign_, urdf_left_force_y_sign_,
        used_forces, used_taxels,
        grid_points, urdf_points,
        urdf_lookup_attempts, urdf_lookup_successes, first_missing_frame);
    }

    if (right_module_index_ >= 0 && static_cast<std::size_t>(right_module_index_) < modules_count) {
      append_module_payload(
        msg->x_modules[static_cast<std::size_t>(right_module_index_)], "right", layout_right_,
        baseline_right_,
        grid_right_force_x_sign_, grid_right_force_y_sign_,
        urdf_right_force_x_sign_, urdf_right_force_y_sign_,
        used_forces, used_taxels,
        grid_points, urdf_points,
        urdf_lookup_attempts, urdf_lookup_successes, first_missing_frame);
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

    std::string grasp_json;
    std::string grasp_event_json;
    double grasp_age_ms = 0.0;
    double grasp_event_age_ms = 0.0;
    const bool has_grasp = get_recent_grasp_json(false, grasp_json, grasp_age_ms);
    const bool has_grasp_event = get_recent_grasp_json(true, grasp_event_json, grasp_event_age_ms);

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
    json << "\"module_gap\":" << module_gap_ << ",";
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
    if (has_grasp) {
      json << ",\"grasp\":" << grasp_json;
      json << ",\"grasp_age_ms\":" << grasp_age_ms;
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

  int left_module_index_{0};
  int right_module_index_{1};
  int grid_rows_{4};
  int grid_cols_{6};
  double cell_size_{0.015};
  double module_gap_{0.04};
  double origin_x_{0.0};
  double origin_y_{0.0};
  bool row_flip_right_{true};
  bool col_flip_right_{true};
  std::vector<int64_t> grid_index_map_left_;
  std::vector<int64_t> grid_index_map_right_;
  std::vector<int64_t> grid_separator_cols_left_;
  std::vector<int64_t> grid_separator_cols_right_;

  double force_scale_{1.0};
  bool use_fz_only_{false};
  bool use_axis_normalization_{true};
  double xy_force_range_{0.8};
  double z_force_range_{14.0};
  bool use_taxels_when_no_forces_{true};
  double xy_taxel_range_{1350.0};
  double z_taxel_range_{10085.0};
  double baseline_duration_sec_{3.0};
  double baseline_deadband_xy_{0.01};
  double baseline_deadband_z_{0.05};
  double baseline_deadband_taxel_xy_{10.0};
  double baseline_deadband_taxel_z_{60.0};

  double grid_left_force_x_sign_{1.0};
  double grid_left_force_y_sign_{-1.0};
  double grid_right_force_x_sign_{-1.0};
  double grid_right_force_y_sign_{-1.0};

  double urdf_left_force_x_sign_{1.0};
  double urdf_left_force_y_sign_{1.0};
  double urdf_right_force_x_sign_{1.0};
  double urdf_right_force_y_sign_{1.0};

  double max_publish_rate_hz_{20.0};
  double publish_period_sec_{0.05};
  bool emit_urdf_points_{false};
  bool freeze_urdf_positions_{true};
  double tf_cache_ttl_sec_{0.5};
  bool include_grasp_meta_{true};
  std::string grasp_telemetry_topic_{"/x_telemetry_2f/grasp_telemetry"};
  std::string grasp_event_topic_{"/x_telemetry_2f/grasp_event"};
  double grasp_meta_timeout_sec_{2.0};

  ModuleLayout layout_left_;
  ModuleLayout layout_right_;
  BaselineState baseline_left_;
  BaselineState baseline_right_;
  std::unordered_map<std::string, TfCacheEntry> tf_cache_;
  mutable std::mutex grasp_meta_mutex_;
  std::string latest_grasp_telemetry_json_;
  std::string latest_grasp_event_json_;
  rclcpp::Time latest_grasp_telemetry_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time latest_grasp_event_stamp_{0, 0, RCL_ROS_TIME};

  std::chrono::steady_clock::time_point last_publish_steady_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::Subscription<xela_taxel_msgs::msg::XTaxelSensorTArray>::SharedPtr sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr grasp_telemetry_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr grasp_event_sub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_parameters_handle_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<XelaTaxelWebBridgeNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
