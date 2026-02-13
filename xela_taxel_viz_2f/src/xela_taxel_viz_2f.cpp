#include <algorithm>
#include <array>
#include <cmath>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <xela_taxel_msgs/msg/x_taxel_sensor_t_array.hpp>

namespace {
struct ColorRGB {
  double r;
  double g;
  double b;
};

struct Vec3 {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

ColorRGB lerpColor(const ColorRGB &low, const ColorRGB &high, double t) {
  t = std::clamp(t, 0.0, 1.0);
  return {low.r + (high.r - low.r) * t,
          low.g + (high.g - low.g) * t,
          low.b + (high.b - low.b) * t};
}

std_msgs::msg::ColorRGBA toColorMsg(const ColorRGB &c, double a) {
  std_msgs::msg::ColorRGBA msg;
  msg.r = static_cast<float>(c.r);
  msg.g = static_cast<float>(c.g);
  msg.b = static_cast<float>(c.b);
  msg.a = static_cast<float>(a);
  return msg;
}

std::vector<int64_t> getIntArrayParam(const rclcpp::Node &node,
                                      const std::string &name,
                                      const rclcpp::Logger &logger) {
  rclcpp::Parameter param;
  if (!node.get_parameter(name, param)) {
    return {};
  }
  if (param.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY) {
    RCLCPP_WARN(logger, "Parameter '%s' has invalid type; ignoring.", name.c_str());
    return {};
  }
  return param.as_integer_array();
}

std::vector<double> getVecParam(const rclcpp::Node &node,
                                const std::string &name,
                                const std::vector<double> &defaults,
                                const rclcpp::Logger &logger) {
  std::vector<double> value = defaults;
  try {
    value = node.get_parameter(name).as_double_array();
  } catch (const rclcpp::ParameterTypeException &) {
    RCLCPP_WARN(logger, "Parameter '%s' has invalid type; using defaults.", name.c_str());
    return defaults;
  }
  if (value.size() != defaults.size()) {
    RCLCPP_WARN(logger, "Parameter '%s' has %zu elements; expected %zu. Using defaults.",
                name.c_str(), value.size(), defaults.size());
    return defaults;
  }
  return value;
}

}  // namespace

class XelaTaxelViz2F : public rclcpp::Node {
public:
  XelaTaxelViz2F() : Node("xela_taxel_viz_2f") {
    declare_parameter<std::string>("in_topic", "/x_taxel_2f");
    declare_parameter<std::string>("out_topic", "/x_taxel_2f/markers");
    declare_parameter<std::string>("frame_id", "x_taxel_viz");
    declare_parameter<std::string>("viz_mode", "grid");
    declare_parameter<bool>("overlay_grid_in_urdf", true);
    declare_parameter<double>("urdf_left_force_x_sign", 1.0);
    declare_parameter<double>("urdf_left_force_y_sign", 1.0);
    declare_parameter<double>("urdf_right_force_x_sign", 1.0);
    declare_parameter<double>("urdf_right_force_y_sign", 1.0);

    declare_parameter<int>("grid_rows", 4);
    declare_parameter<int>("grid_cols", 6);
    declare_parameter<std::vector<int64_t>>("grid_index_map", std::vector<int64_t>{});
    declare_parameter<std::vector<int64_t>>("grid_index_map_left", std::vector<int64_t>{});
    declare_parameter<std::vector<int64_t>>("grid_index_map_right", std::vector<int64_t>{});
    declare_parameter<std::vector<int64_t>>("grid_separator_cols_left", std::vector<int64_t>{});
    declare_parameter<std::vector<int64_t>>("grid_separator_cols_right", std::vector<int64_t>{});
    declare_parameter<double>("cell_size", 0.015);
    declare_parameter<double>("module_gap", 0.04);
    declare_parameter<double>("origin_x", 0.0);
    declare_parameter<double>("origin_y", 0.0);

    declare_parameter<int>("left_module_index", 0);
    declare_parameter<int>("right_module_index", 1);
    declare_parameter<bool>("row_flip_right", true);
    declare_parameter<bool>("col_flip_right", true);

    declare_parameter<double>("force_scale", 1.0);
    declare_parameter<double>("max_force", 0.02);
    declare_parameter<bool>("use_fz_only", false);
    declare_parameter<bool>("use_xy_direction", true);
    declare_parameter<double>("baseline_duration_sec", 2.0);
    declare_parameter<std::string>("marker_stamp_mode", "keep");
    declare_parameter<double>("marker_time_offset_sec", 0.0);
    declare_parameter<double>("baseline_deadband_xy", 0.0);
    declare_parameter<double>("baseline_deadband_z", 0.0);
    declare_parameter<double>("baseline_deadband_taxel_xy", 0.0);
    declare_parameter<double>("baseline_deadband_taxel_z", 0.0);
    declare_parameter<bool>("use_axis_normalization", true);
    declare_parameter<double>("xy_force_range", 0.8);
    declare_parameter<double>("z_force_range", 14.0);
    declare_parameter<bool>("use_taxels_when_no_forces", true);
    declare_parameter<double>("xy_taxel_range", 1350.0);
    declare_parameter<double>("z_taxel_range", 10085.0);
    declare_parameter<double>("urdf_taxel_scale", 0.3);

    declare_parameter<double>("circle_min_radius", 0.002);
    declare_parameter<double>("circle_max_radius", 0.008);
    declare_parameter<double>("circle_height", 0.002);
    declare_parameter<bool>("use_cell_scale", true);
    declare_parameter<double>("circle_area_scale", 2.0);

    declare_parameter<double>("arrow_min_length", 0.004);
    declare_parameter<double>("arrow_max_length", 0.016);
    declare_parameter<double>("arrow_shaft_diameter", 0.001);
    declare_parameter<double>("arrow_head_diameter", 0.002);
    declare_parameter<double>("arrow_head_length", 0.003);
    declare_parameter<double>("arrow_length_scale", 2.0);
    declare_parameter<double>("min_marker_scale", 1e-4);

    declare_parameter<double>("grid_thickness", 0.0008);
    declare_parameter<bool>("grid_lines_enabled", true);
    declare_parameter<double>("grid_line_width", 0.001);
    declare_parameter<double>("grid_line_alpha", 0.2);
    declare_parameter<std::vector<double>>("grid_line_color", {0.2, 0.2, 0.2});
    declare_parameter<double>("grid_separator_width", 0.002);
    declare_parameter<double>("grid_separator_alpha", 0.6);
    declare_parameter<std::vector<double>>("grid_separator_color", {0.1, 0.1, 0.1});

    declare_parameter<double>("grid_alpha", 1.0);
    declare_parameter<double>("circle_alpha", 0.9);
    declare_parameter<double>("arrow_alpha", 1.0);
    declare_parameter<bool>("debug_touch_stats", false);

    declare_parameter<std::vector<double>>("grid_color", {0.91, 0.89, 0.86});
    declare_parameter<std::string>("style_preset", "default");
    declare_parameter<std::vector<double>>("color_low", {0.2, 0.4, 1.0});
    declare_parameter<std::vector<double>>("color_high", {1.0, 0.2, 0.2});
    declare_parameter<double>("left_force_x_sign", 1.0);
    declare_parameter<double>("left_force_y_sign", -1.0);
    declare_parameter<double>("right_force_x_sign", -1.0);
    declare_parameter<double>("right_force_y_sign", -1.0);

    in_topic_ = get_parameter("in_topic").as_string();
    out_topic_ = get_parameter("out_topic").as_string();
    frame_id_ = get_parameter("frame_id").as_string();
    viz_mode_ = get_parameter("viz_mode").as_string();
    overlay_grid_in_urdf_ = get_parameter("overlay_grid_in_urdf").as_bool();
    urdf_left_force_x_sign_ = get_parameter("urdf_left_force_x_sign").as_double();
    urdf_left_force_y_sign_ = get_parameter("urdf_left_force_y_sign").as_double();
    urdf_right_force_x_sign_ = get_parameter("urdf_right_force_x_sign").as_double();
    urdf_right_force_y_sign_ = get_parameter("urdf_right_force_y_sign").as_double();

    grid_rows_ = get_parameter("grid_rows").as_int();
    grid_cols_ = get_parameter("grid_cols").as_int();
    grid_index_map_ = getIntArrayParam(*this, "grid_index_map", get_logger());
    grid_index_map_left_ = getIntArrayParam(*this, "grid_index_map_left", get_logger());
    grid_index_map_right_ = getIntArrayParam(*this, "grid_index_map_right", get_logger());
    grid_separator_cols_left_ = getIntArrayParam(*this, "grid_separator_cols_left", get_logger());
    grid_separator_cols_right_ = getIntArrayParam(*this, "grid_separator_cols_right", get_logger());
    cell_size_ = get_parameter("cell_size").as_double();
    module_gap_ = get_parameter("module_gap").as_double();
    origin_x_ = get_parameter("origin_x").as_double();
    origin_y_ = get_parameter("origin_y").as_double();

    left_module_index_ = get_parameter("left_module_index").as_int();
    right_module_index_ = get_parameter("right_module_index").as_int();
    row_flip_right_ = get_parameter("row_flip_right").as_bool();
    col_flip_right_ = get_parameter("col_flip_right").as_bool();

    force_scale_ = get_parameter("force_scale").as_double();
    max_force_ = get_parameter("max_force").as_double();
    use_fz_only_ = get_parameter("use_fz_only").as_bool();
    use_xy_direction_ = get_parameter("use_xy_direction").as_bool();
    baseline_duration_sec_ = get_parameter("baseline_duration_sec").as_double();
    baseline_deadband_xy_ = get_parameter("baseline_deadband_xy").as_double();
    baseline_deadband_z_ = get_parameter("baseline_deadband_z").as_double();
    baseline_deadband_taxel_xy_ = get_parameter("baseline_deadband_taxel_xy").as_double();
    baseline_deadband_taxel_z_ = get_parameter("baseline_deadband_taxel_z").as_double();
    marker_stamp_mode_ = get_parameter("marker_stamp_mode").as_string();
    marker_time_offset_sec_ = get_parameter("marker_time_offset_sec").as_double();
    use_axis_normalization_ = get_parameter("use_axis_normalization").as_bool();
    xy_force_range_ = get_parameter("xy_force_range").as_double();
    z_force_range_ = get_parameter("z_force_range").as_double();
    use_taxels_when_no_forces_ = get_parameter("use_taxels_when_no_forces").as_bool();
    xy_taxel_range_ = get_parameter("xy_taxel_range").as_double();
    z_taxel_range_ = get_parameter("z_taxel_range").as_double();
    urdf_taxel_scale_ = get_parameter("urdf_taxel_scale").as_double();

    circle_min_radius_ = get_parameter("circle_min_radius").as_double();
    circle_max_radius_ = get_parameter("circle_max_radius").as_double();
    circle_height_ = get_parameter("circle_height").as_double();
    use_cell_scale_ = get_parameter("use_cell_scale").as_bool();
    circle_area_scale_ = get_parameter("circle_area_scale").as_double();

    arrow_min_length_ = get_parameter("arrow_min_length").as_double();
    arrow_max_length_ = get_parameter("arrow_max_length").as_double();
    arrow_shaft_diameter_ = get_parameter("arrow_shaft_diameter").as_double();
    arrow_head_diameter_ = get_parameter("arrow_head_diameter").as_double();
    arrow_head_length_ = get_parameter("arrow_head_length").as_double();
    arrow_length_scale_ = get_parameter("arrow_length_scale").as_double();
    min_marker_scale_ = get_parameter("min_marker_scale").as_double();

    grid_thickness_ = get_parameter("grid_thickness").as_double();
    grid_lines_enabled_ = get_parameter("grid_lines_enabled").as_bool();
    grid_line_width_ = get_parameter("grid_line_width").as_double();
    grid_line_alpha_ = get_parameter("grid_line_alpha").as_double();
    grid_separator_width_ = get_parameter("grid_separator_width").as_double();
    grid_separator_alpha_ = get_parameter("grid_separator_alpha").as_double();

    grid_alpha_ = get_parameter("grid_alpha").as_double();
    circle_alpha_ = get_parameter("circle_alpha").as_double();
    arrow_alpha_ = get_parameter("arrow_alpha").as_double();
    debug_touch_stats_ = get_parameter("debug_touch_stats").as_bool();

    auto grid_color_vec = getVecParam(*this, "grid_color", {0.91, 0.89, 0.86}, get_logger());
    auto grid_line_color_vec = getVecParam(*this, "grid_line_color", {0.2, 0.2, 0.2}, get_logger());
    auto grid_separator_color_vec = getVecParam(*this, "grid_separator_color", {0.1, 0.1, 0.1}, get_logger());
    auto color_low_vec = getVecParam(*this, "color_low", {0.2, 0.4, 1.0}, get_logger());
    auto color_high_vec = getVecParam(*this, "color_high", {1.0, 0.2, 0.2}, get_logger());
    grid_color_ = {grid_color_vec[0], grid_color_vec[1], grid_color_vec[2]};
    grid_line_color_ = {grid_line_color_vec[0], grid_line_color_vec[1], grid_line_color_vec[2]};
    grid_separator_color_ = {grid_separator_color_vec[0], grid_separator_color_vec[1], grid_separator_color_vec[2]};
    color_low_ = {color_low_vec[0], color_low_vec[1], color_low_vec[2]};
    color_high_ = {color_high_vec[0], color_high_vec[1], color_high_vec[2]};

    left_force_x_sign_ = get_parameter("left_force_x_sign").as_double();
    left_force_y_sign_ = get_parameter("left_force_y_sign").as_double();
    right_force_x_sign_ = get_parameter("right_force_x_sign").as_double();
    right_force_y_sign_ = get_parameter("right_force_y_sign").as_double();

    applyStylePreset(get_parameter("style_preset").as_string());

    if (grid_rows_ <= 0 || grid_cols_ <= 0) {
      RCLCPP_WARN(get_logger(), "Invalid grid size; using 4x6.");
      grid_rows_ = 4;
      grid_cols_ = 6;
    }
    grid_cells_ = static_cast<size_t>(grid_rows_ * grid_cols_);
    auto validate_index_map = [this](std::vector<int64_t> &map,
                                     const std::string &name,
                                     size_t grid_cells,
                                     size_t &out_max) -> bool {
      if (map.empty()) {
        return false;
      }
      if (map.size() != grid_cells) {
        RCLCPP_WARN(get_logger(),
                    "%s has %zu cells; expected %zu. Ignoring map.",
                    name.c_str(), map.size(), grid_cells);
        map.clear();
        return false;
      }
      int64_t max_index = -1;
      for (const auto &val : map) {
        if (val > max_index) {
          max_index = val;
        }
      }
      if (max_index < 0) {
        RCLCPP_WARN(get_logger(), "%s has no valid indices; ignoring map.", name.c_str());
        map.clear();
        return false;
      }
      out_max = static_cast<size_t>(max_index);
      return true;
    };

    size_t max_left = 0;
    size_t max_right = 0;
    bool has_left = validate_index_map(grid_index_map_left_, "grid_index_map_left",
                                       grid_cells_, max_left);
    bool has_right = validate_index_map(grid_index_map_right_, "grid_index_map_right",
                                        grid_cells_, max_right);

    size_t max_default = 0;
    bool has_default = validate_index_map(grid_index_map_, "grid_index_map",
                                          grid_cells_, max_default);

    if (!has_left && !has_right && has_default) {
      grid_index_map_left_ = grid_index_map_;
      grid_index_map_right_ = grid_index_map_;
      has_left = true;
      has_right = true;
      max_left = max_default;
      max_right = max_default;
    }

    grid_size_ = grid_cells_;
    if (has_left) {
      grid_size_ = std::max(grid_size_, max_left + 1);
    }
    if (has_right) {
      grid_size_ = std::max(grid_size_, max_right + 1);
    }

    if (use_cell_scale_) {
      double cell_area = cell_size_ * cell_size_;
      double area_scale = std::max(0.0, circle_area_scale_);
      circle_min_radius_ = 0.0;
      circle_max_radius_ = std::sqrt((cell_area * area_scale) / M_PI);
      arrow_min_length_ = 0.0;
      arrow_max_length_ = std::max(0.0, cell_size_ * arrow_length_scale_);
    }

    if (baseline_duration_sec_ < 0.0) {
      baseline_duration_sec_ = 0.0;
    }
    if (baseline_deadband_xy_ < 0.0) {
      baseline_deadband_xy_ = 0.0;
    }
    if (baseline_deadband_z_ < 0.0) {
      baseline_deadband_z_ = 0.0;
    }
    if (baseline_deadband_taxel_xy_ < 0.0) {
      baseline_deadband_taxel_xy_ = 0.0;
    }
    if (baseline_deadband_taxel_z_ < 0.0) {
      baseline_deadband_taxel_z_ = 0.0;
    }
    if (marker_stamp_mode_.empty()) {
      marker_stamp_mode_ = "keep";
    }
    if (std::isnan(marker_time_offset_sec_) || std::isinf(marker_time_offset_sec_)) {
      marker_time_offset_sec_ = 0.0;
    }
    if (min_marker_scale_ <= 0.0) {
      min_marker_scale_ = 1e-6;
    }
    if (circle_height_ < min_marker_scale_) {
      circle_height_ = min_marker_scale_;
    }
    if (arrow_shaft_diameter_ < min_marker_scale_) {
      arrow_shaft_diameter_ = min_marker_scale_;
    }
    if (arrow_head_diameter_ < min_marker_scale_) {
      arrow_head_diameter_ = min_marker_scale_;
    }
    if (arrow_head_length_ < min_marker_scale_) {
      arrow_head_length_ = min_marker_scale_;
    }
    if (grid_thickness_ < min_marker_scale_) {
      grid_thickness_ = min_marker_scale_;
    }
    if (grid_line_width_ < min_marker_scale_) {
      grid_line_width_ = min_marker_scale_;
    }
    if (grid_separator_width_ < min_marker_scale_) {
      grid_separator_width_ = min_marker_scale_;
    }
    if (xy_force_range_ <= 0.0) {
      RCLCPP_WARN(get_logger(), "xy_force_range <= 0, defaulting to 0.8.");
      xy_force_range_ = 0.8;
    }
    if (z_force_range_ <= 0.0) {
      RCLCPP_WARN(get_logger(), "z_force_range <= 0, defaulting to 14.0.");
      z_force_range_ = 14.0;
    }
    if (xy_taxel_range_ <= 0.0) {
      RCLCPP_WARN(get_logger(), "xy_taxel_range <= 0, defaulting to 1350.0.");
      xy_taxel_range_ = 1350.0;
    }
    if (z_taxel_range_ <= 0.0) {
      RCLCPP_WARN(get_logger(), "z_taxel_range <= 0, defaulting to 10085.0.");
      z_taxel_range_ = 10085.0;
    }
    if (urdf_taxel_scale_ <= 0.0) {
      RCLCPP_WARN(get_logger(), "urdf_taxel_scale <= 0, defaulting to 0.3.");
      urdf_taxel_scale_ = 0.3;
    }
    if (grid_line_width_ <= 0.0) {
      grid_line_width_ = 0.001;
    }

    if (viz_mode_ != "grid" && viz_mode_ != "urdf") {
      RCLCPP_WARN(get_logger(), "Unknown viz_mode '%s'; using 'grid'.", viz_mode_.c_str());
      viz_mode_ = "grid";
    }

    buildGridMarkers();

    rclcpp::QoS qos(10);
    qos.transient_local();

    pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(out_topic_, qos);
    sub_ = create_subscription<xela_taxel_msgs::msg::XTaxelSensorTArray>(
      in_topic_, 10, std::bind(&XelaTaxelViz2F::onArray, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "xela_taxel_viz_2f started. in: %s out: %s", in_topic_.c_str(),
                out_topic_.c_str());
  }

private:
  struct ModuleLayout {
    double base_x;
    double base_y;
    bool row_flip;
    bool col_flip;
    bool has_index_map = false;
    std::vector<int64_t> index_map;
    std::vector<int64_t> separator_cols;
    std::vector<bool> cell_enabled;
    std::vector<geometry_msgs::msg::Point> cell_centers;
    std::vector<geometry_msgs::msg::Point> index_centers;
    std::vector<bool> index_valid;
  };

  struct BaselineState {
    BaselineState() : start_time(0, 0, RCL_ROS_TIME) {}
    bool ready = false;
    bool started = false;
    rclcpp::Time start_time;
    size_t samples = 0;
    std::vector<Vec3> force_sum;
    std::vector<Vec3> taxel_sum;
    std::vector<Vec3> force_base;
    std::vector<Vec3> taxel_base;
  };

  void buildGridMarkers() {
    grid_markers_.clear();
    layouts_.clear();

    ModuleLayout left_layout;
    left_layout.base_x = origin_x_;
    left_layout.base_y = origin_y_;
    left_layout.row_flip = false;
    left_layout.col_flip = false;
    if (!grid_index_map_left_.empty()) {
      left_layout.has_index_map = true;
      left_layout.index_map = grid_index_map_left_;
    }
    if (!grid_separator_cols_left_.empty()) {
      left_layout.separator_cols = grid_separator_cols_left_;
    }
    fillCenters(left_layout);

    ModuleLayout right_layout;
    right_layout.base_x = origin_x_ + grid_cols_ * cell_size_ + module_gap_;
    right_layout.base_y = origin_y_;
    right_layout.row_flip = row_flip_right_;
    right_layout.col_flip = col_flip_right_;
    if (!grid_index_map_right_.empty()) {
      right_layout.has_index_map = true;
      right_layout.index_map = grid_index_map_right_;
    }
    if (!grid_separator_cols_right_.empty()) {
      right_layout.separator_cols = grid_separator_cols_right_;
    }
    fillCenters(right_layout);

    layouts_.push_back(left_layout);
    layouts_.push_back(right_layout);

    const std::vector<std::string> module_ns = {"left", "right"};
    for (size_t module_idx = 0; module_idx < layouts_.size(); ++module_idx) {
      const auto &layout = layouts_[module_idx];
      for (size_t cell_idx = 0; cell_idx < layout.cell_centers.size(); ++cell_idx) {
        if (!layout.cell_enabled.empty() && !layout.cell_enabled[cell_idx]) {
          continue;
        }
        visualization_msgs::msg::Marker m;
        m.ns = "grid_" + module_ns[module_idx];
        m.id = static_cast<int>(module_idx * 1000 + cell_idx);
        m.type = visualization_msgs::msg::Marker::CUBE;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.pose.position = layout.cell_centers[cell_idx];
        m.pose.position.z = -grid_thickness_ * 0.5;
        m.pose.orientation.w = 1.0;
        m.scale.x = cell_size_;
        m.scale.y = cell_size_;
        m.scale.z = grid_thickness_;
        m.color = toColorMsg(grid_color_, grid_alpha_);
        grid_markers_.push_back(m);
      }

      if (grid_lines_enabled_) {
        visualization_msgs::msg::Marker lines;
        lines.ns = "grid_lines_" + module_ns[module_idx];
        lines.id = static_cast<int>(module_idx * 1000 + 900);
        lines.type = visualization_msgs::msg::Marker::LINE_LIST;
        lines.action = visualization_msgs::msg::Marker::ADD;
        lines.pose.orientation.w = 1.0;
        lines.scale.x = grid_line_width_;
        lines.color = toColorMsg(grid_line_color_, grid_line_alpha_);

        double x0 = layout.base_x;
        double y0 = layout.base_y;
        double x1 = layout.base_x + grid_cols_ * cell_size_;
        double y1 = layout.base_y - grid_rows_ * cell_size_;

        for (int r = 0; r <= grid_rows_; ++r) {
          double y = y0 - static_cast<double>(r) * cell_size_;
          geometry_msgs::msg::Point p0;
          geometry_msgs::msg::Point p1;
          p0.x = x0; p0.y = y; p0.z = 0.0;
          p1.x = x1; p1.y = y; p1.z = 0.0;
          lines.points.push_back(p0);
          lines.points.push_back(p1);
        }

        for (int c = 0; c <= grid_cols_; ++c) {
          double x = x0 + static_cast<double>(c) * cell_size_;
          geometry_msgs::msg::Point p0;
          geometry_msgs::msg::Point p1;
          p0.x = x; p0.y = y0; p0.z = 0.0;
          p1.x = x; p1.y = y1; p1.z = 0.0;
          lines.points.push_back(p0);
          lines.points.push_back(p1);
        }

        grid_markers_.push_back(std::move(lines));
      }

      if (!layout.separator_cols.empty() && grid_separator_width_ > 0.0 &&
          grid_separator_alpha_ > 0.0) {
        visualization_msgs::msg::Marker sep;
        sep.ns = "grid_sep_" + module_ns[module_idx];
        sep.id = static_cast<int>(module_idx * 1000 + 950);
        sep.type = visualization_msgs::msg::Marker::LINE_LIST;
        sep.action = visualization_msgs::msg::Marker::ADD;
        sep.pose.orientation.w = 1.0;
        sep.scale.x = grid_separator_width_;
        sep.color = toColorMsg(grid_separator_color_, grid_separator_alpha_);

        double y0 = layout.base_y;
        double y1 = layout.base_y - grid_rows_ * cell_size_;
        for (auto col : layout.separator_cols) {
          if (col <= 0 || col >= grid_cols_) {
            continue;
          }
          double x = layout.base_x + static_cast<double>(col) * cell_size_;
          geometry_msgs::msg::Point p0;
          geometry_msgs::msg::Point p1;
          p0.x = x; p0.y = y0; p0.z = 0.0;
          p1.x = x; p1.y = y1; p1.z = 0.0;
          sep.points.push_back(p0);
          sep.points.push_back(p1);
        }
        if (!sep.points.empty()) {
          grid_markers_.push_back(std::move(sep));
        }
      }
    }
  }

  void fillCenters(ModuleLayout &layout) {
    layout.cell_centers.clear();
    layout.cell_enabled.clear();
    layout.cell_centers.reserve(grid_cells_);
    layout.cell_enabled.reserve(grid_cells_);
    geometry_msgs::msg::Point zero_point;
    layout.index_centers.assign(grid_size_, zero_point);
    layout.index_valid.assign(grid_size_, false);

    for (int r = 0; r < grid_rows_; ++r) {
      for (int c = 0; c < grid_cols_; ++c) {
        int draw_r = layout.row_flip ? (grid_rows_ - 1 - r) : r;
        int draw_c = layout.col_flip ? (grid_cols_ - 1 - c) : c;
        geometry_msgs::msg::Point p;
        p.x = layout.base_x + (static_cast<double>(draw_c) + 0.5) * cell_size_;
        p.y = layout.base_y - (static_cast<double>(draw_r) + 0.5) * cell_size_;
        p.z = 0.0;
        layout.cell_centers.push_back(p);

        bool enabled = true;
        if (layout.has_index_map) {
          const size_t cell_idx = static_cast<size_t>(r * grid_cols_ + c);
          const int64_t map_idx = layout.index_map[cell_idx];
          if (map_idx >= 0 && static_cast<size_t>(map_idx) < grid_size_) {
            layout.index_centers[static_cast<size_t>(map_idx)] = p;
            layout.index_valid[static_cast<size_t>(map_idx)] = true;
          } else {
            enabled = false;
          }
        }
        layout.cell_enabled.push_back(enabled);
      }
    }

    if (!layout.has_index_map) {
      layout.index_centers = layout.cell_centers;
      layout.index_valid.assign(grid_size_, true);
    }
  }

  rclcpp::Time getStamp(const std_msgs::msg::Header &header) {
    if (marker_stamp_mode_ == "zero") {
      return applyMarkerOffset(rclcpp::Time(0, 0, get_clock()->get_clock_type()));
    }
    if (marker_stamp_mode_ == "now") {
      return applyMarkerOffset(now());
    }
    if (header.stamp.sec == 0 && header.stamp.nanosec == 0) {
      return applyMarkerOffset(now());
    }
    rclcpp::Time stamp(header.stamp);
    rclcpp::Time current = now();
    if (stamp > current) {
      return applyMarkerOffset(current);
    }
    return applyMarkerOffset(stamp);
  }

  rclcpp::Time applyMarkerOffset(const rclcpp::Time &stamp) {
    if (marker_time_offset_sec_ == 0.0) {
      return stamp;
    }
    rclcpp::Duration offset = rclcpp::Duration::from_seconds(marker_time_offset_sec_);
    rclcpp::Time adjusted = stamp + offset;
    rclcpp::Time current = now();
    if (adjusted > current) {
      return current;
    }
    if (adjusted < rclcpp::Time(0, 0, stamp.get_clock_type())) {
      return rclcpp::Time(0, 0, stamp.get_clock_type());
    }
    return adjusted;
  }

  void ensureBaselineSize(BaselineState &state) {
    if (state.force_sum.size() == grid_size_) {
      return;
    }
    state.force_sum.assign(grid_size_, {});
    state.taxel_sum.assign(grid_size_, {});
    state.force_base.assign(grid_size_, {});
    state.taxel_base.assign(grid_size_, {});
    state.samples = 0;
  }

  void updateBaseline(const xela_taxel_msgs::msg::XTaxelSensorTArray &msg,
                      int module_index,
                      BaselineState &state,
                      const rclcpp::Time &stamp) {
    if (baseline_duration_sec_ <= 0.0) {
      state.ready = true;
      return;
    }
    if (module_index < 0 || static_cast<size_t>(module_index) >= msg.x_modules.size()) {
      return;
    }

    ensureBaselineSize(state);

    if (!state.started) {
      state.start_time = stamp;
      state.started = true;
    }

    if (state.ready) {
      return;
    }

    double elapsed = (stamp - state.start_time).seconds();
    if (elapsed < 0.0) {
      state.start_time = stamp;
      elapsed = 0.0;
    }

    if (elapsed <= baseline_duration_sec_) {
      const auto &module = msg.x_modules[static_cast<size_t>(module_index)];
      for (size_t idx = 0; idx < grid_size_; ++idx) {
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
        for (size_t idx = 0; idx < grid_size_; ++idx) {
          state.force_base[idx].x = state.force_sum[idx].x / state.samples;
          state.force_base[idx].y = state.force_sum[idx].y / state.samples;
          state.force_base[idx].z = state.force_sum[idx].z / state.samples;
          state.taxel_base[idx].x = state.taxel_sum[idx].x / state.samples;
          state.taxel_base[idx].y = state.taxel_sum[idx].y / state.samples;
          state.taxel_base[idx].z = state.taxel_sum[idx].z / state.samples;
        }
      }
      state.ready = true;
      RCLCPP_INFO(get_logger(), "Baseline computed for module index %d with %zu samples.",
                  module_index, state.samples);
    }
  }

  void onArray(const xela_taxel_msgs::msg::XTaxelSensorTArray::SharedPtr msg) {
    visualization_msgs::msg::MarkerArray out;
    rclcpp::Time stamp = getStamp(msg->header);

    bool show_grid = (viz_mode_ == "grid") || (viz_mode_ == "urdf" && overlay_grid_in_urdf_);
    if (show_grid) {
      for (auto marker : grid_markers_) {
        marker.header.frame_id = frame_id_;
        marker.header.stamp = stamp;
        out.markers.push_back(std::move(marker));
      }
    }

    updateBaseline(*msg, left_module_index_, baselines_[0], stamp);
    updateBaseline(*msg, right_module_index_, baselines_[1], stamp);

    if (viz_mode_ == "grid") {
      appendModuleMarkersGrid(*msg, left_module_index_, baselines_[0], layouts_[0], 0, "left", out);
      appendModuleMarkersGrid(*msg, right_module_index_, baselines_[1], layouts_[1], 1, "right", out);
    } else {
      appendModuleMarkersUrdf(*msg, left_module_index_, baselines_[0], 0, "left", out);
      appendModuleMarkersUrdf(*msg, right_module_index_, baselines_[1], 1, "right", out);
    }

    pub_->publish(out);
  }

  void appendModuleMarkersGrid(const xela_taxel_msgs::msg::XTaxelSensorTArray &msg,
                               int module_index,
                               const BaselineState &baseline,
                               const ModuleLayout &layout,
                               int module_offset,
                               const std::string &module_ns,
                               visualization_msgs::msg::MarkerArray &out) {
    if (module_index < 0) {
      return;
    }
    if (static_cast<size_t>(module_index) >= msg.x_modules.size()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Module index %d out of range (size=%zu).", module_index,
                           msg.x_modules.size());
      return;
    }

    const auto &module = msg.x_modules[static_cast<size_t>(module_index)];
    const bool use_taxels = use_taxels_when_no_forces_ && module.forces.empty() &&
                            !module.taxels.empty();
    const double xy_range = use_taxels ? xy_taxel_range_ : xy_force_range_;
    const double z_range = use_taxels ? z_taxel_range_ : z_force_range_;

    const rclcpp::Time stamp = getStamp(msg.header);

    size_t used = 0;
    double max_mag = 0.0;
    double sum_mag = 0.0;

    for (size_t idx = 0; idx < grid_size_; ++idx) {
      if (!layout.index_valid.empty() && !layout.index_valid[idx]) {
        continue;
      }
      double fx = 0.0;
      double fy = 0.0;
      double fz = 0.0;
      if (use_taxels) {
        if (idx < module.taxels.size()) {
          fx = static_cast<double>(module.taxels[idx].x);
          fy = static_cast<double>(module.taxels[idx].y);
          fz = static_cast<double>(module.taxels[idx].z);
        }
      } else if (idx < module.forces.size()) {
        fx = static_cast<double>(module.forces[idx].x);
        fy = static_cast<double>(module.forces[idx].y);
        fz = static_cast<double>(module.forces[idx].z);
      }

      if (baseline_duration_sec_ > 0.0 && !baseline.ready) {
        fx = 0.0;
        fy = 0.0;
        fz = 0.0;
      } else if (use_taxels && baseline.taxel_base.size() == grid_size_) {
        fx -= baseline.taxel_base[idx].x;
        fy -= baseline.taxel_base[idx].y;
        fz -= baseline.taxel_base[idx].z;
      } else if (!use_taxels && baseline.force_base.size() == grid_size_) {
        fx -= baseline.force_base[idx].x;
        fy -= baseline.force_base[idx].y;
        fz -= baseline.force_base[idx].z;
      }

      const double db_xy = use_taxels ? baseline_deadband_taxel_xy_ : baseline_deadband_xy_;
      const double db_z = use_taxels ? baseline_deadband_taxel_z_ : baseline_deadband_z_;
      if (std::fabs(fx) < db_xy) {
        fx = 0.0;
      }
      if (std::fabs(fy) < db_xy) {
        fy = 0.0;
      }
      if (std::fabs(fz) < db_z) {
        fz = 0.0;
      }

      fx *= force_scale_;
      fy *= force_scale_;
      fz *= force_scale_;
      double sign_x = (module_offset == 0) ? left_force_x_sign_ : right_force_x_sign_;
      double sign_y = (module_offset == 0) ? left_force_y_sign_ : right_force_y_sign_;
      fx *= sign_x;
      fy *= sign_y;

      double magnitude = 0.0;
      double normalized = 0.0;
      if (use_fz_only_) {
        magnitude = std::fabs(fz);
        if (max_force_ > 0.0) {
          normalized = std::min(magnitude / max_force_, 1.0);
        }
      } else if (use_axis_normalization_) {
        double z_clamped = std::max(0.0, fz);
        double nx = fx / xy_range;
        double ny = fy / xy_range;
        double nz = z_clamped / z_range;
        magnitude = std::sqrt(fx * fx + fy * fy + fz * fz);
        normalized = std::min(std::sqrt(nx * nx + ny * ny + nz * nz), 1.0);
      } else {
        magnitude = std::sqrt(fx * fx + fy * fy + fz * fz);
        if (max_force_ > 0.0) {
          normalized = std::min(magnitude / max_force_, 1.0);
        }
      }

      if (debug_touch_stats_) {
        if (magnitude > 0.0) {
          used += 1;
          sum_mag += magnitude;
          if (magnitude > max_mag) {
            max_mag = magnitude;
          }
        }
      }

      double radius = circle_min_radius_ + normalized * (circle_max_radius_ - circle_min_radius_);
      const bool zero_value = (radius <= 0.0);
      if (radius < min_marker_scale_) {
        radius = min_marker_scale_;
      }
      double arrow_len = arrow_min_length_ + normalized * (arrow_max_length_ - arrow_min_length_);

      const auto &center = layout.index_centers[idx];

      visualization_msgs::msg::Marker circle;
      circle.header.frame_id = frame_id_;
      circle.header.stamp = stamp;
      circle.ns = "circle_" + module_ns;
      circle.id = module_offset * 1000 + 100 + static_cast<int>(idx);
      circle.type = visualization_msgs::msg::Marker::CYLINDER;
      circle.action = visualization_msgs::msg::Marker::ADD;
      circle.pose.position = center;
      circle.pose.position.z = circle_height_ * 0.5;
      circle.pose.orientation.w = 1.0;
      circle.scale.x = radius * 2.0;
      circle.scale.y = radius * 2.0;
      circle.scale.z = circle_height_;
      circle.color = toColorMsg(lerpColor(color_low_, color_high_, normalized), circle_alpha_);
      if (zero_value) {
        circle.color.a = 0.0;
      }

      out.markers.push_back(std::move(circle));

      visualization_msgs::msg::Marker arrow;
      arrow.header.frame_id = frame_id_;
      arrow.header.stamp = stamp;
      arrow.ns = "arrow_" + module_ns;
      arrow.id = module_offset * 1000 + 200 + static_cast<int>(idx);
      arrow.type = visualization_msgs::msg::Marker::ARROW;
      arrow.action = visualization_msgs::msg::Marker::ADD;
      arrow.scale.x = arrow_shaft_diameter_;
      arrow.scale.y = arrow_head_diameter_;
      arrow.scale.z = arrow_head_length_;
      arrow.color = toColorMsg({0.1, 0.1, 0.1}, arrow_alpha_);

      geometry_msgs::msg::Point start = center;
      start.z = circle_height_ + 0.001;

      geometry_msgs::msg::Point end = start;
      if (arrow_len > 0.0) {
        if (use_xy_direction_) {
          double dir_len = std::sqrt(fx * fx + fy * fy);
          if (dir_len > 1e-9) {
            end.x += (fx / dir_len) * arrow_len;
            end.y += (fy / dir_len) * arrow_len;
          }
        } else {
          double dir = (fz >= 0.0) ? 1.0 : -1.0;
          end.z += dir * arrow_len;
        }
      }

      arrow.points = {start, end};
      out.markers.push_back(std::move(arrow));
    }

    if (debug_touch_stats_) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Grid[%s] use_taxels=%s used=%zu/%zu max=%.6f avg=%.6f",
                           module_ns.c_str(), use_taxels ? "true" : "false", used, grid_size_,
                           max_mag, used > 0 ? (sum_mag / static_cast<double>(used)) : 0.0);
    }
  }

  void appendModuleMarkersUrdf(const xela_taxel_msgs::msg::XTaxelSensorTArray &msg,
                               int module_index,
                               const BaselineState &baseline,
                               int module_offset,
                               const std::string &module_ns,
                               visualization_msgs::msg::MarkerArray &out) {
    if (module_index < 0) {
      return;
    }
    if (static_cast<size_t>(module_index) >= msg.x_modules.size()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Module index %d out of range (size=%zu).", module_index,
                           msg.x_modules.size());
      return;
    }

    const auto &module = msg.x_modules[static_cast<size_t>(module_index)];
    const bool use_taxels = use_taxels_when_no_forces_ && module.forces.empty() &&
                            !module.taxels.empty();
    const double xy_range = use_taxels ? xy_taxel_range_ : xy_force_range_;
    const double z_range = use_taxels ? z_taxel_range_ : z_force_range_;

    const rclcpp::Time stamp = getStamp(msg.header);

    size_t used = 0;
    double max_mag = 0.0;
    double sum_mag = 0.0;

    for (size_t idx = 0; idx < grid_size_; ++idx) {
      std::string frame_id;
      if (idx < module.frame_ids.size()) {
        frame_id = module.frame_ids[idx];
      }
      if (frame_id.empty()) {
        continue;
      }
      if (frame_id.size() > 6 && frame_id.rfind("_joint") == frame_id.size() - 6) {
        frame_id = frame_id.substr(0, frame_id.size() - 6) + "_link";
      }

      double fx = 0.0;
      double fy = 0.0;
      double fz = 0.0;
      if (use_taxels) {
        if (idx < module.taxels.size()) {
          fx = static_cast<double>(module.taxels[idx].x);
          fy = static_cast<double>(module.taxels[idx].y);
          fz = static_cast<double>(module.taxels[idx].z);
        }
      } else if (idx < module.forces.size()) {
        fx = static_cast<double>(module.forces[idx].x);
        fy = static_cast<double>(module.forces[idx].y);
        fz = static_cast<double>(module.forces[idx].z);
      }

      if (baseline_duration_sec_ > 0.0 && !baseline.ready) {
        fx = 0.0;
        fy = 0.0;
        fz = 0.0;
      } else if (use_taxels && baseline.taxel_base.size() == grid_size_) {
        fx -= baseline.taxel_base[idx].x;
        fy -= baseline.taxel_base[idx].y;
        fz -= baseline.taxel_base[idx].z;
      } else if (!use_taxels && baseline.force_base.size() == grid_size_) {
        fx -= baseline.force_base[idx].x;
        fy -= baseline.force_base[idx].y;
        fz -= baseline.force_base[idx].z;
      }

      const double db_xy = use_taxels ? baseline_deadband_taxel_xy_ : baseline_deadband_xy_;
      const double db_z = use_taxels ? baseline_deadband_taxel_z_ : baseline_deadband_z_;
      if (std::fabs(fx) < db_xy) {
        fx = 0.0;
      }
      if (std::fabs(fy) < db_xy) {
        fy = 0.0;
      }
      if (std::fabs(fz) < db_z) {
        fz = 0.0;
      }

      fx *= force_scale_;
      fy *= force_scale_;
      fz *= force_scale_;

      double sign_x = (module_offset == 0) ? urdf_left_force_x_sign_ : urdf_right_force_x_sign_;
      double sign_y = (module_offset == 0) ? urdf_left_force_y_sign_ : urdf_right_force_y_sign_;
      fx *= sign_x;
      fy *= sign_y;

      double magnitude = 0.0;
      double normalized = 0.0;
      if (use_fz_only_) {
        magnitude = std::fabs(fz);
        if (max_force_ > 0.0) {
          normalized = std::min(magnitude / max_force_, 1.0);
        }
      } else if (use_axis_normalization_) {
        double z_clamped = std::max(0.0, fz);
        double nx = fx / xy_range;
        double ny = fy / xy_range;
        double nz = z_clamped / z_range;
        magnitude = std::sqrt(fx * fx + fy * fy + fz * fz);
        normalized = std::min(std::sqrt(nx * nx + ny * ny + nz * nz), 1.0);
      } else {
        magnitude = std::sqrt(fx * fx + fy * fy + fz * fz);
        if (max_force_ > 0.0) {
          normalized = std::min(magnitude / max_force_, 1.0);
        }
      }

      if (debug_touch_stats_) {
        if (magnitude > 0.0) {
          used += 1;
          sum_mag += magnitude;
          if (magnitude > max_mag) {
            max_mag = magnitude;
          }
        }
      }

      double radius = circle_min_radius_ + normalized * (circle_max_radius_ - circle_min_radius_);
      const bool zero_value = (radius <= 0.0);
      if (radius < min_marker_scale_) {
        radius = min_marker_scale_;
      }
      double arrow_len = arrow_min_length_ + normalized * (arrow_max_length_ - arrow_min_length_);

      visualization_msgs::msg::Marker circle;
      circle.header.frame_id = frame_id;
      circle.header.stamp = stamp;
      circle.ns = "circle_" + module_ns;
      circle.id = module_offset * 1000 + 100 + static_cast<int>(idx);
      circle.type = visualization_msgs::msg::Marker::CYLINDER;
      circle.action = visualization_msgs::msg::Marker::ADD;
      circle.pose.orientation.w = 1.0;
      circle.pose.position.x = 0.0;
      circle.pose.position.y = 0.0;
      circle.pose.position.z = circle_height_ * 0.5;
      circle.scale.x = radius * 2.0;
      circle.scale.y = radius * 2.0;
      circle.scale.z = circle_height_;
      circle.color = toColorMsg(lerpColor(color_low_, color_high_, normalized), circle_alpha_);
      if (zero_value) {
        circle.color.a = 0.0;
      }

      out.markers.push_back(std::move(circle));

      visualization_msgs::msg::Marker arrow;
      arrow.header.frame_id = frame_id;
      arrow.header.stamp = stamp;
      arrow.ns = "arrow_" + module_ns;
      arrow.id = module_offset * 1000 + 200 + static_cast<int>(idx);
      arrow.type = visualization_msgs::msg::Marker::ARROW;
      arrow.action = visualization_msgs::msg::Marker::ADD;
      arrow.scale.x = arrow_shaft_diameter_;
      arrow.scale.y = arrow_head_diameter_;
      arrow.scale.z = arrow_head_length_;
      arrow.color = toColorMsg({0.1, 0.1, 0.1}, arrow_alpha_);

      geometry_msgs::msg::Point start;
      geometry_msgs::msg::Point end;
      start.x = 0.0;
      start.y = 0.0;
      start.z = circle_height_ + 0.001;
      end = start;

      if (arrow_len > 0.0) {
        if (use_xy_direction_) {
          double dir_len = std::sqrt(fx * fx + fy * fy);
          if (dir_len > 1e-9) {
            end.x += (fx / dir_len) * arrow_len;
            end.y += (fy / dir_len) * arrow_len;
          }
        } else {
          double dir = (fz >= 0.0) ? 1.0 : -1.0;
          end.z += dir * arrow_len;
        }
      }

      arrow.points = {start, end};
      out.markers.push_back(std::move(arrow));
    }

    if (debug_touch_stats_) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                           "URDF[%s] use_taxels=%s used=%zu/%zu max=%.6f avg=%.6f",
                           module_ns.c_str(), use_taxels ? "true" : "false", used, grid_size_,
                           max_mag, used > 0 ? (sum_mag / static_cast<double>(used)) : 0.0);
    }
  }

  void applyStylePreset(const std::string &preset) {
    if (preset == "default" || preset.empty()) {
      return;
    }

    if (preset == "cool_steel") {
      grid_color_ = {0.26, 0.28, 0.31};
      grid_alpha_ = 0.75;
      grid_line_color_ = {0.65, 0.70, 0.80};
      grid_line_alpha_ = 0.22;
      grid_separator_color_ = {0.20, 0.85, 0.90};
      grid_separator_alpha_ = 0.7;
      color_low_ = {0.05, 0.70, 0.95};
      color_high_ = {1.00, 0.20, 0.65};
      return;
    }

    if (preset == "deep_navy") {
      grid_color_ = {0.12, 0.14, 0.20};
      grid_alpha_ = 0.78;
      grid_line_color_ = {0.45, 0.55, 0.75};
      grid_line_alpha_ = 0.18;
      grid_separator_color_ = {0.15, 0.80, 0.75};
      grid_separator_alpha_ = 0.7;
      color_low_ = {0.45, 0.45, 0.95};
      color_high_ = {0.65, 0.55, 1.0};
      return;
    }

    if (preset == "warm_graphite") {
      grid_color_ = {0.20, 0.18, 0.16};
      grid_alpha_ = 0.78;
      grid_line_color_ = {0.85, 0.78, 0.68};
      grid_line_alpha_ = 0.18;
      grid_separator_color_ = {0.90, 0.75, 0.35};
      grid_separator_alpha_ = 0.7;
      color_low_ = {0.70, 0.45, 0.90};
      color_high_ = {0.85, 0.55, 0.95};
      return;
    }

    RCLCPP_WARN(get_logger(), "Unknown style_preset '%s'; using defaults.", preset.c_str());
  }

  std::string in_topic_;
  std::string out_topic_;
  std::string frame_id_;
  std::string viz_mode_;
  bool overlay_grid_in_urdf_ = true;
  double urdf_left_force_x_sign_ = 1.0;
  double urdf_left_force_y_sign_ = 1.0;
  double urdf_right_force_x_sign_ = 1.0;
  double urdf_right_force_y_sign_ = 1.0;

  int grid_rows_ = 4;
  int grid_cols_ = 6;
  size_t grid_cells_ = 24;
  size_t grid_size_ = 24;
  std::vector<int64_t> grid_index_map_;
  std::vector<int64_t> grid_index_map_left_;
  std::vector<int64_t> grid_index_map_right_;
  std::vector<int64_t> grid_separator_cols_left_;
  std::vector<int64_t> grid_separator_cols_right_;
  double cell_size_ = 0.015;
  double module_gap_ = 0.04;
  double origin_x_ = 0.0;
  double origin_y_ = 0.0;

  int left_module_index_ = 0;
  int right_module_index_ = 1;
  bool row_flip_right_ = true;
  bool col_flip_right_ = true;

  double force_scale_ = 1.0;
  double max_force_ = 0.02;
  bool use_fz_only_ = false;
  bool use_xy_direction_ = true;
  double baseline_duration_sec_ = 2.0;
  double baseline_deadband_xy_ = 0.0;
  double baseline_deadband_z_ = 0.0;
  double baseline_deadband_taxel_xy_ = 0.0;
  double baseline_deadband_taxel_z_ = 0.0;
  bool use_axis_normalization_ = true;
  double xy_force_range_ = 0.8;
  double z_force_range_ = 14.0;
  bool use_taxels_when_no_forces_ = true;
  double xy_taxel_range_ = 1350.0;
  double z_taxel_range_ = 10085.0;
  double urdf_taxel_scale_ = 0.3;

  double circle_min_radius_ = 0.002;
  double circle_max_radius_ = 0.008;
  double circle_height_ = 0.002;
  bool use_cell_scale_ = true;
  double circle_area_scale_ = 2.0;

  double arrow_min_length_ = 0.004;
  double arrow_max_length_ = 0.016;
  double arrow_shaft_diameter_ = 0.001;
  double arrow_head_diameter_ = 0.002;
  double arrow_head_length_ = 0.003;

  std::string marker_stamp_mode_ = "keep";
  double min_marker_scale_ = 1e-4;
  bool debug_touch_stats_ = false;
  double marker_time_offset_sec_ = 0.0;
  double arrow_length_scale_ = 2.0;

  double grid_thickness_ = 0.0008;
  bool grid_lines_enabled_ = true;
  double grid_line_width_ = 0.001;
  double grid_line_alpha_ = 0.2;
  double grid_separator_width_ = 0.002;
  double grid_separator_alpha_ = 0.6;

  double grid_alpha_ = 1.0;
  double circle_alpha_ = 0.9;
  double arrow_alpha_ = 1.0;

  ColorRGB grid_color_{0.91, 0.89, 0.86};
  ColorRGB grid_line_color_{0.2, 0.2, 0.2};
  ColorRGB grid_separator_color_{0.1, 0.1, 0.1};
  ColorRGB color_low_{0.2, 0.4, 1.0};
  ColorRGB color_high_{1.0, 0.2, 0.2};

  double left_force_x_sign_ = 1.0;
  double left_force_y_sign_ = -1.0;
  double right_force_x_sign_ = -1.0;
  double right_force_y_sign_ = -1.0;

  std::vector<visualization_msgs::msg::Marker> grid_markers_;
  std::vector<ModuleLayout> layouts_;
  std::array<BaselineState, 2> baselines_{};

  rclcpp::Subscription<xela_taxel_msgs::msg::XTaxelSensorTArray>::SharedPtr sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<XelaTaxelViz2F>());
  rclcpp::shutdown();
  return 0;
}
