#include <algorithm>
#include <cmath>
#include <fstream>
#include <regex>
#include <string>
#include <unordered_map>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <xela_taxel_msgs/msg/x_taxel_sensor_t_array.hpp>
#include <yaml-cpp/yaml.h>

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

// Resolves a "package://<pkg>/<rel>" style path used elsewhere in this codebase
// (see std_xela_joint_state_publisher.cpp) so grid_config_yaml can be given
// either as a plain filesystem path or a package:// URI.
std::string resolvePackagePath(const std::string &input) {
  const std::string prefix = "package://";
  if (input.rfind(prefix, 0) != 0) {
    return input;
  }
  std::string rest = input.substr(prefix.size());
  auto slash = rest.find('/');
  if (slash == std::string::npos) {
    return input;
  }
  const std::string pkg_name = rest.substr(0, slash);
  const std::string rel_path = rest.substr(slash + 1);
  try {
    auto share_dir = ament_index_cpp::get_package_share_directory(pkg_name);
    return share_dir + "/" + rel_path;
  } catch (const std::exception &) {
    return input;
  }
}

}  // namespace

// A named, reusable cell layout for one sensor housing shape, keyed by the
// taxel's own dot number (the trailing _NN in its joint name, e.g. "09" in
// "x_taxel_1_f3_dg5f_ft_09_joint") rather than by its position in the
// incoming message's taxels[]/frame_ids[] array. Message array order is
// vendor/firmware-defined and can change if xela_server2_dg5f's
// r_server_model_joint_map.yaml is ever regenerated in a different order;
// keying by dot number instead means this layout keeps working unchanged
// as long as the joint *naming* convention (x_taxel_<hand>_<module>_<NN>_joint)
// stays the same, regardless of what order taxels happen to arrive in.
struct LayoutTemplate {
  int rows = 0;
  int cols = 0;
  std::unordered_map<int, std::pair<int, int>> dot_to_rc;  // dot number -> (row, col)
};

// Each ModuleBlock describes one of DG-5F's 16 named sensor housings
// (5x dg5f_ft fingertip, 5x mid uSPa22, 5x prox uSPa22, 1x palm uSPa46) as a
// rectangular block of cells on one shared canvas. If `layout` names a
// LayoutTemplate (see above), each taxel's cell is resolved by its dot
// number via that template; otherwise (mid/prox/palm today -- their
// per-dot physical layout is not yet confirmed) this falls back to placing
// local slot 0..taxel_count-1 row-major into the block, where slot order
// follows the incoming message's per-module taxels/frame_ids array order
// (vendor-arbitrary per xela_server2_dg5f/config/r_server_model_joint_map.yaml,
// not necessarily physically adjacent -- see grid.yaml's own header comment).
struct ModuleBlock {
  std::string name;
  int taxel_count = 0;
  int grid_rows = 0;
  int grid_cols = 0;
  int offset_row = 0;
  int offset_col = 0;
  size_t flat_offset = 0;
  const LayoutTemplate *layout = nullptr;
};

class StdXelaTaxelVizDg5f : public rclcpp::Node {
public:
  StdXelaTaxelVizDg5f() : Node("std_xela_taxel_viz_dg5f") {
    declare_parameter<std::string>("in_topic", "/x_taxel_dg5f");
    declare_parameter<std::string>("out_topic", "markers");
    declare_parameter<std::string>("frame_id", "world");
    declare_parameter<std::string>("frame_prefix", "");
    declare_parameter<std::string>("viz_mode", "grid");
    declare_parameter<bool>("overlay_grid_in_urdf", false);
    declare_parameter<double>("urdf_force_x_sign", 1.0);
    declare_parameter<double>("urdf_force_y_sign", 1.0);

    declare_parameter<std::string>("grid_config_yaml", "");
    declare_parameter<int>("canvas_rows", 14);
    declare_parameter<int>("canvas_cols", 24);
    declare_parameter<double>("cell_size", 0.006);
    declare_parameter<double>("origin_x", 0.0);
    declare_parameter<double>("origin_y", 0.0);

    declare_parameter<double>("force_scale", 1.0);
    declare_parameter<double>("max_force", 0.02);
    declare_parameter<bool>("use_fz_only", false);
    declare_parameter<bool>("use_xy_direction", true);
    declare_parameter<double>("baseline_duration_sec", 3.0);
    declare_parameter<std::string>("marker_stamp_mode", "now");
    declare_parameter<double>("marker_time_offset_sec", 0.0);
    declare_parameter<double>("baseline_deadband_xy", 0.0);
    declare_parameter<double>("baseline_deadband_z", 0.0);
    declare_parameter<double>("baseline_deadband_taxel_xy", 0.0);
    declare_parameter<double>("baseline_deadband_taxel_z", 0.0);
    declare_parameter<bool>("use_axis_normalization", true);
    declare_parameter<double>("xy_force_range", 0.8);
    declare_parameter<double>("z_force_range", 14.0);
    declare_parameter<bool>("use_taxels_when_no_forces", true);
    // DG-5F has no published sensor spec yet (new product); default to the AH
    // uSCu (curved fingertip) sensor's own measured "H" sensitivity range from
    // xela_taxel_docs/uSCu_Specsheet_Oct_2024.pdf (Table 1: x/y=+-1820 LSB,
    // z=+18520 LSB) as a stand-in, per user direction (2026-08-23) -- replace
    // once DG-5F's own spec sheet is published.
    declare_parameter<double>("xy_taxel_range", 1820.0);
    declare_parameter<double>("z_taxel_range", 18520.0);
    declare_parameter<double>("urdf_taxel_scale", 0.3);

    declare_parameter<double>("circle_min_radius", 0.001);
    declare_parameter<double>("circle_max_radius", 0.003);
    declare_parameter<double>("circle_height", 0.002);
    declare_parameter<bool>("use_cell_scale", true);
    declare_parameter<double>("circle_area_scale", 2.0);

    declare_parameter<double>("arrow_min_length", 0.002);
    declare_parameter<double>("arrow_max_length", 0.006);
    declare_parameter<double>("arrow_shaft_diameter", 0.0006);
    declare_parameter<double>("arrow_head_diameter", 0.0012);
    declare_parameter<double>("arrow_head_length", 0.0018);
    declare_parameter<double>("arrow_length_scale", 2.0);
    declare_parameter<double>("min_marker_scale", 1e-4);

    declare_parameter<double>("grid_thickness", 0.0006);
    declare_parameter<bool>("grid_lines_enabled", true);
    declare_parameter<double>("grid_line_width", 0.0004);
    declare_parameter<double>("grid_line_alpha", 0.2);
    declare_parameter<std::vector<double>>("grid_line_color", {0.2, 0.2, 0.2});

    declare_parameter<double>("grid_alpha", 1.0);
    declare_parameter<double>("circle_alpha", 0.9);
    declare_parameter<double>("arrow_alpha", 1.0);
    declare_parameter<bool>("debug_touch_stats", false);

    declare_parameter<std::vector<double>>("grid_color", {0.91, 0.89, 0.86});
    declare_parameter<std::string>("style_preset", "default");
    declare_parameter<std::vector<double>>("color_low", {0.2, 0.4, 1.0});
    declare_parameter<std::vector<double>>("color_high", {1.0, 0.2, 0.2});
    declare_parameter<double>("force_x_sign", 1.0);
    declare_parameter<double>("force_y_sign", -1.0);

    in_topic_ = get_parameter("in_topic").as_string();
    out_topic_ = get_parameter("out_topic").as_string();
    frame_id_ = get_parameter("frame_id").as_string();
    frame_prefix_ = get_parameter("frame_prefix").as_string();
    if (!frame_prefix_.empty() && frame_id_.rfind(frame_prefix_, 0) != 0) {
      frame_id_ = frame_prefix_ + frame_id_;
    }
    viz_mode_ = get_parameter("viz_mode").as_string();
    overlay_grid_in_urdf_ = get_parameter("overlay_grid_in_urdf").as_bool();
    urdf_force_x_sign_ = get_parameter("urdf_force_x_sign").as_double();
    urdf_force_y_sign_ = get_parameter("urdf_force_y_sign").as_double();

    grid_config_yaml_ = get_parameter("grid_config_yaml").as_string();
    canvas_rows_ = get_parameter("canvas_rows").as_int();
    canvas_cols_ = get_parameter("canvas_cols").as_int();
    cell_size_ = get_parameter("cell_size").as_double();
    origin_x_ = get_parameter("origin_x").as_double();
    origin_y_ = get_parameter("origin_y").as_double();

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

    grid_alpha_ = get_parameter("grid_alpha").as_double();
    circle_alpha_ = get_parameter("circle_alpha").as_double();
    arrow_alpha_ = get_parameter("arrow_alpha").as_double();
    debug_touch_stats_ = get_parameter("debug_touch_stats").as_bool();

    auto grid_color_vec = getVecParam(*this, "grid_color", {0.91, 0.89, 0.86}, get_logger());
    auto grid_line_color_vec = getVecParam(*this, "grid_line_color", {0.2, 0.2, 0.2}, get_logger());
    auto color_low_vec = getVecParam(*this, "color_low", {0.2, 0.4, 1.0}, get_logger());
    auto color_high_vec = getVecParam(*this, "color_high", {1.0, 0.2, 0.2}, get_logger());
    grid_color_ = {grid_color_vec[0], grid_color_vec[1], grid_color_vec[2]};
    grid_line_color_ = {grid_line_color_vec[0], grid_line_color_vec[1], grid_line_color_vec[2]};
    color_low_ = {color_low_vec[0], color_low_vec[1], color_low_vec[2]};
    color_high_ = {color_high_vec[0], color_high_vec[1], color_high_vec[2]};

    force_x_sign_ = get_parameter("force_x_sign").as_double();
    force_y_sign_ = get_parameter("force_y_sign").as_double();

    applyStylePreset(get_parameter("style_preset").as_string());

    if (use_cell_scale_) {
      double cell_area = cell_size_ * cell_size_;
      double area_scale = std::max(0.0, circle_area_scale_);
      circle_min_radius_ = 0.0;
      circle_max_radius_ = std::sqrt((cell_area * area_scale) / M_PI);
      arrow_min_length_ = 0.0;
      arrow_max_length_ = std::max(0.0, cell_size_ * arrow_length_scale_);
    }
    if (min_marker_scale_ <= 0.0) {
      min_marker_scale_ = 1e-6;
    }
    if (xy_force_range_ <= 0.0) {
      xy_force_range_ = 0.8;
    }
    if (z_force_range_ <= 0.0) {
      z_force_range_ = 14.0;
    }
    if (xy_taxel_range_ <= 0.0) {
      xy_taxel_range_ = 1820.0;
    }
    if (z_taxel_range_ <= 0.0) {
      z_taxel_range_ = 18520.0;
    }
    if (viz_mode_ != "grid" && viz_mode_ != "urdf") {
      RCLCPP_WARN(get_logger(), "Unknown viz_mode '%s'; using 'grid'.", viz_mode_.c_str());
      viz_mode_ = "grid";
    }

    loadModules();
    buildGridMarkers();
    ensureBaselineSize();

    rclcpp::QoS qos(10);
    qos.transient_local();

    pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(out_topic_, qos);
    sub_ = create_subscription<xela_taxel_msgs::msg::XTaxelSensorTArray>(
      in_topic_, 10, std::bind(&StdXelaTaxelVizDg5f::onArray, this, std::placeholders::_1));
    set_mode_service_ = create_service<std_srvs::srv::SetBool>(
      "std_xela_taxel_viz_dg5f/set_mode",
      std::bind(&StdXelaTaxelVizDg5f::onSetMode, this,
                std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(get_logger(),
                "std_xela_taxel_viz_dg5f started. in: %s out: %s modules: %zu taxels: %zu",
                in_topic_.c_str(), out_topic_.c_str(), modules_.size(), total_taxels_);
  }

private:
  std::string applyFramePrefix(const std::string &frame_id) const {
    if (frame_prefix_.empty()) {
      return frame_id;
    }
    if (frame_id.rfind(frame_prefix_, 0) == 0) {
      return frame_id;
    }
    return frame_prefix_ + frame_id;
  }

  // Extracts the module key from a taxel joint name, e.g.
  // "x_taxel_1_f3_mid_uSPa22_03_joint" -> "f3_mid_uSPa22". Matching x_modules
  // to config modules by parsing frame_ids[0] this way (rather than assuming
  // a fixed x_modules array order) is robust to whatever order the DG-5F
  // driver publishes its 16 sensor units in.
  // dot_number receives the joint name's trailing _NN as an int (e.g. 9 for
  // "..._09_joint"); pass nullptr if the caller doesn't need it.
  static bool parseModuleKey(const std::string &frame_id, std::string &module_key,
                             int *dot_number = nullptr) {
    static const std::regex pattern(R"(^x_taxel_\d+_(.+)_(\d{1,3})_joint$)");
    std::smatch match;
    if (!std::regex_match(frame_id, match, pattern)) {
      return false;
    }
    module_key = match[1].str();
    if (dot_number != nullptr) {
      *dot_number = std::stoi(match[2].str());
    }
    return true;
  }

  void loadModules() {
    modules_.clear();
    total_taxels_ = 0;

    std::string resolved = resolvePackagePath(grid_config_yaml_);
    if (resolved.empty() || !std::ifstream(resolved).good()) {
      RCLCPP_ERROR(get_logger(),
                   "grid_config_yaml '%s' not found; no modules loaded, viz will be empty.",
                   grid_config_yaml_.c_str());
      return;
    }

    YAML::Node root = YAML::LoadFile(resolved);

    layout_templates_.clear();
    YAML::Node templates_node = root["layout_templates"];
    if (templates_node && templates_node.IsMap()) {
      for (const auto &tpl_entry : templates_node) {
        const std::string tpl_name = tpl_entry.first.as<std::string>();
        LayoutTemplate tpl;
        tpl.rows = tpl_entry.second["rows"].as<int>();
        tpl.cols = tpl_entry.second["cols"].as<int>();
        YAML::Node dot_positions = tpl_entry.second["dot_positions"];
        if (dot_positions && dot_positions.IsMap()) {
          for (const auto &dot_entry : dot_positions) {
            const int dot = std::stoi(dot_entry.first.as<std::string>());
            const auto rc = dot_entry.second.as<std::vector<int>>();
            if (rc.size() == 2) {
              tpl.dot_to_rc[dot] = {rc[0], rc[1]};
            }
          }
        }
        layout_templates_.emplace(tpl_name, std::move(tpl));
      }
    }

    YAML::Node modules_node = root["modules"];
    if (!modules_node || !modules_node.IsSequence()) {
      RCLCPP_ERROR(get_logger(), "'modules' list missing/invalid in %s", resolved.c_str());
      return;
    }

    for (const auto &entry : modules_node) {
      ModuleBlock block;
      block.name = entry["name"].as<std::string>();
      block.taxel_count = entry["taxel_count"].as<int>();
      block.grid_rows = entry["grid_rows"].as<int>();
      block.grid_cols = entry["grid_cols"].as<int>();
      block.offset_row = entry["offset_row"].as<int>();
      block.offset_col = entry["offset_col"].as<int>();
      if (entry["layout"]) {
        const std::string layout_name = entry["layout"].as<std::string>();
        auto tpl_it = layout_templates_.find(layout_name);
        if (tpl_it == layout_templates_.end()) {
          RCLCPP_WARN(get_logger(), "Module '%s' references unknown layout '%s'; using fallback order.",
                      block.name.c_str(), layout_name.c_str());
        } else {
          block.layout = &tpl_it->second;
        }
      }
      if (block.taxel_count <= 0 || block.grid_rows <= 0 || block.grid_cols <= 0) {
        RCLCPP_WARN(get_logger(), "Module '%s' has invalid size; skipping.", block.name.c_str());
        continue;
      }
      if (block.grid_rows * block.grid_cols < block.taxel_count) {
        RCLCPP_WARN(get_logger(),
                    "Module '%s' block (%dx%d) smaller than taxel_count %d; extra taxels dropped.",
                    block.name.c_str(), block.grid_rows, block.grid_cols, block.taxel_count);
      }
      block.flat_offset = total_taxels_;
      total_taxels_ += static_cast<size_t>(block.taxel_count);
      module_index_by_name_[block.name] = modules_.size();
      modules_.push_back(block);
    }
  }

  void ensureBaselineSize() {
    baseline_.force_sum.assign(total_taxels_, {});
    baseline_.taxel_sum.assign(total_taxels_, {});
    baseline_.force_base.assign(total_taxels_, {});
    baseline_.taxel_base.assign(total_taxels_, {});
  }

  geometry_msgs::msg::Point cellCenter(int row, int col) const {
    geometry_msgs::msg::Point p;
    p.x = origin_x_ + (static_cast<double>(col) + 0.5) * cell_size_;
    p.y = origin_y_ - (static_cast<double>(row) + 0.5) * cell_size_;
    p.z = 0.0;
    return p;
  }

  void buildGridMarkers() {
    grid_markers_.clear();
    taxel_points_.assign(total_taxels_, geometry_msgs::msg::Point());

    int marker_id = 0;
    for (const auto &block : modules_) {
      // Slot 0..taxel_count-1 here is keyed by dot number (slot = dot-1) when
      // the module has a layout template, so it stays correct regardless of
      // message array order; see resolveTaxels() for the matching lookup on
      // the incoming-data side.
      for (int idx = 0; idx < block.taxel_count; ++idx) {
        int row = block.offset_row + idx / block.grid_cols;
        int col = block.offset_col + idx % block.grid_cols;
        if (block.layout != nullptr) {
          const int dot = idx + 1;
          auto rc_it = block.layout->dot_to_rc.find(dot);
          if (rc_it != block.layout->dot_to_rc.end()) {
            row = block.offset_row + rc_it->second.first;
            col = block.offset_col + rc_it->second.second;
          }
        }
        taxel_points_[block.flat_offset + static_cast<size_t>(idx)] = cellCenter(row, col);

        visualization_msgs::msg::Marker m;
        m.ns = "grid_" + block.name;
        m.id = marker_id++;
        m.type = visualization_msgs::msg::Marker::CUBE;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.pose.position = cellCenter(row, col);
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
        lines.ns = "grid_lines_" + block.name;
        lines.id = marker_id++;
        lines.type = visualization_msgs::msg::Marker::LINE_LIST;
        lines.action = visualization_msgs::msg::Marker::ADD;
        lines.pose.orientation.w = 1.0;
        lines.scale.x = grid_line_width_;
        lines.color = toColorMsg(grid_line_color_, grid_line_alpha_);

        double x0 = origin_x_ + block.offset_col * cell_size_;
        double y0 = origin_y_ - block.offset_row * cell_size_;
        double x1 = x0 + block.grid_cols * cell_size_;
        double y1 = y0 - block.grid_rows * cell_size_;

        for (int r = 0; r <= block.grid_rows; ++r) {
          double y = y0 - static_cast<double>(r) * cell_size_;
          geometry_msgs::msg::Point p0, p1;
          p0.x = x0; p0.y = y; p0.z = 0.0;
          p1.x = x1; p1.y = y; p1.z = 0.0;
          lines.points.push_back(p0);
          lines.points.push_back(p1);
        }
        for (int c = 0; c <= block.grid_cols; ++c) {
          double x = x0 + static_cast<double>(c) * cell_size_;
          geometry_msgs::msg::Point p0, p1;
          p0.x = x; p0.y = y0; p0.z = 0.0;
          p1.x = x; p1.y = y1; p1.z = 0.0;
          lines.points.push_back(p0);
          lines.points.push_back(p1);
        }
        grid_markers_.push_back(std::move(lines));
      }
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

  // Per-taxel data resolved for one incoming XTaxelSensorT module, aligned to
  // this node's flat 0..total_taxels_-1 index space.
  struct ResolvedTaxel {
    size_t flat_index;
    std::string frame_id;
    Vec3 raw;
    bool use_taxels;
  };

  std::vector<ResolvedTaxel> resolveTaxels(const xela_taxel_msgs::msg::XTaxelSensorTArray &msg) {
    std::vector<ResolvedTaxel> out;
    out.reserve(total_taxels_);

    for (const auto &module : msg.x_modules) {
      if (module.frame_ids.empty()) {
        continue;
      }
      std::string module_key;
      if (!parseModuleKey(module.frame_ids[0], module_key)) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                             "Could not parse module key from frame_id '%s'; skipping module.",
                             module.frame_ids[0].c_str());
        continue;
      }
      auto it = module_index_by_name_.find(module_key);
      if (it == module_index_by_name_.end()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                             "Module '%s' not present in grid config; skipping.",
                             module_key.c_str());
        continue;
      }
      const auto &block = modules_[it->second];
      const bool use_taxels = use_taxels_when_no_forces_ && module.forces.empty() &&
                              !module.taxels.empty();

      const size_t local_count = std::min(static_cast<size_t>(block.taxel_count),
                                          std::max(module.taxels.size(), module.forces.size()));
      for (size_t idx = 0; idx < local_count; ++idx) {
        ResolvedTaxel rt;
        rt.frame_id = (idx < module.frame_ids.size()) ? module.frame_ids[idx] : "";

        // With a layout template, resolve this taxel's slot from its own dot
        // number (parsed from frame_id) rather than its position idx in the
        // message array, so re-ordering upstream (e.g. a regenerated
        // r_server_model_joint_map.yaml) can't silently misplace it.
        size_t slot = idx;
        if (block.layout != nullptr && !rt.frame_id.empty()) {
          std::string parsed_module_key;
          int dot_number = 0;
          if (parseModuleKey(rt.frame_id, parsed_module_key, &dot_number) &&
              dot_number >= 1 && static_cast<size_t>(dot_number) <= static_cast<size_t>(block.taxel_count)) {
            slot = static_cast<size_t>(dot_number - 1);
          } else {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                 "Module '%s': could not resolve dot number from frame_id '%s'; "
                                 "falling back to array order for this taxel.",
                                 block.name.c_str(), rt.frame_id.c_str());
          }
        }
        rt.flat_index = block.flat_offset + slot;
        rt.use_taxels = use_taxels;
        if (use_taxels) {
          if (idx < module.taxels.size()) {
            rt.raw = {static_cast<double>(module.taxels[idx].x),
                      static_cast<double>(module.taxels[idx].y),
                      static_cast<double>(module.taxels[idx].z)};
          }
        } else if (idx < module.forces.size()) {
          rt.raw = {static_cast<double>(module.forces[idx].x),
                    static_cast<double>(module.forces[idx].y),
                    static_cast<double>(module.forces[idx].z)};
        }
        out.push_back(rt);
      }
    }
    return out;
  }

  void updateBaseline(const std::vector<ResolvedTaxel> &taxels, const rclcpp::Time &stamp) {
    if (baseline_duration_sec_ <= 0.0) {
      baseline_.ready = true;
      return;
    }
    if (!baseline_.started) {
      baseline_.start_time = stamp;
      baseline_.started = true;
    }
    if (baseline_.ready) {
      return;
    }

    double elapsed = (stamp - baseline_.start_time).seconds();
    if (elapsed < 0.0) {
      baseline_.start_time = stamp;
      elapsed = 0.0;
    }

    if (elapsed <= baseline_duration_sec_) {
      for (const auto &t : taxels) {
        if (t.flat_index >= total_taxels_) {
          continue;
        }
        if (t.use_taxels) {
          baseline_.taxel_sum[t.flat_index].x += t.raw.x;
          baseline_.taxel_sum[t.flat_index].y += t.raw.y;
          baseline_.taxel_sum[t.flat_index].z += t.raw.z;
        } else {
          baseline_.force_sum[t.flat_index].x += t.raw.x;
          baseline_.force_sum[t.flat_index].y += t.raw.y;
          baseline_.force_sum[t.flat_index].z += t.raw.z;
        }
      }
      baseline_.samples += 1;
    }

    if (elapsed >= baseline_duration_sec_) {
      if (baseline_.samples > 0) {
        for (size_t idx = 0; idx < total_taxels_; ++idx) {
          baseline_.force_base[idx].x = baseline_.force_sum[idx].x / baseline_.samples;
          baseline_.force_base[idx].y = baseline_.force_sum[idx].y / baseline_.samples;
          baseline_.force_base[idx].z = baseline_.force_sum[idx].z / baseline_.samples;
          baseline_.taxel_base[idx].x = baseline_.taxel_sum[idx].x / baseline_.samples;
          baseline_.taxel_base[idx].y = baseline_.taxel_sum[idx].y / baseline_.samples;
          baseline_.taxel_base[idx].z = baseline_.taxel_sum[idx].z / baseline_.samples;
        }
      }
      baseline_.ready = true;
      RCLCPP_INFO(get_logger(), "Baseline computed with %zu samples.", baseline_.samples);
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

    auto taxels = resolveTaxels(*msg);
    updateBaseline(taxels, stamp);

    if (viz_mode_ == "grid") {
      appendMarkersGrid(taxels, stamp, out);
    } else {
      appendMarkersUrdf(taxels, stamp, out);
    }

    pub_->publish(out);
  }

  void publishDeleteAll(const rclcpp::Time &stamp) {
    visualization_msgs::msg::MarkerArray out;
    visualization_msgs::msg::Marker clear;
    clear.header.frame_id = frame_id_;
    clear.header.stamp = stamp;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    out.markers.push_back(clear);
    pub_->publish(out);
  }

  void onSetMode(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
    const std::string target_mode = request->data ? "urdf" : "grid";
    if (target_mode == viz_mode_) {
      response->success = true;
      response->message = "Already in " + target_mode + " mode";
      return;
    }
    publishDeleteAll(now());
    viz_mode_ = target_mode;
    response->success = true;
    response->message = "Switched to " + target_mode;
    RCLCPP_INFO(get_logger(), "viz_mode switched via service: %s", viz_mode_.c_str());
  }

  // Shared per-taxel normalization/coloring math, used by both grid and urdf
  // rendering below. Returns {normalized (0..1), fx, fy, fz (post scale/sign)}.
  struct Normalized {
    double normalized = 0.0;
    double fx = 0.0;
    double fy = 0.0;
    double fz = 0.0;
  };

  Normalized normalize(const ResolvedTaxel &t, double sign_x, double sign_y) {
    Normalized n;
    double fx = t.raw.x;
    double fy = t.raw.y;
    double fz = t.raw.z;

    if (baseline_duration_sec_ > 0.0 && !baseline_.ready) {
      fx = fy = fz = 0.0;
    } else if (t.use_taxels && baseline_.taxel_base.size() == total_taxels_) {
      fx -= baseline_.taxel_base[t.flat_index].x;
      fy -= baseline_.taxel_base[t.flat_index].y;
      fz -= baseline_.taxel_base[t.flat_index].z;
    } else if (!t.use_taxels && baseline_.force_base.size() == total_taxels_) {
      fx -= baseline_.force_base[t.flat_index].x;
      fy -= baseline_.force_base[t.flat_index].y;
      fz -= baseline_.force_base[t.flat_index].z;
    }

    const double db_xy = t.use_taxels ? baseline_deadband_taxel_xy_ : baseline_deadband_xy_;
    const double db_z = t.use_taxels ? baseline_deadband_taxel_z_ : baseline_deadband_z_;
    if (std::fabs(fx) < db_xy) fx = 0.0;
    if (std::fabs(fy) < db_xy) fy = 0.0;
    if (std::fabs(fz) < db_z) fz = 0.0;

    fx *= force_scale_ * sign_x;
    fy *= force_scale_ * sign_y;
    fz *= force_scale_;

    const double xy_range = t.use_taxels ? xy_taxel_range_ : xy_force_range_;
    const double z_range = t.use_taxels ? z_taxel_range_ : z_force_range_;

    if (use_fz_only_) {
      double magnitude = std::fabs(fz);
      n.normalized = (max_force_ > 0.0) ? std::min(magnitude / max_force_, 1.0) : 0.0;
    } else if (use_axis_normalization_) {
      double z_clamped = std::max(0.0, fz);
      double nx = fx / xy_range;
      double ny = fy / xy_range;
      double nz = z_clamped / z_range;
      n.normalized = std::min(std::sqrt(nx * nx + ny * ny + nz * nz), 1.0);
    } else {
      double magnitude = std::sqrt(fx * fx + fy * fy + fz * fz);
      n.normalized = (max_force_ > 0.0) ? std::min(magnitude / max_force_, 1.0) : 0.0;
    }

    n.fx = fx;
    n.fy = fy;
    n.fz = fz;
    return n;
  }

  void appendMarkersGrid(const std::vector<ResolvedTaxel> &taxels, const rclcpp::Time &stamp,
                         visualization_msgs::msg::MarkerArray &out) {
    for (const auto &t : taxels) {
      if (t.flat_index >= taxel_points_.size()) {
        continue;
      }
      Normalized n = normalize(t, force_x_sign_, force_y_sign_);

      double radius = circle_min_radius_ + n.normalized * (circle_max_radius_ - circle_min_radius_);
      const bool zero_value = (radius <= 0.0);
      radius = std::max(radius, min_marker_scale_);
      double arrow_len = arrow_min_length_ + n.normalized * (arrow_max_length_ - arrow_min_length_);

      const auto &center = taxel_points_[t.flat_index];

      visualization_msgs::msg::Marker circle;
      circle.header.frame_id = frame_id_;
      circle.header.stamp = stamp;
      circle.ns = "circle";
      circle.id = 100000 + static_cast<int>(t.flat_index);
      circle.type = visualization_msgs::msg::Marker::CYLINDER;
      circle.action = visualization_msgs::msg::Marker::ADD;
      circle.pose.position = center;
      circle.pose.position.z = circle_height_ * 0.5;
      circle.pose.orientation.w = 1.0;
      circle.scale.x = radius * 2.0;
      circle.scale.y = radius * 2.0;
      circle.scale.z = circle_height_;
      circle.color = toColorMsg(lerpColor(color_low_, color_high_, n.normalized), circle_alpha_);
      if (zero_value) {
        circle.color.a = 0.0;
      }
      out.markers.push_back(std::move(circle));

      visualization_msgs::msg::Marker arrow;
      arrow.header.frame_id = frame_id_;
      arrow.header.stamp = stamp;
      arrow.ns = "arrow";
      arrow.id = 200000 + static_cast<int>(t.flat_index);
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
          double dir_len = std::sqrt(n.fx * n.fx + n.fy * n.fy);
          if (dir_len > 1e-9) {
            end.x += (n.fx / dir_len) * arrow_len;
            end.y += (n.fy / dir_len) * arrow_len;
          }
        } else {
          double dir = (n.fz >= 0.0) ? 1.0 : -1.0;
          end.z += dir * arrow_len;
        }
      }
      arrow.points = {start, end};
      out.markers.push_back(std::move(arrow));
    }
  }

  void appendMarkersUrdf(const std::vector<ResolvedTaxel> &taxels, const rclcpp::Time &stamp,
                         visualization_msgs::msg::MarkerArray &out) {
    for (const auto &t : taxels) {
      if (t.frame_id.empty()) {
        continue;
      }
      std::string frame_id = t.frame_id;
      if (frame_id.size() > 6 && frame_id.rfind("_joint") == frame_id.size() - 6) {
        frame_id = frame_id.substr(0, frame_id.size() - 6) + "_link";
      }
      frame_id = applyFramePrefix(frame_id);

      Normalized n = normalize(t, urdf_force_x_sign_, urdf_force_y_sign_);

      double radius = circle_min_radius_ + n.normalized * (circle_max_radius_ - circle_min_radius_);
      const bool zero_value = (radius <= 0.0);
      radius = std::max(radius, min_marker_scale_);
      double arrow_len = arrow_min_length_ + n.normalized * (arrow_max_length_ - arrow_min_length_);

      visualization_msgs::msg::Marker circle;
      circle.header.frame_id = frame_id;
      circle.header.stamp = stamp;
      circle.ns = "circle";
      circle.id = 100000 + static_cast<int>(t.flat_index);
      circle.type = visualization_msgs::msg::Marker::CYLINDER;
      circle.action = visualization_msgs::msg::Marker::ADD;
      circle.pose.orientation.w = 1.0;
      circle.pose.position.z = circle_height_ * 0.5;
      circle.scale.x = radius * 2.0;
      circle.scale.y = radius * 2.0;
      circle.scale.z = circle_height_;
      circle.color = toColorMsg(lerpColor(color_low_, color_high_, n.normalized), circle_alpha_);
      if (zero_value) {
        circle.color.a = 0.0;
      }
      out.markers.push_back(std::move(circle));

      visualization_msgs::msg::Marker arrow;
      arrow.header.frame_id = frame_id;
      arrow.header.stamp = stamp;
      arrow.ns = "arrow";
      arrow.id = 200000 + static_cast<int>(t.flat_index);
      arrow.type = visualization_msgs::msg::Marker::ARROW;
      arrow.action = visualization_msgs::msg::Marker::ADD;
      arrow.scale.x = arrow_shaft_diameter_;
      arrow.scale.y = arrow_head_diameter_;
      arrow.scale.z = arrow_head_length_;
      arrow.color = toColorMsg({0.1, 0.1, 0.1}, arrow_alpha_);

      geometry_msgs::msg::Point start;
      start.z = circle_height_ + 0.001;
      geometry_msgs::msg::Point end = start;
      if (arrow_len > 0.0) {
        if (use_xy_direction_) {
          double dir_len = std::sqrt(n.fx * n.fx + n.fy * n.fy);
          if (dir_len > 1e-9) {
            end.x += (n.fx / dir_len) * arrow_len;
            end.y += (n.fy / dir_len) * arrow_len;
          }
        } else {
          double dir = (n.fz >= 0.0) ? 1.0 : -1.0;
          end.z += dir * arrow_len;
        }
      }
      arrow.points = {start, end};
      out.markers.push_back(std::move(arrow));
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
      color_low_ = {0.05, 0.70, 0.95};
      color_high_ = {1.00, 0.20, 0.65};
      return;
    }
    RCLCPP_WARN(get_logger(), "Unknown style_preset '%s'; using defaults.", preset.c_str());
  }

  std::string in_topic_;
  std::string out_topic_;
  std::string frame_id_;
  std::string frame_prefix_;
  std::string viz_mode_;
  bool overlay_grid_in_urdf_ = false;
  double urdf_force_x_sign_ = 1.0;
  double urdf_force_y_sign_ = 1.0;

  std::string grid_config_yaml_;
  int canvas_rows_ = 14;
  int canvas_cols_ = 24;
  double cell_size_ = 0.006;
  double origin_x_ = 0.0;
  double origin_y_ = 0.0;

  double force_scale_ = 1.0;
  double max_force_ = 0.02;
  bool use_fz_only_ = false;
  bool use_xy_direction_ = true;
  double baseline_duration_sec_ = 3.0;
  double baseline_deadband_xy_ = 0.0;
  double baseline_deadband_z_ = 0.0;
  double baseline_deadband_taxel_xy_ = 0.0;
  double baseline_deadband_taxel_z_ = 0.0;
  bool use_axis_normalization_ = true;
  double xy_force_range_ = 0.8;
  double z_force_range_ = 14.0;
  bool use_taxels_when_no_forces_ = true;
  double xy_taxel_range_ = 1820.0;
  double z_taxel_range_ = 18520.0;
  double urdf_taxel_scale_ = 0.3;

  double circle_min_radius_ = 0.001;
  double circle_max_radius_ = 0.003;
  double circle_height_ = 0.002;
  bool use_cell_scale_ = true;
  double circle_area_scale_ = 2.0;

  double arrow_min_length_ = 0.002;
  double arrow_max_length_ = 0.006;
  double arrow_shaft_diameter_ = 0.0006;
  double arrow_head_diameter_ = 0.0012;
  double arrow_head_length_ = 0.0018;
  double arrow_length_scale_ = 2.0;

  std::string marker_stamp_mode_ = "now";
  double min_marker_scale_ = 1e-4;
  bool debug_touch_stats_ = false;
  double marker_time_offset_sec_ = 0.0;

  double grid_thickness_ = 0.0006;
  bool grid_lines_enabled_ = true;
  double grid_line_width_ = 0.0004;
  double grid_line_alpha_ = 0.2;

  double grid_alpha_ = 1.0;
  double circle_alpha_ = 0.9;
  double arrow_alpha_ = 1.0;

  ColorRGB grid_color_{0.91, 0.89, 0.86};
  ColorRGB grid_line_color_{0.2, 0.2, 0.2};
  ColorRGB color_low_{0.2, 0.4, 1.0};
  ColorRGB color_high_{1.0, 0.2, 0.2};

  double force_x_sign_ = 1.0;
  double force_y_sign_ = -1.0;

  std::vector<ModuleBlock> modules_;
  std::unordered_map<std::string, size_t> module_index_by_name_;
  std::unordered_map<std::string, LayoutTemplate> layout_templates_;
  size_t total_taxels_ = 0;
  std::vector<geometry_msgs::msg::Point> taxel_points_;
  std::vector<visualization_msgs::msg::Marker> grid_markers_;
  BaselineState baseline_;

  rclcpp::Subscription<xela_taxel_msgs::msg::XTaxelSensorTArray>::SharedPtr sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_mode_service_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StdXelaTaxelVizDg5f>());
  rclcpp::shutdown();
  return 0;
}
