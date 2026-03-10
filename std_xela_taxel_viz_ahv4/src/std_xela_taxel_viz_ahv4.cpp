#include <cmath>
#include <string>
#include <unordered_map>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "xela_taxel_msgs/msg/x_taxel_sensor_t_array.hpp"

namespace {
struct Vec3 {
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct ColorRGB {
  double r{1.0};
  double g{1.0};
  double b{1.0};
};

struct BaselineState {
  bool started{false};
  bool ready{false};
  rclcpp::Time start_time;
  std::vector<Vec3> sum;
  std::vector<Vec3> base;
  std::vector<size_t> counts;
};

ColorRGB lerpColor(const ColorRGB &a, const ColorRGB &b, double t) {
  t = std::max(0.0, std::min(1.0, t));
  return {a.r + (b.r - a.r) * t, a.g + (b.g - a.g) * t, a.b + (b.b - a.b) * t};
}

std_msgs::msg::ColorRGBA toColorMsg(const ColorRGB &c, double a) {
  std_msgs::msg::ColorRGBA out;
  out.r = static_cast<float>(c.r);
  out.g = static_cast<float>(c.g);
  out.b = static_cast<float>(c.b);
  out.a = static_cast<float>(a);
  return out;
}

std::vector<double> getVecParam(rclcpp::Node &node, const std::string &name,
                                const std::vector<double> &defaults,
                                const rclcpp::Logger &logger) {
  std::vector<double> out = defaults;
  if (node.has_parameter(name)) {
    auto param = node.get_parameter(name).as_double_array();
    if (param.size() >= defaults.size()) {
      out.assign(param.begin(), param.begin() + defaults.size());
    } else {
      RCLCPP_WARN(logger, "Parameter %s size mismatch; using defaults.", name.c_str());
    }
  }
  return out;
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

class StdXelaTaxelVizAhv4 : public rclcpp::Node {
public:
  StdXelaTaxelVizAhv4() : Node("std_xela_taxel_viz_ahv4") {
    declare_parameter<std::string>("in_topic", "/x_taxel_ah");
    declare_parameter<std::string>("out_topic", "markers");
    declare_parameter<std::string>("frame_id", "world");
    declare_parameter<std::string>("frame_prefix", "");
    declare_parameter<std::string>("mapping_yaml", "");
    declare_parameter<std::string>("hand_side", "left");
    declare_parameter<std::string>("pattern_yaml", "");
    declare_parameter<std::string>("viz_mode", "grid");
    declare_parameter<bool>("overlay_grid_in_urdf", false);
    declare_parameter<double>("urdf_taxel_scale", 1.0);

    declare_parameter<bool>("show_thumb", true);
    declare_parameter<bool>("show_index", true);
    declare_parameter<bool>("show_middle", true);
    declare_parameter<bool>("show_ring", true);
    declare_parameter<bool>("show_palm", true);

    for (const std::string& mod : {"01", "02", "03", "10", "11", "12", "13", "20", "21", "22", "23", "30", "31", "32", "33", "51", "52", "53"}) {
        declare_parameter<bool>("show_module_" + mod, true);
        show_modules_[mod] = true;
    }

    declare_parameter<int>("grid_rows", 31);
    declare_parameter<int>("grid_cols", 26);
    declare_parameter<double>("cell_size", 0.01);
    declare_parameter<double>("origin_x", 0.0);
    declare_parameter<double>("origin_y", 0.0);

    declare_parameter<bool>("use_forces_if_present", true);
    declare_parameter<double>("baseline_duration_sec", 2.0);
    declare_parameter<bool>("use_axis_normalization", true);
    declare_parameter<double>("xy_range", 1350.0);
    declare_parameter<double>("z_range", 16000.0);
    declare_parameter<std::string>("marker_stamp_mode", "keep");
    declare_parameter<double>("marker_time_offset_sec", 0.0);

    declare_parameter<bool>("use_fz_only", false);
    declare_parameter<bool>("use_xy_direction", true);
    declare_parameter<double>("max_force", 1.0);
    declare_parameter<double>("force_x_sign", 1.0);
    declare_parameter<double>("force_y_sign", 1.0);

    declare_parameter<bool>("use_cell_scale", true);
    declare_parameter<double>("circle_area_scale", 2.0);
    declare_parameter<double>("circle_height", 0.002);
    declare_parameter<double>("circle_min_radius", 0.001);
    declare_parameter<double>("circle_max_radius", 0.004);

    declare_parameter<double>("arrow_length_scale", 2.0);
    declare_parameter<double>("arrow_min_length", 0.0);
    declare_parameter<double>("arrow_max_length", 0.02);
    declare_parameter<double>("arrow_shaft_diameter", 0.001);
    declare_parameter<double>("arrow_head_diameter", 0.002);
    declare_parameter<double>("arrow_head_length", 0.003);

    declare_parameter<double>("grid_thickness", 0.0008);
    declare_parameter<bool>("grid_lines_enabled", true);
    declare_parameter<double>("grid_line_width", 0.0004);
    declare_parameter<double>("grid_line_alpha", 0.2);
    declare_parameter<std::vector<double>>("grid_line_color", {0.2, 0.2, 0.2});

    declare_parameter<double>("grid_alpha", 1.0);
    declare_parameter<double>("circle_alpha", 0.9);
    declare_parameter<double>("arrow_alpha", 1.0);

    declare_parameter<std::vector<double>>("grid_color", {0.91, 0.89, 0.86});
    declare_parameter<std::vector<double>>("color_low", {0.2, 0.4, 1.0});
    declare_parameter<std::vector<double>>("color_high", {1.0, 0.2, 0.2});

    in_topic_ = get_parameter("in_topic").as_string();
    out_topic_ = get_parameter("out_topic").as_string();
    frame_id_ = get_parameter("frame_id").as_string();
    frame_prefix_ = get_parameter("frame_prefix").as_string();
    if (!frame_prefix_.empty() && frame_id_.rfind(frame_prefix_, 0) != 0) {
      frame_id_ = frame_prefix_ + frame_id_;
    }
    mapping_yaml_ = get_parameter("mapping_yaml").as_string();
    hand_side_ = get_parameter("hand_side").as_string();
    pattern_yaml_ = get_parameter("pattern_yaml").as_string();
    viz_mode_ = get_parameter("viz_mode").as_string();
    overlay_grid_in_urdf_ = get_parameter("overlay_grid_in_urdf").as_bool();
    urdf_taxel_scale_ = get_parameter("urdf_taxel_scale").as_double();

    show_thumb_ = get_parameter("show_thumb").as_bool();
    show_index_ = get_parameter("show_index").as_bool();
    show_middle_ = get_parameter("show_middle").as_bool();
    show_ring_ = get_parameter("show_ring").as_bool();
    show_palm_ = get_parameter("show_palm").as_bool();

    grid_rows_ = get_parameter("grid_rows").as_int();
    grid_cols_ = get_parameter("grid_cols").as_int();
    cell_size_ = get_parameter("cell_size").as_double();
    origin_x_ = get_parameter("origin_x").as_double();
    origin_y_ = get_parameter("origin_y").as_double();

    use_forces_if_present_ = get_parameter("use_forces_if_present").as_bool();
    baseline_duration_sec_ = get_parameter("baseline_duration_sec").as_double();
    use_axis_normalization_ = get_parameter("use_axis_normalization").as_bool();
    xy_range_ = get_parameter("xy_range").as_double();
    z_range_ = get_parameter("z_range").as_double();
    marker_stamp_mode_ = get_parameter("marker_stamp_mode").as_string();
    marker_time_offset_sec_ = get_parameter("marker_time_offset_sec").as_double();

    use_fz_only_ = get_parameter("use_fz_only").as_bool();
    use_xy_direction_ = get_parameter("use_xy_direction").as_bool();
    max_force_ = get_parameter("max_force").as_double();
    force_x_sign_ = get_parameter("force_x_sign").as_double();
    force_y_sign_ = get_parameter("force_y_sign").as_double();

    use_cell_scale_ = get_parameter("use_cell_scale").as_bool();
    circle_area_scale_ = get_parameter("circle_area_scale").as_double();
    circle_height_ = get_parameter("circle_height").as_double();
    circle_min_radius_ = get_parameter("circle_min_radius").as_double();
    circle_max_radius_ = get_parameter("circle_max_radius").as_double();

    arrow_length_scale_ = get_parameter("arrow_length_scale").as_double();
    arrow_min_length_ = get_parameter("arrow_min_length").as_double();
    arrow_max_length_ = get_parameter("arrow_max_length").as_double();
    arrow_shaft_diameter_ = get_parameter("arrow_shaft_diameter").as_double();
    arrow_head_diameter_ = get_parameter("arrow_head_diameter").as_double();
    arrow_head_length_ = get_parameter("arrow_head_length").as_double();

    grid_thickness_ = get_parameter("grid_thickness").as_double();
    grid_lines_enabled_ = get_parameter("grid_lines_enabled").as_bool();
    grid_line_width_ = get_parameter("grid_line_width").as_double();
    grid_line_alpha_ = get_parameter("grid_line_alpha").as_double();

    grid_alpha_ = get_parameter("grid_alpha").as_double();
    circle_alpha_ = get_parameter("circle_alpha").as_double();
    arrow_alpha_ = get_parameter("arrow_alpha").as_double();

    auto grid_color_vec = getVecParam(*this, "grid_color", {0.91, 0.89, 0.86}, get_logger());
    auto grid_line_color_vec = getVecParam(*this, "grid_line_color", {0.2, 0.2, 0.2}, get_logger());
    auto color_low_vec = getVecParam(*this, "color_low", {0.2, 0.4, 1.0}, get_logger());
    auto color_high_vec = getVecParam(*this, "color_high", {1.0, 0.2, 0.2}, get_logger());
    grid_color_ = {grid_color_vec[0], grid_color_vec[1], grid_color_vec[2]};
    grid_line_color_ = {grid_line_color_vec[0], grid_line_color_vec[1], grid_line_color_vec[2]};
    color_low_ = {color_low_vec[0], color_low_vec[1], color_low_vec[2]};
    color_high_ = {color_high_vec[0], color_high_vec[1], color_high_vec[2]};

    if (grid_rows_ <= 0 || grid_cols_ <= 0) {
      RCLCPP_WARN(get_logger(), "Invalid grid size; using 31x26.");
      grid_rows_ = 31;
      grid_cols_ = 26;
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
    if (xy_range_ <= 0.0) {
      RCLCPP_WARN(get_logger(), "xy_range <= 0, defaulting to 1350.");
      xy_range_ = 1350.0;
    }
    if (z_range_ <= 0.0) {
      RCLCPP_WARN(get_logger(), "z_range <= 0, defaulting to 16000.");
      z_range_ = 16000.0;
    }

    if (viz_mode_ != "grid" && viz_mode_ != "urdf") {
      RCLCPP_WARN(get_logger(), "viz_mode '%s' invalid; defaulting to grid.", viz_mode_.c_str());
      viz_mode_ = "grid";
    }
    if (urdf_taxel_scale_ <= 0.0) {
      RCLCPP_WARN(get_logger(), "urdf_taxel_scale <= 0; defaulting to 1.0.");
      urdf_taxel_scale_ = 1.0;
    }
    show_grid_ = (viz_mode_ == "grid") || overlay_grid_in_urdf_;

    loadMapping();
    if (show_grid_) {
      loadPattern();
      buildGridMarkers();
    }

    baseline_.sum.assign(data_size_, {});
    baseline_.base.assign(data_size_, {});
    baseline_.counts.assign(data_size_, 0);

    on_set_parameters_handle_ = add_on_set_parameters_callback(
        std::bind(&StdXelaTaxelVizAhv4::onSetParameters, this, std::placeholders::_1));

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    sub_ = create_subscription<xela_taxel_msgs::msg::XTaxelSensorTArray>(
        in_topic_, qos,
        std::bind(&StdXelaTaxelVizAhv4::onArray, this, std::placeholders::_1));
    pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(out_topic_, 10);

    RCLCPP_INFO(get_logger(), "std_xela_taxel_viz_ahv4 started. in: %s out: %s",
                in_topic_.c_str(), out_topic_.c_str());
  }

private:
  rcl_interfaces::msg::SetParametersResult onSetParameters(
      const std::vector<rclcpp::Parameter> &parameters) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    for (const auto &param : parameters) {
      if (param.get_name() == "show_thumb") {
        show_thumb_ = param.as_bool();
      } else if (param.get_name() == "show_index") {
        show_index_ = param.as_bool();
      } else if (param.get_name() == "show_middle") {
        show_middle_ = param.as_bool();
      } else if (param.get_name() == "show_ring") {
        show_ring_ = param.as_bool();
      } else if (param.get_name() == "show_palm") {
        show_palm_ = param.as_bool();
      } else if (param.get_name().find("show_module_") == 0) {
        std::string mod_id = param.get_name().substr(12);
        if (show_modules_.find(mod_id) != show_modules_.end()) {
          show_modules_[mod_id] = param.as_bool();
        }
      } else if (param.get_name() == "overlay_grid_in_urdf") {
        overlay_grid_in_urdf_ = param.as_bool();
        show_grid_ = (viz_mode_ == "grid") || overlay_grid_in_urdf_;
      } else if (param.get_name() == "viz_mode") {
        viz_mode_ = param.as_string();
        show_grid_ = (viz_mode_ == "grid") || overlay_grid_in_urdf_;
      }
    }
    return result;
  }

  std::string applyFramePrefix(const std::string &frame_id) const {
    if (frame_prefix_.empty()) {
      return frame_id;
    }
    if (frame_id.rfind(frame_prefix_, 0) == 0) {
      return frame_id;
    }
    return frame_prefix_ + frame_id;
  }

  void loadMapping() {
    if (mapping_yaml_.empty()) {
      throw std::runtime_error("mapping_yaml is empty");
    }
    std::string prefix_from = "x_taxel_0_";
    std::string prefix_to = "x_taxel_0_";
    if (hand_side_ == "right" || hand_side_ == "r") {
      prefix_from = "x_taxel_0_";
      prefix_to = "x_taxel_1_";
    } else if (hand_side_ == "left" || hand_side_ == "l") {
      prefix_from = "x_taxel_1_";
      prefix_to = "x_taxel_0_";
    }
    YAML::Node root = YAML::LoadFile(mapping_yaml_);
    YAML::Node map_node = root["taxel_joint_map"];
    if (!map_node) {
      throw std::runtime_error("taxel_joint_map not found in mapping_yaml");
    }

    size_t max_index = 0;
    for (const auto &it : map_node) {
      int idx = it.first.as<int>();
      std::string joint = it.second.as<std::string>();
      SensorGroup group = identifyGroup(joint);
      std::string mod_id = extractModuleId(joint);
      
      if (joint.rfind(prefix_from, 0) == 0) {
        joint.replace(0, prefix_from.size(), prefix_to);
      }
      if (idx < 0) {
        continue;
      }
      max_index = std::max(max_index, static_cast<size_t>(idx));
      joint_to_index_[joint] = static_cast<size_t>(idx);
      
      if (index_to_group_.size() <= static_cast<size_t>(idx)) {
        index_to_group_.resize(static_cast<size_t>(idx) + 1, SensorGroup::NONE);
        index_to_module_.resize(static_cast<size_t>(idx) + 1, "");
      }
      index_to_group_[static_cast<size_t>(idx)] = group;
      index_to_module_[static_cast<size_t>(idx)] = mod_id;
    }
    data_size_ = max_index + 1;
    if (data_size_ == 0) {
      throw std::runtime_error("mapping_yaml has no valid indices");
    }
    RCLCPP_INFO(get_logger(), "Loaded mapping_yaml with %zu indices (0..%zu).",
                data_size_, data_size_ - 1);
    if (hand_side_ == "right" || hand_side_ == "r") {
      RCLCPP_INFO(get_logger(), "Applied hand_side=right mapping prefix x_taxel_1_.");
    } else if (hand_side_ == "left" || hand_side_ == "l") {
      RCLCPP_INFO(get_logger(), "Applied hand_side=left mapping prefix x_taxel_0_.");
    }
  }

  void loadPattern() {
    if (pattern_yaml_.empty()) {
      throw std::runtime_error("pattern_yaml is empty");
    }
    YAML::Node root = YAML::LoadFile(pattern_yaml_);
    YAML::Node pattern = root["pattern"];
    if (!pattern) {
      throw std::runtime_error("pattern section not found in pattern_yaml");
    }

    int rows = pattern["rows"].as<int>();
    int cols = pattern["cols"].as<int>();
    YAML::Node index_map = pattern["index_map"];
    if (!index_map || !index_map.IsSequence()) {
      throw std::runtime_error("index_map not found or invalid in pattern_yaml");
    }

    if (rows <= 0 || cols <= 0) {
      throw std::runtime_error("pattern rows/cols invalid");
    }

    grid_rows_ = rows;
    grid_cols_ = cols;
    grid_size_cells_ = static_cast<size_t>(grid_rows_ * grid_cols_);

    index_positions_.assign(data_size_, geometry_msgs::msg::Point());
    index_has_pos_.assign(data_size_, false);
    cell_active_.assign(static_cast<size_t>(grid_rows_ * grid_cols_), false);

    if (static_cast<int>(index_map.size()) != grid_rows_) {
      RCLCPP_WARN(get_logger(), "pattern rows mismatch: index_map rows=%zu, expected=%d",
                  index_map.size(), grid_rows_);
    }

    for (int r = 0; r < grid_rows_; ++r) {
      YAML::Node row = index_map[r];
      if (!row || !row.IsSequence()) {
        continue;
      }
      for (int c = 0; c < grid_cols_ && c < static_cast<int>(row.size()); ++c) {
        int idx = row[c].as<int>();
        if (idx < 0) {
          continue;
        }
        if (static_cast<size_t>(idx) >= data_size_) {
          RCLCPP_WARN(get_logger(), "Pattern index %d out of range (size=%zu).", idx, data_size_);
          continue;
        }
        geometry_msgs::msg::Point p;
        p.x = origin_x_ + (static_cast<double>(c) + 0.5) * cell_size_;
        p.y = origin_y_ - (static_cast<double>(r) + 0.5) * cell_size_;
        p.z = 0.0;
        if (index_has_pos_[idx]) {
          RCLCPP_WARN(get_logger(), "Duplicate index %d in pattern; using first occurrence.", idx);
          continue;
        }
        index_positions_[idx] = p;
        index_has_pos_[idx] = true;
        cell_active_[static_cast<size_t>(r * grid_cols_ + c)] = true;
      }
    }

    size_t active = 0;
    for (bool v : index_has_pos_) {
      if (v) {
        active++;
      }
    }
    RCLCPP_INFO(get_logger(), "Loaded pattern_yaml: %dx%d with %zu active indices.",
                grid_rows_, grid_cols_, active);
  }

  void buildGridMarkers() {
    grid_markers_.clear();
    int id = 0;
    for (int r = 0; r < grid_rows_; ++r) {
      for (int c = 0; c < grid_cols_; ++c) {
        if (!cell_active_.empty()) {
          size_t cell_idx = static_cast<size_t>(r * grid_cols_ + c);
          if (cell_idx < cell_active_.size() && !cell_active_[cell_idx]) {
            continue;
          }
        }
        visualization_msgs::msg::Marker m;
        m.ns = "grid";
        m.id = id++;
        m.type = visualization_msgs::msg::Marker::CUBE;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.pose.orientation.w = 1.0;
        m.pose.position.x = origin_x_ + (static_cast<double>(c) + 0.5) * cell_size_;
        m.pose.position.y = origin_y_ - (static_cast<double>(r) + 0.5) * cell_size_;
        m.pose.position.z = -grid_thickness_ * 0.5;
        m.scale.x = cell_size_;
        m.scale.y = cell_size_;
        m.scale.z = grid_thickness_;
        m.color = toColorMsg(grid_color_, grid_alpha_);
        grid_markers_.push_back(m);
      }
    }

    if (grid_lines_enabled_) {
      visualization_msgs::msg::Marker lines;
      lines.ns = "grid_lines";
      lines.id = 0;
      lines.type = visualization_msgs::msg::Marker::LINE_LIST;
      lines.action = visualization_msgs::msg::Marker::ADD;
      lines.pose.orientation.w = 1.0;
      lines.scale.x = grid_line_width_;
      lines.color = toColorMsg(grid_line_color_, grid_line_alpha_);

      double x0 = origin_x_;
      double y0 = origin_y_;
      double x1 = origin_x_ + grid_cols_ * cell_size_;
      double y1 = origin_y_ - grid_rows_ * cell_size_;

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

      grid_markers_.push_back(lines);
    }
  }

  void ensureBaselineSize() {
    if (baseline_.sum.size() == data_size_) {
      return;
    }
    baseline_.sum.assign(data_size_, {});
    baseline_.base.assign(data_size_, {});
    baseline_.counts.assign(data_size_, 0);
  }

  void updateBaseline(const std::vector<Vec3> &current,
                      const std::vector<bool> &present,
                      const rclcpp::Time &stamp) {
    if (baseline_duration_sec_ <= 0.0) {
      baseline_.ready = true;
      return;
    }

    ensureBaselineSize();

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
      for (size_t i = 0; i < data_size_; ++i) {
        if (!present[i]) {
          continue;
        }
        baseline_.sum[i].x += current[i].x;
        baseline_.sum[i].y += current[i].y;
        baseline_.sum[i].z += current[i].z;
        baseline_.counts[i] += 1;
      }
    }

    if (elapsed >= baseline_duration_sec_) {
      for (size_t i = 0; i < data_size_; ++i) {
        if (baseline_.counts[i] == 0) {
          continue;
        }
        baseline_.base[i].x = baseline_.sum[i].x / baseline_.counts[i];
        baseline_.base[i].y = baseline_.sum[i].y / baseline_.counts[i];
        baseline_.base[i].z = baseline_.sum[i].z / baseline_.counts[i];
      }
      baseline_.ready = true;
      RCLCPP_INFO(get_logger(), "Baseline computed with duration %.2fs.", baseline_duration_sec_);
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

  struct Sample {
    double fx{0.0};
    double fy{0.0};
    double fz{0.0};
    double magnitude{0.0};
    double normalized{0.0};
  };

  Sample computeSample(size_t idx, const std::vector<Vec3> &current) {
    Sample s;
    if (idx >= current.size()) {
      return s;
    }

    double fx = current[idx].x;
    double fy = current[idx].y;
    double fz = current[idx].z;

    if (!baseline_.ready && baseline_duration_sec_ > 0.0) {
      fx = 0.0;
      fy = 0.0;
      fz = 0.0;
    } else if (baseline_.base.size() == data_size_) {
      fx -= baseline_.base[idx].x;
      fy -= baseline_.base[idx].y;
      fz -= baseline_.base[idx].z;
    }

    fx *= force_x_sign_;
    fy *= force_y_sign_;

    double magnitude = 0.0;
    double normalized = 0.0;
    if (use_fz_only_) {
      magnitude = std::fabs(fz);
      if (max_force_ > 0.0) {
        normalized = std::min(magnitude / max_force_, 1.0);
      }
    } else if (use_axis_normalization_) {
      double z_clamped = std::max(0.0, fz);
      double nx = fx / xy_range_;
      double ny = fy / xy_range_;
      double nz = z_clamped / z_range_;
      magnitude = std::sqrt(fx * fx + fy * fy + fz * fz);
      normalized = std::min(std::sqrt(nx * nx + ny * ny + nz * nz), 1.0);
    } else {
      magnitude = std::sqrt(fx * fx + fy * fy + fz * fz);
      if (max_force_ > 0.0) {
        normalized = std::min(magnitude / max_force_, 1.0);
      }
    }

    s.fx = fx;
    s.fy = fy;
    s.fz = fz;
    s.magnitude = magnitude;
    s.normalized = normalized;
    return s;
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

  std::string jointToLinkName(const std::string &joint) const {
    const std::string suffix = "_joint";
    if (joint.size() > suffix.size() &&
        joint.compare(joint.size() - suffix.size(), suffix.size(), suffix) == 0) {
      return applyFramePrefix(joint.substr(0, joint.size() - suffix.size()) + "_link");
    }
    return applyFramePrefix(joint + "_link");
  }

  void appendMarkersUrdf(visualization_msgs::msg::MarkerArray &out,
                         const xela_taxel_msgs::msg::XTaxelSensorTArray &msg,
                         const std::vector<Vec3> &current,
                         const rclcpp::Time &stamp) {
    for (const auto &module : msg.x_modules) {
      for (const auto &fid : module.frame_ids) {
        auto it = joint_to_index_.find(fid);
        if (it == joint_to_index_.end()) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                               "frame_id not in mapping: %s", fid.c_str());
          continue;
        }
        size_t idx = it->second;
        if (idx >= data_size_) {
          continue;
        }
        
        if (!isIndexVisible(idx)) {
          continue;
        }

        Sample s = computeSample(idx, current);
        double radius = circle_min_radius_ + s.normalized * (circle_max_radius_ - circle_min_radius_);
        double arrow_len = arrow_min_length_ + s.normalized * (arrow_max_length_ - arrow_min_length_);

        radius *= urdf_taxel_scale_;
        arrow_len *= urdf_taxel_scale_;

        visualization_msgs::msg::Marker circle;
        circle.header.frame_id = jointToLinkName(fid);
        circle.header.stamp = stamp;
        circle.ns = "urdf_circle";
        circle.id = static_cast<int>(idx);
        circle.type = visualization_msgs::msg::Marker::CYLINDER;
        circle.action = visualization_msgs::msg::Marker::ADD;
        circle.pose.orientation.w = 1.0;
        circle.pose.position.x = 0.0;
        circle.pose.position.y = 0.0;
        circle.pose.position.z = (circle_height_ * 0.5) * urdf_taxel_scale_;
        circle.scale.x = radius * 2.0;
        circle.scale.y = radius * 2.0;
        circle.scale.z = circle_height_ * urdf_taxel_scale_;
        circle.color = toColorMsg(lerpColor(color_low_, color_high_, s.normalized), circle_alpha_);
        out.markers.push_back(std::move(circle));

        visualization_msgs::msg::Marker arrow;
        arrow.header.frame_id = jointToLinkName(fid);
        arrow.header.stamp = stamp;
        arrow.ns = "urdf_arrow";
        arrow.id = static_cast<int>(idx);
        arrow.type = visualization_msgs::msg::Marker::ARROW;
        arrow.action = visualization_msgs::msg::Marker::ADD;
        arrow.scale.x = arrow_shaft_diameter_ * urdf_taxel_scale_;
        arrow.scale.y = arrow_head_diameter_ * urdf_taxel_scale_;
        arrow.scale.z = arrow_head_length_ * urdf_taxel_scale_;
        arrow.color = toColorMsg({0.1, 0.1, 0.1}, arrow_alpha_);

        geometry_msgs::msg::Point start;
        start.x = 0.0;
        start.y = 0.0;
        start.z = (circle_height_ + 0.001) * urdf_taxel_scale_;
        geometry_msgs::msg::Point end = start;
        if (arrow_len > 0.0) {
          if (use_xy_direction_) {
            double dir_len = std::sqrt(s.fx * s.fx + s.fy * s.fy);
            if (dir_len > 1e-9) {
              end.x += (s.fx / dir_len) * arrow_len;
              end.y += (s.fy / dir_len) * arrow_len;
            }
          } else {
            double dir = (s.fz >= 0.0) ? 1.0 : -1.0;
            end.z += dir * arrow_len;
          }
        }
        arrow.points = {start, end};
        out.markers.push_back(std::move(arrow));
      }
    }
  }

  void onArray(const xela_taxel_msgs::msg::XTaxelSensorTArray::SharedPtr msg) {
    std::vector<Vec3> current(data_size_);
    std::vector<bool> present(data_size_, false);

    for (const auto &module : msg->x_modules) {
      bool use_forces = use_forces_if_present_ && !module.forces.empty();
      size_t n = std::min(module.frame_ids.size(),
                          use_forces ? module.forces.size() : module.taxels.size());
      for (size_t j = 0; j < n; ++j) {
        const auto &fid = module.frame_ids[j];
        auto it = joint_to_index_.find(fid);
        if (it == joint_to_index_.end()) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                               "frame_id not in mapping: %s", fid.c_str());
          continue;
        }
        size_t idx = it->second;
        if (idx >= data_size_) {
          continue;
        }
        Vec3 v;
        if (use_forces) {
          v.x = static_cast<double>(module.forces[j].x);
          v.y = static_cast<double>(module.forces[j].y);
          v.z = static_cast<double>(module.forces[j].z);
        } else {
          v.x = static_cast<double>(module.taxels[j].x);
          v.y = static_cast<double>(module.taxels[j].y);
          v.z = static_cast<double>(module.taxels[j].z);
        }
        current[idx] = v;
        present[idx] = true;
      }
    }

    rclcpp::Time stamp = getStamp(msg->header);
    updateBaseline(current, present, stamp);

    visualization_msgs::msg::MarkerArray out;
    if (show_grid_) {
      for (auto marker : grid_markers_) {
        marker.header.frame_id = frame_id_;
        marker.header.stamp = stamp;
        out.markers.push_back(std::move(marker));
      }
    }

    if (show_grid_ && !index_has_pos_.empty()) {
      for (size_t idx = 0; idx < data_size_; ++idx) {
        if (!index_has_pos_[idx]) {
          continue;
        }
        if (!isIndexVisible(idx)) {
          continue;
        }
        Sample s = computeSample(idx, current);

        double radius = circle_min_radius_ + s.normalized * (circle_max_radius_ - circle_min_radius_);
        double arrow_len = arrow_min_length_ + s.normalized * (arrow_max_length_ - arrow_min_length_);

        const auto &center = index_positions_[idx];

        visualization_msgs::msg::Marker circle;
        circle.header.frame_id = frame_id_;
        circle.header.stamp = stamp;
        circle.ns = "circle";
        circle.id = static_cast<int>(idx);
        circle.type = visualization_msgs::msg::Marker::CYLINDER;
        circle.action = visualization_msgs::msg::Marker::ADD;
        circle.pose.position = center;
        circle.pose.position.z = circle_height_ * 0.5;
        circle.pose.orientation.w = 1.0;
        circle.scale.x = radius * 2.0;
        circle.scale.y = radius * 2.0;
        circle.scale.z = circle_height_;
        circle.color = toColorMsg(lerpColor(color_low_, color_high_, s.normalized), circle_alpha_);
        out.markers.push_back(std::move(circle));

        visualization_msgs::msg::Marker arrow;
        arrow.header.frame_id = frame_id_;
        arrow.header.stamp = stamp;
        arrow.ns = "arrow";
        arrow.id = static_cast<int>(idx);
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
            double dir_len = std::sqrt(s.fx * s.fx + s.fy * s.fy);
            if (dir_len > 1e-9) {
              end.x += (s.fx / dir_len) * arrow_len;
              end.y += (s.fy / dir_len) * arrow_len;
            }
          } else {
            double dir = (s.fz >= 0.0) ? 1.0 : -1.0;
            end.z += dir * arrow_len;
          }
        }
        arrow.points = {start, end};
        out.markers.push_back(std::move(arrow));
      }
    }

    if (viz_mode_ == "urdf") {
      appendMarkersUrdf(out, *msg, current, stamp);
    }

    pub_->publish(out);
  }

  std::string in_topic_;
  std::string out_topic_;
  std::string frame_id_;
  std::string frame_prefix_;
  std::string mapping_yaml_;
  std::string hand_side_;
  std::string pattern_yaml_;
  std::string viz_mode_{"grid"};
  bool overlay_grid_in_urdf_{false};
  double urdf_taxel_scale_{1.0};
  bool show_grid_{true};

  bool show_thumb_{true};
  bool show_index_{true};
  bool show_middle_{true};
  bool show_ring_{true};
  bool show_palm_{true};

  int grid_rows_{31};
  int grid_cols_{26};
  size_t grid_size_cells_{0};
  
  std::vector<SensorGroup> index_to_group_;
  std::vector<std::string> index_to_module_;
  std::unordered_map<std::string, bool> show_modules_ = {
    {"01", true}, {"02", true}, {"03", true},
    {"10", true}, {"11", true}, {"12", true}, {"13", true},
    {"20", true}, {"21", true}, {"22", true}, {"23", true},
    {"30", true}, {"31", true}, {"32", true}, {"33", true},
    {"51", true}, {"52", true}, {"53", true}
  };

  size_t data_size_{368};
  double cell_size_{0.01};
  double origin_x_{0.0};
  double origin_y_{0.0};

  bool use_forces_if_present_{true};
  double baseline_duration_sec_{2.0};
  bool use_axis_normalization_{true};
  double xy_range_{1350.0};
  double z_range_{16000.0};
  std::string marker_stamp_mode_{"keep"};
  double marker_time_offset_sec_{0.0};

  bool use_fz_only_{false};
  bool use_xy_direction_{true};
  double max_force_{1.0};
  double force_x_sign_{1.0};
  double force_y_sign_{1.0};

  bool use_cell_scale_{true};
  double circle_area_scale_{2.0};
  double circle_height_{0.002};
  double circle_min_radius_{0.001};
  double circle_max_radius_{0.004};

  double arrow_length_scale_{2.0};
  double arrow_min_length_{0.0};
  double arrow_max_length_{0.02};
  double arrow_shaft_diameter_{0.001};
  double arrow_head_diameter_{0.002};
  double arrow_head_length_{0.003};

  double grid_thickness_{0.0008};
  bool grid_lines_enabled_{true};
  double grid_line_width_{0.0004};
  double grid_line_alpha_{0.2};

  double grid_alpha_{1.0};
  double circle_alpha_{0.9};
  double arrow_alpha_{1.0};

  ColorRGB grid_color_{0.91, 0.89, 0.86};
  ColorRGB grid_line_color_{0.2, 0.2, 0.2};
  ColorRGB color_low_{0.2, 0.4, 1.0};
  ColorRGB color_high_{1.0, 0.2, 0.2};

  std::unordered_map<std::string, size_t> joint_to_index_;
  std::vector<geometry_msgs::msg::Point> index_positions_;
  std::vector<bool> index_has_pos_;
  std::vector<bool> cell_active_;
  std::vector<visualization_msgs::msg::Marker> grid_markers_;
  BaselineState baseline_;

  rclcpp::Subscription<xela_taxel_msgs::msg::XTaxelSensorTArray>::SharedPtr sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_;
  OnSetParametersCallbackHandle::SharedPtr on_set_parameters_handle_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<StdXelaTaxelVizAhv4>();
    rclcpp::spin(node);
  } catch (const std::exception &e) {
    fprintf(stderr, "Fatal error: %s\n", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
