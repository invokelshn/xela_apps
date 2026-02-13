#include <algorithm>
#include <cmath>
#include <string>
#include <unordered_map>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "rclcpp/rclcpp.hpp"
#include "xela_taxel_msgs/msg/x_taxel_sensor_t_array.hpp"

class XelaTaxelVizAhv4Demo : public rclcpp::Node {
public:
  XelaTaxelVizAhv4Demo() : Node("xela_taxel_viz_ahv4_demo") {
    declare_parameter<std::string>("out_topic", "/x_taxel_ah");
    declare_parameter<std::string>("mapping_yaml", "");
    declare_parameter<std::string>("hand_side", "left");
    declare_parameter<std::string>("frame_id", "base_link");
    declare_parameter<std::string>("model", "uSCuAH");
    declare_parameter<int>("sensor_pos", 0);
    declare_parameter<double>("publish_rate_hz", 20.0);
    declare_parameter<double>("base_x", 30000.0);
    declare_parameter<double>("base_y", 30000.0);
    declare_parameter<double>("base_z", 30000.0);
    declare_parameter<double>("xy_amp", 1350.0);
    declare_parameter<double>("z_amp", 8000.0);
    declare_parameter<double>("phase_step", 0.15);
    declare_parameter<double>("freq_hz", 0.2);

    out_topic_ = get_parameter("out_topic").as_string();
    mapping_yaml_ = get_parameter("mapping_yaml").as_string();
    hand_side_ = get_parameter("hand_side").as_string();
    frame_id_ = get_parameter("frame_id").as_string();
    model_ = get_parameter("model").as_string();
    sensor_pos_ = static_cast<uint8_t>(get_parameter("sensor_pos").as_int());

    publish_rate_hz_ = get_parameter("publish_rate_hz").as_double();
    base_x_ = get_parameter("base_x").as_double();
    base_y_ = get_parameter("base_y").as_double();
    base_z_ = get_parameter("base_z").as_double();
    xy_amp_ = get_parameter("xy_amp").as_double();
    z_amp_ = get_parameter("z_amp").as_double();
    phase_step_ = get_parameter("phase_step").as_double();
    freq_hz_ = get_parameter("freq_hz").as_double();

    loadMapping();

    pub_ = create_publisher<xela_taxel_msgs::msg::XTaxelSensorTArray>(out_topic_, 10);

    if (publish_rate_hz_ <= 0.0) {
      publish_rate_hz_ = 20.0;
    }
    auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&XelaTaxelVizAhv4Demo::onTimer, this));

    RCLCPP_INFO(get_logger(), "xela_taxel_viz_ahv4_demo started. out: %s", out_topic_.c_str());
  }

private:
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
      if (idx < 0) {
        continue;
      }
      max_index = std::max(max_index, static_cast<size_t>(idx));
    }

    size_t size = max_index + 1;
    frame_ids_.assign(size, "");
    for (const auto &it : map_node) {
      int idx = it.first.as<int>();
      if (idx < 0) {
        continue;
      }
      std::string joint = it.second.as<std::string>();
      if (joint.rfind(prefix_from, 0) == 0) {
        joint.replace(0, prefix_from.size(), prefix_to);
      }
      frame_ids_[static_cast<size_t>(idx)] = joint;
    }

    for (const auto &fid : frame_ids_) {
      if (fid.empty()) {
        RCLCPP_WARN(get_logger(), "mapping_yaml has empty frame_id entry");
        break;
      }
    }

    RCLCPP_INFO(get_logger(), "Loaded mapping_yaml with %zu indices.", frame_ids_.size());
    if (hand_side_ == "right" || hand_side_ == "r") {
      RCLCPP_INFO(get_logger(), "Applied hand_side=right mapping prefix x_taxel_1_.");
    } else if (hand_side_ == "left" || hand_side_ == "l") {
      RCLCPP_INFO(get_logger(), "Applied hand_side=left mapping prefix x_taxel_0_.");
    }
  }

  static uint16_t clampU16(double value) {
    if (value < 0.0) {
      return 0;
    }
    if (value > 65535.0) {
      return 65535;
    }
    return static_cast<uint16_t>(value);
  }

  void onTimer() {
    auto msg = xela_taxel_msgs::msg::XTaxelSensorTArray();
    msg.header.stamp = now();
    msg.header.frame_id = frame_id_;

    xela_taxel_msgs::msg::XTaxelSensorT module;
    module.message = seq_++;
    module.time = now().seconds();
    module.model = model_;
    module.sensor_pos = sensor_pos_;

    module.frame_ids = frame_ids_;
    module.taxels.resize(frame_ids_.size());
    module.forces.clear();
    module.temps.assign(frame_ids_.size(), 300.0f);

    double t = now().seconds();
    for (size_t i = 0; i < frame_ids_.size(); ++i) {
      double phase = (t * freq_hz_ * 2.0 * M_PI) + static_cast<double>(i) * phase_step_;
      double dx = xy_amp_ * std::sin(phase);
      double dy = xy_amp_ * std::cos(phase * 0.9);
      double dz = z_amp_ * (0.5 * (std::sin(phase * 0.5) + 1.0));

      module.taxels[i].x = clampU16(base_x_ + dx);
      module.taxels[i].y = clampU16(base_y_ + dy);
      module.taxels[i].z = clampU16(base_z_ + dz);
    }

    msg.x_modules = {module};
    msg.md_frame_ids.clear();
    msg.md_frame_ids.push_back(model_);

    pub_->publish(msg);
  }

  std::string out_topic_;
  std::string mapping_yaml_;
  std::string hand_side_;
  std::string frame_id_;
  std::string model_;
  uint8_t sensor_pos_{0};

  double publish_rate_hz_{20.0};
  double base_x_{30000.0};
  double base_y_{30000.0};
  double base_z_{30000.0};
  double xy_amp_{1350.0};
  double z_amp_{8000.0};
  double phase_step_{0.15};
  double freq_hz_{0.2};

  std::vector<std::string> frame_ids_;
  uint32_t seq_{0};

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<xela_taxel_msgs::msg::XTaxelSensorTArray>::SharedPtr pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<XelaTaxelVizAhv4Demo>();
    rclcpp::spin(node);
  } catch (const std::exception &e) {
    fprintf(stderr, "Fatal error: %s\n", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
