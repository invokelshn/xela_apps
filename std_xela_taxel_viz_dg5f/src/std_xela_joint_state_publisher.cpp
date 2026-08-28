#include <algorithm>
#include <chrono>
#include <filesystem>
#include <string>
#include <unordered_set>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <yaml-cpp/yaml.h>

namespace {
constexpr char kDefaultOutputTopic[] = "joint_states";
constexpr double kDefaultPublishRate = 30.0;
constexpr char kPackagePrefix[] = "package://";

std::string trim(const std::string &input) {
  auto start = input.find_first_not_of(" \t\n\r");
  if (start == std::string::npos) {
    return "";
  }
  auto end = input.find_last_not_of(" \t\n\r");
  return input.substr(start, end - start + 1);
}

void append_unique(std::vector<std::string> &out, const std::vector<std::string> &in) {
  std::unordered_set<std::string> seen(out.begin(), out.end());
  for (const auto &item : in) {
    const auto trimmed = trim(item);
    if (trimmed.empty()) {
      continue;
    }
    if (seen.emplace(trimmed).second) {
      out.push_back(trimmed);
    }
  }
}

std::vector<std::string> parse_string_list(const YAML::Node &node) {
  std::vector<std::string> out;
  if (!node || !node.IsSequence()) {
    return out;
  }
  out.reserve(node.size());
  for (const auto &item : node) {
    if (item.IsScalar()) {
      out.push_back(item.as<std::string>());
    }
  }
  return out;
}

std::vector<std::string> load_keep_joints_file(const std::filesystem::path &path) {
  std::vector<std::string> out;
  YAML::Node file = YAML::LoadFile(path.string());
  YAML::Node list_node;

  if (file["keep_joints"]) {
    list_node = file["keep_joints"];
  } else if (file.IsSequence()) {
    list_node = file;
  }

  if (!list_node || !list_node.IsSequence()) {
    return out;
  }

  out.reserve(list_node.size());
  for (const auto &item : list_node) {
    if (item.IsScalar()) {
      out.push_back(item.as<std::string>());
    }
  }
  return out;
}

std::string resolve_package_path(const std::string &input) {
  if (input.rfind(kPackagePrefix, 0) != 0) {
    return input;
  }

  std::string rest = input.substr(std::char_traits<char>::length(kPackagePrefix));
  auto slash = rest.find('/');
  if (slash == std::string::npos) {
    return input;
  }

  const std::string pkg_name = rest.substr(0, slash);
  const std::string rel_path = rest.substr(slash + 1);

  try {
    auto share_dir = ament_index_cpp::get_package_share_directory(pkg_name);
    return (std::filesystem::path(share_dir) / rel_path).string();
  } catch (const std::exception &) {
    return input;
  }
}

}  // namespace

class StdXelaJointStatePublisher : public rclcpp::Node {
public:
  StdXelaJointStatePublisher() : Node("std_xela_joint_state_publisher") {
    declare_parameter<std::string>("output_topic", kDefaultOutputTopic);
    declare_parameter<double>("publish_rate", kDefaultPublishRate);
    declare_parameter<std::vector<std::string>>("joint_names", std::vector<std::string>{});
    declare_parameter<std::vector<std::string>>("extra_joints", std::vector<std::string>{});
    declare_parameter<std::string>("config_yaml", "");
    declare_parameter<std::vector<std::string>>("device_profiles", std::vector<std::string>{});
    declare_parameter<std::string>("device_profile", "");
    declare_parameter<std::vector<double>>("initial_positions", std::vector<double>{});

    output_topic_ = get_parameter("output_topic").as_string();
    publish_rate_ = get_parameter("publish_rate").as_double();
    joint_names_ = get_parameter("joint_names").as_string_array();
    extra_joints_ = get_parameter("extra_joints").as_string_array();
    config_yaml_ = get_parameter("config_yaml").as_string();
    device_profiles_ = get_parameter("device_profiles").as_string_array();
    device_profile_ = get_parameter("device_profile").as_string();

    if (publish_rate_ <= 0.0) {
      RCLCPP_WARN(get_logger(), "publish_rate <= 0; defaulting to %.1f", kDefaultPublishRate);
      publish_rate_ = kDefaultPublishRate;
    }

    loadJointNames();
    if (joint_names_.empty()) {
      RCLCPP_WARN(get_logger(), "No joint names configured; publishing empty JointState.");
    }

    positions_.assign(joint_names_.size(), 0.0);
    velocities_.assign(joint_names_.size(), 0.0);
    efforts_.assign(joint_names_.size(), 0.0);

    auto initial_positions = get_parameter("initial_positions").as_double_array();
    if (!initial_positions.empty() && initial_positions.size() == joint_names_.size()) {
      positions_.assign(initial_positions.begin(), initial_positions.end());
    } else if (!initial_positions.empty()) {
      RCLCPP_WARN(get_logger(),
                  "initial_positions size mismatch (%zu != %zu); using zeros.",
                  initial_positions.size(), joint_names_.size());
    }

    pub_ = create_publisher<sensor_msgs::msg::JointState>(output_topic_, 10);

    const auto period = std::chrono::duration<double>(1.0 / publish_rate_);
    timer_ = create_wall_timer(period, std::bind(&StdXelaJointStatePublisher::onTimer, this));

    RCLCPP_INFO(get_logger(), "std_xela_joint_state_publisher started. out: %s joints: %zu",
                output_topic_.c_str(), joint_names_.size());
  }

private:
  void loadJointNames() {
    std::vector<std::string> resolved;

    if (!joint_names_.empty()) {
      append_unique(resolved, joint_names_);
      append_unique(resolved, extra_joints_);
      joint_names_ = resolved;
      return;
    }

    if (config_yaml_.empty()) {
      append_unique(resolved, extra_joints_);
      joint_names_ = resolved;
      return;
    }

    std::string resolved_config = resolve_package_path(config_yaml_);
    std::filesystem::path config_path(resolved_config);
    if (!std::filesystem::exists(config_path)) {
      RCLCPP_WARN(get_logger(), "config_yaml not found: %s", resolved_config.c_str());
      append_unique(resolved, extra_joints_);
      joint_names_ = resolved;
      return;
    }

    YAML::Node root = YAML::LoadFile(resolved_config);
    if (device_profile_.empty() && !device_profiles_.empty()) {
      device_profile_ = device_profiles_.front();
    }
    if (device_profile_.empty()) {
      RCLCPP_WARN(get_logger(), "device_profile is empty; no joints loaded from config.");
      append_unique(resolved, extra_joints_);
      joint_names_ = resolved;
      return;
    }

    YAML::Node profiles = root["device_profiles"];
    if (!profiles || !profiles[device_profile_]) {
      RCLCPP_WARN(get_logger(), "device_profile '%s' not found in config_yaml.",
                  device_profile_.c_str());
      append_unique(resolved, extra_joints_);
      joint_names_ = resolved;
      return;
    }

    YAML::Node profile = profiles[device_profile_];
    auto files = parse_string_list(profile["keep_joints_files"]);
    for (const auto &file : files) {
      std::filesystem::path file_path(file);
      if (!file_path.is_absolute()) {
        file_path = config_path.parent_path() / file_path;
      }
      if (!std::filesystem::exists(file_path)) {
        RCLCPP_WARN(get_logger(), "keep_joints file not found: %s", file_path.string().c_str());
        continue;
      }
      auto joints = load_keep_joints_file(file_path);
      append_unique(resolved, joints);
    }

    append_unique(resolved, extra_joints_);
    joint_names_ = resolved;
  }

  void onTimer() {
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = now();
    msg.name = joint_names_;
    msg.position = positions_;
    msg.velocity = velocities_;
    msg.effort = efforts_;
    pub_->publish(msg);
  }

  std::string output_topic_;
  double publish_rate_{kDefaultPublishRate};
  std::vector<std::string> joint_names_;
  std::vector<std::string> extra_joints_;
  std::vector<std::string> device_profiles_;
  std::string device_profile_;
  std::string config_yaml_;

  std::vector<double> positions_;
  std::vector<double> velocities_;
  std::vector<double> efforts_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StdXelaJointStatePublisher>());
  rclcpp::shutdown();
  return 0;
}
