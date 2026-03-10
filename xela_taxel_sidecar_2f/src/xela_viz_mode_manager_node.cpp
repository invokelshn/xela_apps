#include <chrono>
#include <cctype>
#include <mutex>
#include <string>
#include <memory>

#include <rcl_interfaces/msg/parameter.hpp>
#include <rcl_interfaces/msg/parameter_type.hpp>
#include <rcl_interfaces/msg/parameter_value.hpp>
#include <rcl_interfaces/srv/set_parameters.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>

namespace {

std::string normalize_mode(std::string mode)
{
  for (auto & c : mode) {
    c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  }
  return (mode == "urdf") ? "urdf" : "grid";
}

std::string normalize_node_name(std::string name)
{
  if (name.empty()) {
    return "/";
  }
  if (name.front() != '/') {
    name = "/" + name;
  }
  return name;
}

}  // namespace

class XelaVizModeManagerNode : public rclcpp::Node
{
public:
  XelaVizModeManagerNode()
  : Node("xela_viz_mode_manager_cpp")
  {
    // Keep legacy parameter declarations for launch-file compatibility.
    this->declare_parameter<std::string>("initial_viz_mode", "grid");
    this->declare_parameter<std::string>("model_name", "uSPr2F");
    this->declare_parameter<std::string>("style_preset", "cool_steel");
    this->declare_parameter<std::string>("frame_id", "x_taxel_viz");
    this->declare_parameter<std::string>("grid_parent_frame", "world");
    this->declare_parameter<std::string>("legacy_out_topic", "/x_taxel_2f/markers");
    this->declare_parameter<double>("max_publish_rate_hz", 20.0);
    this->declare_parameter<bool>("publisher_transient_local", false);
    this->declare_parameter<std::string>("marker_stamp_mode", "now");
    this->declare_parameter<std::string>("circle_marker_type", "sphere");
    this->declare_parameter<std::string>("managed_launch_package", "ur5e_x2f_140_config2");
    this->declare_parameter<std::string>("managed_launch_file", "xela_viz_safe.launch.py");
    this->declare_parameter<std::string>("bridge_node_name", "/xela_taxel_web_bridge_cpp");
    this->declare_parameter<std::string>("viz_node_name", "/xela_taxel_viz_2f");

    current_mode_ = normalize_mode(this->get_parameter("initial_viz_mode").as_string());
    bridge_node_name_ = normalize_node_name(this->get_parameter("bridge_node_name").as_string());
    viz_node_name_ = normalize_node_name(this->get_parameter("viz_node_name").as_string());

    bridge_set_params_client_ = this->create_client<rcl_interfaces::srv::SetParameters>(
      bridge_node_name_ + "/set_parameters");
    viz_set_mode_client_ = this->create_client<std_srvs::srv::SetBool>(
      viz_node_name_ + "/set_mode");
    set_mode_service_ = this->create_service<std_srvs::srv::SetBool>(
      "xela_viz_mode_manager/set_mode",
      std::bind(&XelaVizModeManagerNode::on_set_mode, this, std::placeholders::_1, std::placeholders::_2));

    startup_sync_timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&XelaVizModeManagerNode::sync_initial_mode, this));

    RCLCPP_INFO(
      this->get_logger(),
      "xela_viz_mode_manager_cpp started. mode=%s bridge=%s viz=%s service=/xela_viz_mode_manager/set_mode",
      current_mode_.c_str(),
      bridge_node_name_.c_str(),
      viz_node_name_.c_str());
  }

private:
  bool set_bridge_mode(const std::string & mode)
  {
    if (!bridge_set_params_client_->wait_for_service(std::chrono::milliseconds(800))) {
      RCLCPP_WARN(this->get_logger(), "Bridge set_parameters service unavailable: %s",
                  (bridge_node_name_ + "/set_parameters").c_str());
      return false;
    }

    auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
    request->parameters.reserve(3);

    rcl_interfaces::msg::Parameter mode_param;
    mode_param.name = "viz_mode";
    mode_param.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    mode_param.value.string_value = mode;
    request->parameters.push_back(mode_param);

    const bool emit_urdf_points = mode == "urdf";
    rcl_interfaces::msg::Parameter emit_param;
    emit_param.name = "emit_urdf_points";
    emit_param.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
    emit_param.value.bool_value = emit_urdf_points;
    request->parameters.push_back(emit_param);

    rcl_interfaces::msg::Parameter freeze_param;
    freeze_param.name = "freeze_urdf_positions";
    freeze_param.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
    freeze_param.value.bool_value = emit_urdf_points;
    request->parameters.push_back(freeze_param);

    (void)bridge_set_params_client_->async_send_request(request);
    return true;
  }

  bool set_viz_mode(const std::string & mode)
  {
    if (!viz_set_mode_client_->wait_for_service(std::chrono::milliseconds(800))) {
      RCLCPP_WARN(this->get_logger(), "Viz mode service unavailable: %s",
                  (viz_node_name_ + "/set_mode").c_str());
      return false;
    }

    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = (mode == "urdf");

    (void)viz_set_mode_client_->async_send_request(
      request,
      [logger = this->get_logger(), mode](
        rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future) {
        try {
          const auto response = future.get();
          if (!response->success) {
            RCLCPP_WARN(
              logger, "xela_taxel_viz_2f rejected mode '%s': %s",
              mode.c_str(), response->message.c_str());
          }
        } catch (const std::exception & ex) {
          RCLCPP_WARN(logger, "xela_taxel_viz_2f mode request failed: %s", ex.what());
        }
      });
    return true;
  }

  void sync_initial_mode()
  {
    if (startup_sync_done_) {
      return;
    }

    const bool bridge_ok = set_bridge_mode(current_mode_);
    const bool viz_ok = set_viz_mode(current_mode_);
    if (!bridge_ok || !viz_ok) {
      return;
    }

    startup_sync_done_ = true;
    startup_sync_timer_->cancel();
    RCLCPP_INFO(this->get_logger(), "Initial mode synced: %s", current_mode_.c_str());
  }

  void on_set_mode(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
  {
    const std::string target_mode = request->data ? "urdf" : "grid";
    std::lock_guard<std::mutex> lock(mutex_);

    if (target_mode == current_mode_) {
      set_bridge_mode(target_mode);
      response->success = true;
      response->message = "Already in " + target_mode + " mode";
      return;
    }

    RCLCPP_INFO(
      this->get_logger(), "Switch viz mode: %s -> %s", current_mode_.c_str(), target_mode.c_str());

    const bool bridge_ok = set_bridge_mode(target_mode);
    const bool viz_ok = set_viz_mode(target_mode);
    if (!bridge_ok || !viz_ok) {
      response->success = false;
      response->message = "Failed to send mode request to bridge or viz node";
      return;
    }

    current_mode_ = target_mode;
    response->success = true;
    response->message = "Requested switch to " + target_mode;
  }

  std::mutex mutex_;
  std::string current_mode_;
  std::string bridge_node_name_;
  std::string viz_node_name_;
  bool startup_sync_done_{false};

  rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr bridge_set_params_client_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr viz_set_mode_client_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_mode_service_;
  rclcpp::TimerBase::SharedPtr startup_sync_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<XelaVizModeManagerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
