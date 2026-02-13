#include <atomic>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "boost/asio/ip/tcp.hpp"
#include "boost/beast/core.hpp"
#include "boost/beast/websocket.hpp"
#include "rclcpp/rclcpp.hpp"
#include "xela_taxel_msgs/msg/x_taxel_sensor_t_array.hpp"

#include "xela_server2_2f/frame_ids.hpp"
#include "xela_server2_2f/parser.hpp"

namespace xela_server2_2f {
namespace {

class LatestMessageQueue {
 public:
  void Push(std::string msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_ = std::move(msg);
  }

  std::optional<std::string> Pop() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!latest_.has_value()) {
      return std::nullopt;
    }
    std::optional<std::string> out = std::move(latest_);
    latest_.reset();
    return out;
  }

 private:
  std::mutex mutex_;
  std::optional<std::string> latest_;
};

std::string StripWsScheme(const std::string &host) {
  const std::string ws_prefix = "ws://";
  const std::string wss_prefix = "wss://";
  if (host.rfind(ws_prefix, 0) == 0) {
    return host.substr(ws_prefix.size());
  }
  if (host.rfind(wss_prefix, 0) == 0) {
    return host.substr(wss_prefix.size());
  }
  return host;
}

bool ReadFileToString(const std::string &path, std::string &out, std::string &error) {
  std::ifstream file(path);
  if (!file.is_open()) {
    error = "failed to open file";
    return false;
  }
  std::ostringstream buffer;
  buffer << file.rdbuf();
  out = buffer.str();
  return true;
}

bool ExtractJsonPayload(const std::string &input, std::string &payload, std::string &error) {
  size_t start = input.find('{');
  size_t end = input.rfind('}');
  if (start == std::string::npos || end == std::string::npos || end <= start) {
    error = "failed to locate JSON object in file";
    return false;
  }
  payload = input.substr(start, end - start + 1);
  return true;
}

}  // namespace

class XelaServer2Node : public rclcpp::Node {
 public:
  XelaServer2Node() : Node("xela_server2_2f"), running_(true) {
    std::string default_frame_ids;
    try {
      default_frame_ids = ament_index_cpp::get_package_share_directory("xela_server2_2f");
      default_frame_ids += "/config/server2_2f_config.yaml";
    } catch (const std::exception &e) {
      RCLCPP_WARN(get_logger(), "Failed to locate xela_server2_2f share: %s", e.what());
      std::filesystem::path source_root = std::filesystem::path(__FILE__).parent_path().parent_path();
      default_frame_ids = (source_root / "config" / "server2_2f_config.yaml").string();
    }

    ws_host_ = declare_parameter<std::string>("ws_host", "localhost");
    ws_port_ = declare_parameter<int>("ws_port", 5000);
    frame_ids_yaml_ = declare_parameter<std::string>("frame_ids_yaml", default_frame_ids);
    header_frame_id_ = declare_parameter<std::string>("header_frame_id", "");
    use_ros_time_for_sensor_time_ =
        declare_parameter<bool>("use_ros_time_for_sensor_time", false);
    publisher_qos_depth_ = declare_parameter<int>("publisher_qos_depth", 10);
    input_json_path_ = declare_parameter<std::string>("input_json_path", "");
    playback_interval_ms_ = declare_parameter<int>("playback_interval_ms", 100);
    playback_loop_ = declare_parameter<bool>("playback_loop", true);

    std::string frame_error;
    if (frame_ids_yaml_.empty()) {
      RCLCPP_ERROR(get_logger(), "frame_ids_yaml parameter is empty");
    } else {
      frame_config_ = LoadFrameConfig(frame_ids_yaml_, frame_error);
      if (!frame_error.empty()) {
        RCLCPP_ERROR(get_logger(), "Failed to load frame config: %s", frame_error.c_str());
      } else {
        RCLCPP_INFO(get_logger(), "Loaded %zu model frame configs", frame_config_.models.size());
      }
    }

    rclcpp::QoS qos{rclcpp::KeepLast(publisher_qos_depth_)};
    publisher_ = create_publisher<xela_taxel_msgs::msg::XTaxelSensorTArray>(
        "/x_taxel_2f", qos);

    parse_config_.use_ros_time_for_sensor_time = use_ros_time_for_sensor_time_;
    parse_config_.header_frame_id = header_frame_id_;

    timer_ = create_wall_timer(std::chrono::milliseconds(10),
                               std::bind(&XelaServer2Node::OnTimer, this));

    if (input_json_path_.empty()) {
      ws_thread_ = std::thread(&XelaServer2Node::WebsocketLoop, this);
      ws_thread_started_ = true;
    } else {
      std::string file_error;
      std::string file_content;
      if (!ReadFileToString(input_json_path_, file_content, file_error)) {
        RCLCPP_ERROR(get_logger(), "Failed to read input_json_path: %s (%s)",
                     input_json_path_.c_str(), file_error.c_str());
      } else if (!ExtractJsonPayload(file_content, playback_payload_, file_error)) {
        RCLCPP_ERROR(get_logger(), "Failed to parse input_json_path: %s (%s)",
                     input_json_path_.c_str(), file_error.c_str());
      } else {
        playback_timer_ = create_wall_timer(
            std::chrono::milliseconds(playback_interval_ms_),
            std::bind(&XelaServer2Node::OnPlaybackTimer, this));
        RCLCPP_INFO(get_logger(), "Playback enabled from file: %s",
                    input_json_path_.c_str());
      }
    }
  }

  ~XelaServer2Node() override {
    running_.store(false);
    if (ws_thread_started_ && ws_thread_.joinable()) {
      ws_thread_.join();
    }
  }

 private:
  void OnTimer() {
    auto msg_opt = queue_.Pop();
    if (!msg_opt.has_value()) {
      return;
    }

    xela_taxel_msgs::msg::XTaxelSensorTArray out_msg;
    std::string error;
    auto status = ParseJsonToMessage(*msg_opt, frame_config_, now(), parse_config_, out_msg, error);
    if (status == ParseStatus::kOk || status == ParseStatus::kWarn) {
      if (status == ParseStatus::kWarn && !error.empty()) {
        RCLCPP_WARN(get_logger(), "Publish with warnings: %s", error.c_str());
      }
      publisher_->publish(out_msg);
      return;
    }
    if ((status == ParseStatus::kIgnore || status == ParseStatus::kError) && !error.empty()) {
      RCLCPP_WARN(get_logger(), "Drop message: %s", error.c_str());
    }
  }

  void WebsocketLoop() {
    using boost::asio::ip::tcp;
    namespace websocket = boost::beast::websocket;
    const std::string host = StripWsScheme(ws_host_);
    const std::string port = std::to_string(ws_port_);
    const std::string host_header = host + ":" + port;

    while (running_.load()) {
      try {
        boost::asio::io_context ioc;
        tcp::resolver resolver(ioc);
        auto const results = resolver.resolve(host, port);

        boost::beast::tcp_stream stream(ioc);
        stream.expires_after(std::chrono::seconds(5));
        stream.connect(results);

        websocket::stream<boost::beast::tcp_stream> ws(std::move(stream));
        ws.set_option(websocket::stream_base::timeout::suggested(boost::beast::role_type::client));
        ws.handshake(host_header, "/");
        RCLCPP_INFO(get_logger(), "WebSocket connected to ws://%s:%s", host.c_str(), port.c_str());

        while (running_.load()) {
          boost::beast::flat_buffer buffer;
          boost::beast::get_lowest_layer(ws).expires_after(std::chrono::seconds(1));
          boost::beast::error_code ec;
          ws.read(buffer, ec);
          if (ec == boost::beast::error::timeout || ec == boost::asio::error::timed_out) {
            continue;
          }
          if (ec) {
            throw boost::beast::system_error(ec);
          }
          queue_.Push(boost::beast::buffers_to_string(buffer.data()));
        }

        boost::beast::error_code close_ec;
        ws.close(websocket::close_code::normal, close_ec);
      } catch (const std::exception &e) {
        if (running_.load()) {
          RCLCPP_ERROR(get_logger(), "WebSocket error: %s", e.what());
          std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
      }
    }
  }

  void OnPlaybackTimer() {
    if (playback_payload_.empty()) {
      return;
    }
    queue_.Push(playback_payload_);
    if (!playback_loop_ && playback_timer_) {
      playback_timer_->cancel();
    }
  }

  std::string ws_host_;
  int ws_port_ = 0;
  std::string frame_ids_yaml_;
  std::string header_frame_id_;
  bool use_ros_time_for_sensor_time_ = false;
  int publisher_qos_depth_ = 10;
  std::string input_json_path_;
  int playback_interval_ms_ = 100;
  bool playback_loop_ = true;
  std::string playback_payload_;
  bool ws_thread_started_ = false;

  FrameConfig frame_config_;
  ParseConfig parse_config_;

  LatestMessageQueue queue_;
  rclcpp::Publisher<xela_taxel_msgs::msg::XTaxelSensorTArray>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr playback_timer_;

  std::atomic<bool> running_;
  std::thread ws_thread_;
};

}  // namespace xela_server2_2f

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<xela_server2_2f::XelaServer2Node>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
