#include <chrono>
#include <cmath>
#include <fstream>
#include <random>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "boost/asio.hpp"
#include "boost/beast/core.hpp"
#include "boost/beast/websocket.hpp"
#include "nlohmann/json.hpp"
#include "rclcpp/rclcpp.hpp"

namespace {
using tcp = boost::asio::ip::tcp;
namespace websocket = boost::beast::websocket;

struct SensorArrays {
  nlohmann::json *integer = nullptr;
  nlohmann::json *calibrated = nullptr;
  std::vector<double> base_integer;
  std::vector<double> base_calibrated;
  std::vector<double> prev_integer;
  std::vector<double> prev_calibrated;
  bool has_prev_integer = false;
  bool has_prev_calibrated = false;
  bool has_calibrated = false;
};

std::string ReadFile(const std::string &path) {
  std::ifstream in(path);
  if (!in.is_open()) {
    throw std::runtime_error("failed to open file: " + path);
  }
  std::ostringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

std::string SanitizeJsonStringLiterals(const std::string &input) {
  std::string out;
  out.reserve(input.size());
  bool in_string = false;
  bool escape = false;
  for (size_t i = 0; i < input.size(); ++i) {
    char c = input[i];
    if (in_string) {
      if (escape) {
        out.push_back(c);
        escape = false;
        continue;
      }
      if (c == '\\') {
        out.push_back(c);
        escape = true;
        continue;
      }
      if (c == '"') {
        out.push_back(c);
        in_string = false;
        continue;
      }
      if (c == '\n') {
        out.append("\\n");
        continue;
      }
      if (c == '\r') {
        if (i + 1 < input.size() && input[i + 1] == '\n') {
          ++i;
        }
        out.append("\\n");
        continue;
      }
      out.push_back(c);
      continue;
    }

    if (c == '"') {
      out.push_back(c);
      in_string = true;
      continue;
    }
    out.push_back(c);
  }
  return out;
}

bool IsNumericKey(const std::string &key) {
  if (key.empty()) {
    return false;
  }
  for (char c : key) {
    if (!std::isdigit(static_cast<unsigned char>(c))) {
      return false;
    }
  }
  return true;
}

}  // namespace

class WebsocketReplayerServer : public rclcpp::Node {
public:
  WebsocketReplayerServer()
      : Node(
          "sim_xela_server",
          rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)) {
    DeclareIfNot<std::string>("model_name", "");
    DeclareIfNot<std::string>("ref_dir", "");
    DeclareIfNot<std::string>("bind_host", "0.0.0.0");
    DeclareIfNot<int>("bind_port", 5000);
    DeclareIfNot<int>("publish_period_ms", 200);
    DeclareIfNot<double>("warmup_sec", 5.0);
    DeclareIfNot<bool>("use_calibrated_if_present", true);
    DeclareIfNot<double>("integer_xy_range", 1350.0);
    DeclareIfNot<double>("integer_z_range", 10085.0);
    DeclareIfNot<double>("calib_xy_range", 0.8);
    DeclareIfNot<double>("calib_z_range", 14.0);
    DeclareIfNot<double>("z_bias_power", 1.0);
    DeclareIfNot<double>("sine_freq_hz", 0.05);
    DeclareIfNot<double>("sine_phase_step", 0.1);
    DeclareIfNot<std::string>("variation_mode", "random");
    DeclareIfNot<int>("random_seed", 0);
    DeclareIfNot<double>("random_global_strength", 0.3);
    DeclareIfNot<double>("random_local_strength", 0.7);
    DeclareIfNot<double>("random_temporal_alpha", 0.7);
    DeclareIfNot<double>("random_zero_prob", 0.4);
    DeclareIfNot<std::string>("preset", "normal");

    model_name_ = get_parameter("model_name").as_string();
    ref_dir_ = get_parameter("ref_dir").as_string();
    bind_host_ = get_parameter("bind_host").as_string();
    bind_port_ = get_parameter("bind_port").as_int();
    publish_period_ms_ = get_parameter("publish_period_ms").as_int();
    warmup_sec_ = get_parameter("warmup_sec").as_double();
    use_calibrated_if_present_ = get_parameter("use_calibrated_if_present").as_bool();
    integer_xy_range_ = get_parameter("integer_xy_range").as_double();
    integer_z_range_ = get_parameter("integer_z_range").as_double();
    calib_xy_range_ = get_parameter("calib_xy_range").as_double();
    calib_z_range_ = get_parameter("calib_z_range").as_double();
    z_bias_power_ = get_parameter("z_bias_power").as_double();
    sine_freq_hz_ = get_parameter("sine_freq_hz").as_double();
    sine_phase_step_ = get_parameter("sine_phase_step").as_double();
    variation_mode_ = get_parameter("variation_mode").as_string();
    random_seed_ = get_parameter("random_seed").as_int();
    random_global_strength_ = get_parameter("random_global_strength").as_double();
    random_local_strength_ = get_parameter("random_local_strength").as_double();
    random_temporal_alpha_ = get_parameter("random_temporal_alpha").as_double();
    random_zero_prob_ = get_parameter("random_zero_prob").as_double();
    if (random_seed_ != 0) {
      rng_.seed(static_cast<uint32_t>(random_seed_));
    }
    apply_preset();

    if (model_name_.empty()) {
      throw std::runtime_error("model_name parameter is required");
    }

    if (ref_dir_.empty()) {
      ref_dir_ = ament_index_cpp::get_package_share_directory("sim_xela_server") +
                 "/resource";
    }

    LoadReference();

    worker_ = std::thread([this]() { RunServer(); });
    RCLCPP_INFO(get_logger(), "WebSocket replayer server listening on %s:%d", bind_host_.c_str(),
                bind_port_);
  }

  ~WebsocketReplayerServer() override {
    stop_.store(true);
    if (worker_.joinable()) {
      worker_.join();
    }
  }

private:
  template <typename T>
  void DeclareIfNot(const std::string &name, const T &value) {
    if (!has_parameter(name)) {
      declare_parameter<T>(name, value);
    }
  }

  void LoadReference() {
    std::string file = model_name_;
    if (file.size() < 5 || file.substr(file.size() - 5) != ".json") {
      file += ".json";
    }
    std::string path = ref_dir_ + "/" + file;
    base_text_ = ReadFile(path);
    try {
      base_json_ = nlohmann::json::parse(base_text_);
    } catch (const nlohmann::json::parse_error &e) {
      std::string sanitized = SanitizeJsonStringLiterals(base_text_);
      base_json_ = nlohmann::json::parse(sanitized);
      base_text_ = std::move(sanitized);
      RCLCPP_WARN(get_logger(), "Reference JSON had unescaped newlines; sanitized before parsing.");
    }

    sensor_arrays_.clear();
    for (auto it = base_json_.begin(); it != base_json_.end(); ++it) {
      if (!it.value().is_object()) {
        continue;
      }
      if (!IsNumericKey(it.key())) {
        continue;
      }
      auto &sensor = it.value();
      SensorArrays arrays;
      if (sensor.contains("integer") && sensor["integer"].is_array()) {
        arrays.integer = &sensor["integer"];
        arrays.base_integer.reserve(sensor["integer"].size());
        for (const auto &v : sensor["integer"]) {
          arrays.base_integer.push_back(v.get<double>());
        }
      }
      if (sensor.contains("calibrated") && sensor["calibrated"].is_array()) {
        arrays.calibrated = &sensor["calibrated"];
        arrays.base_calibrated.reserve(sensor["calibrated"].size());
        for (const auto &v : sensor["calibrated"]) {
          arrays.base_calibrated.push_back(v.get<double>());
        }
        arrays.has_calibrated = !arrays.base_calibrated.empty();
      }
      sensor_arrays_.push_back(arrays);
    }

    if (sensor_arrays_.empty()) {
      throw std::runtime_error("reference JSON has no numeric sensor objects");
    }
  }

  static double Clamp(double value, double min_v, double max_v) {
    return std::max(min_v, std::min(max_v, value));
  }

  double ApplyZBias(double unit_value) const {
    double power = z_bias_power_;
    if (!std::isfinite(power) || power <= 0.0) {
      power = 1.0;
    }
    double u = std::clamp(unit_value, 0.0, 1.0);
    if (power == 1.0) {
      return u;
    }
    return std::pow(u, power);
  }

  void ApplySineVariation(double elapsed_sec) {
    double omega = 2.0 * M_PI * sine_freq_hz_;
    double t = elapsed_sec;

    for (auto &arrays : sensor_arrays_) {
      bool use_calib = use_calibrated_if_present_ && arrays.has_calibrated;
      if (use_calib && arrays.calibrated) {
        auto &arr = *arrays.calibrated;
        for (size_t i = 0; i < arrays.base_calibrated.size(); ++i) {
          double base = arrays.base_calibrated[i];
          size_t axis = i % 3;
          size_t taxel = i / 3;
          double phase = sine_phase_step_ * static_cast<double>(taxel);
          double s = std::sin(omega * t + phase);
          double value = base;
          if (axis < 2) {
            value = base + calib_xy_range_ * s;
            value = Clamp(value, base - calib_xy_range_, base + calib_xy_range_);
          } else {
            const double u = ApplyZBias(0.5 * (s + 1.0));
            value = base + calib_z_range_ * u;
            value = Clamp(value, base, base + calib_z_range_);
          }
          arr[i] = value;
        }
      } else if (arrays.integer) {
        auto &arr = *arrays.integer;
        for (size_t i = 0; i < arrays.base_integer.size(); ++i) {
          double base = arrays.base_integer[i];
          size_t axis = i % 3;
          size_t taxel = i / 3;
          double phase = sine_phase_step_ * static_cast<double>(taxel);
          double s = std::sin(omega * t + phase);
          double value = base;
          if (axis < 2) {
            value = base + integer_xy_range_ * s;
            value = Clamp(value, base - integer_xy_range_, base + integer_xy_range_);
          } else {
            const double u = ApplyZBias(0.5 * (s + 1.0));
            value = base + integer_z_range_ * u;
            value = Clamp(value, base, base + integer_z_range_);
          }
          arr[i] = static_cast<int>(std::llround(value));
        }
      }
    }
  }

  void ApplyRandomVariation() {
    double global_strength = std::max(0.0, random_global_strength_);
    double local_strength = std::max(0.0, random_local_strength_);
    const double sum_strength = global_strength + local_strength;
    if (sum_strength <= 0.0) {
      return;
    }
    if (sum_strength > 1.0) {
      global_strength /= sum_strength;
      local_strength /= sum_strength;
    }
    double alpha = std::clamp(random_temporal_alpha_, 0.0, 1.0);
    double zero_prob = std::clamp(random_zero_prob_, 0.0, 1.0);
    std::uniform_real_distribution<double> zero_dist(0.0, 1.0);

    for (auto &arrays : sensor_arrays_) {
      bool use_calib = use_calibrated_if_present_ && arrays.has_calibrated;
      if (use_calib && arrays.calibrated) {
        auto &arr = *arrays.calibrated;
        const double gxy = calib_xy_range_ * global_strength;
        const double lxy = calib_xy_range_ * local_strength;
        const double gz = calib_z_range_ * global_strength;
        const double lz = calib_z_range_ * local_strength;
        std::uniform_real_distribution<double> gxy_dist(-gxy, gxy);
        std::uniform_real_distribution<double> lxy_dist(-lxy, lxy);
        std::uniform_real_distribution<double> unit_dist(0.0, 1.0);
        const double global_x = gxy_dist(rng_);
        const double global_y = gxy_dist(rng_);
        const double global_z = ApplyZBias(unit_dist(rng_)) * gz;
        for (size_t i = 0; i < arrays.base_calibrated.size(); ++i) {
          double base = arrays.base_calibrated[i];
          size_t axis = i % 3;
          double value = base;
          if (axis < 2) {
            const double local = lxy_dist(rng_);
            value = base + ((axis == 0) ? global_x : global_y) + local;
            value = Clamp(value, base - calib_xy_range_, base + calib_xy_range_);
          } else {
            const double local = ApplyZBias(unit_dist(rng_)) * lz;
            value = base + global_z + local;
            value = Clamp(value, base, base + calib_z_range_);
          }
          if (zero_dist(rng_) < zero_prob) {
            value = base;
          }
          if (alpha > 0.0) {
            if (!arrays.has_prev_calibrated || arrays.prev_calibrated.size() != arrays.base_calibrated.size()) {
              arrays.prev_calibrated.assign(arrays.base_calibrated.size(), value);
              arrays.has_prev_calibrated = true;
            } else {
              value = alpha * arrays.prev_calibrated[i] + (1.0 - alpha) * value;
              arrays.prev_calibrated[i] = value;
            }
          }
          arr[i] = value;
        }
      } else if (arrays.integer) {
        auto &arr = *arrays.integer;
        const double gxy = integer_xy_range_ * global_strength;
        const double lxy = integer_xy_range_ * local_strength;
        const double gz = integer_z_range_ * global_strength;
        const double lz = integer_z_range_ * local_strength;
        std::uniform_real_distribution<double> gxy_dist(-gxy, gxy);
        std::uniform_real_distribution<double> lxy_dist(-lxy, lxy);
        std::uniform_real_distribution<double> unit_dist(0.0, 1.0);
        const double global_x = gxy_dist(rng_);
        const double global_y = gxy_dist(rng_);
        const double global_z = ApplyZBias(unit_dist(rng_)) * gz;
        for (size_t i = 0; i < arrays.base_integer.size(); ++i) {
          double base = arrays.base_integer[i];
          size_t axis = i % 3;
          double value = base;
          if (axis < 2) {
            const double local = lxy_dist(rng_);
            value = base + ((axis == 0) ? global_x : global_y) + local;
            value = Clamp(value, base - integer_xy_range_, base + integer_xy_range_);
          } else {
            const double local = ApplyZBias(unit_dist(rng_)) * lz;
            value = base + global_z + local;
            value = Clamp(value, base, base + integer_z_range_);
          }
          if (zero_dist(rng_) < zero_prob) {
            value = base;
          }
          if (alpha > 0.0) {
            if (!arrays.has_prev_integer || arrays.prev_integer.size() != arrays.base_integer.size()) {
              arrays.prev_integer.assign(arrays.base_integer.size(), value);
              arrays.has_prev_integer = true;
            } else {
              value = alpha * arrays.prev_integer[i] + (1.0 - alpha) * value;
              arrays.prev_integer[i] = value;
            }
          }
          arr[i] = static_cast<int>(std::llround(value));
        }
      }
    }
  }

  std::string BuildMessage(const rclcpp::Time &start_time) {
    if (warmup_sec_ <= 0.0) {
      ApplyVariation((now() - start_time).seconds());
      return base_json_.dump();
    }

    double elapsed = (now() - start_time).seconds();
    if (elapsed >= warmup_sec_) {
      ApplyVariation(elapsed - warmup_sec_);
    }
    return base_json_.dump();
  }

  void ApplyVariation(double elapsed_sec) {
    if (variation_mode_ == "sine") {
      ApplySineVariation(elapsed_sec);
    } else {
      ApplyRandomVariation();
    }
  }

  void RunServer() {
    try {
      boost::asio::io_context ioc(1);
      tcp::endpoint endpoint;
      if (bind_host_.empty() || bind_host_ == "0.0.0.0") {
        endpoint = tcp::endpoint(tcp::v4(), static_cast<unsigned short>(bind_port_));
      } else {
        endpoint = tcp::endpoint(boost::asio::ip::make_address(bind_host_),
                                 static_cast<unsigned short>(bind_port_));
      }

      tcp::acceptor acceptor(ioc);
      acceptor.open(endpoint.protocol());
      acceptor.set_option(boost::asio::socket_base::reuse_address(true));
      acceptor.bind(endpoint);
      acceptor.listen();
      acceptor.non_blocking(true);

      std::vector<std::shared_ptr<websocket::stream<tcp::socket>>> clients;
      rclcpp::Time start_time = now();
      auto period = std::chrono::milliseconds(std::max(1, publish_period_ms_));
      auto next_send = std::chrono::steady_clock::now();

      while (rclcpp::ok() && !stop_.load()) {
        boost::system::error_code accept_ec;
        tcp::socket socket(ioc);
        acceptor.accept(socket, accept_ec);
        if (!accept_ec) {
          auto ws = std::make_shared<websocket::stream<tcp::socket>>(std::move(socket));
          ws->set_option(websocket::stream_base::timeout::suggested(boost::beast::role_type::server));
          boost::beast::error_code ws_ec;
          ws->accept(ws_ec);
          if (ws_ec) {
            RCLCPP_WARN(get_logger(), "WebSocket accept error: %s", ws_ec.message().c_str());
          } else {
            clients.push_back(ws);
            RCLCPP_INFO(get_logger(), "Client connected. total=%zu", clients.size());
          }
        } else if (accept_ec != boost::asio::error::would_block &&
                   accept_ec != boost::asio::error::try_again) {
          RCLCPP_WARN(get_logger(), "Accept error: %s", accept_ec.message().c_str());
        }

        auto now_steady = std::chrono::steady_clock::now();
        if (now_steady >= next_send) {
          if (!clients.empty()) {
            std::string payload = BuildMessage(start_time);
            for (size_t i = 0; i < clients.size();) {
              boost::beast::error_code ec;
              clients[i]->write(boost::asio::buffer(payload), ec);
              if (ec) {
                boost::beast::error_code close_ec;
                clients[i]->close(websocket::close_code::normal, close_ec);
                RCLCPP_WARN(get_logger(), "WebSocket write error: %s", ec.message().c_str());
                clients.erase(clients.begin() + static_cast<long>(i));
                RCLCPP_INFO(get_logger(), "Client disconnected. total=%zu", clients.size());
                continue;
              }
              ++i;
            }
          }
          next_send = now_steady + period;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    } catch (const std::exception &e) {
      RCLCPP_ERROR(get_logger(), "WebSocket server error: %s", e.what());
    }
  }

  void apply_preset() {
    const auto preset = get_parameter("preset").as_string();
    if (preset.empty() || preset == "custom") {
      return;
    }

    const std::string base = "replayer_presets." + preset + ".";
    const std::string period_key = base + "publish_period_ms";
    const std::string freq_key = base + "sine_freq_hz";
    const std::string zero_key = base + "random_zero_prob";
    const std::string z_bias_key = base + "z_bias_power";
    const std::string integer_z_key = base + "integer_z_range";
    const std::string calib_z_key = base + "calib_z_range";

    bool applied = false;
    if (has_parameter(period_key)) {
      publish_period_ms_ = get_parameter(period_key).as_int();
      applied = true;
    }
    if (has_parameter(freq_key)) {
      sine_freq_hz_ = get_parameter(freq_key).as_double();
      applied = true;
    }
    if (has_parameter(zero_key)) {
      random_zero_prob_ = get_parameter(zero_key).as_double();
      applied = true;
    }
    if (has_parameter(z_bias_key)) {
      z_bias_power_ = get_parameter(z_bias_key).as_double();
      applied = true;
    }
    if (has_parameter(integer_z_key)) {
      integer_z_range_ = get_parameter(integer_z_key).as_double();
      applied = true;
    }
    if (has_parameter(calib_z_key)) {
      calib_z_range_ = get_parameter(calib_z_key).as_double();
      applied = true;
    }
    if (applied) {
      RCLCPP_INFO(get_logger(),
                  "Applied replayer preset '%s' (publish_period_ms=%d, sine_freq_hz=%.3f, z_bias_power=%.2f)",
                  preset.c_str(), publish_period_ms_, sine_freq_hz_, z_bias_power_);
    } else {
      RCLCPP_WARN(get_logger(),
                  "Preset '%s' not found in replayer_presets; using explicit parameters.",
                  preset.c_str());
    }
  }

  std::string model_name_;
  std::string ref_dir_;
  std::string bind_host_;
  int bind_port_{5000};
  int publish_period_ms_{200};
  double warmup_sec_{5.0};
  bool use_calibrated_if_present_{true};
  double integer_xy_range_{1350.0};
  double integer_z_range_{10085.0};
  double calib_xy_range_{0.8};
  double calib_z_range_{14.0};
  double z_bias_power_{1.0};
  double sine_freq_hz_{0.05};
  double sine_phase_step_{0.1};
  std::string variation_mode_{"random"};
  int random_seed_{0};
  double random_global_strength_{0.3};
  double random_local_strength_{0.7};
  double random_temporal_alpha_{0.7};
  double random_zero_prob_{0.4};
  std::mt19937 rng_{std::random_device{}()};

  std::atomic<bool> stop_{false};
  std::thread worker_;

  std::string base_text_;
  nlohmann::json base_json_;
  std::vector<SensorArrays> sensor_arrays_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<WebsocketReplayerServer>();
    rclcpp::spin(node);
  } catch (const std::exception &e) {
    fprintf(stderr, "Fatal error: %s\n", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
