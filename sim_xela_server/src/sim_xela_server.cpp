#include <chrono>
#include <cmath>
#include <fstream>
#include <mutex>
#include <optional>
#include <random>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_set>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "boost/asio.hpp"
#include "boost/beast/core.hpp"
#include "boost/beast/websocket.hpp"
#include "nlohmann/json.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace {
using tcp = boost::asio::ip::tcp;
namespace websocket = boost::beast::websocket;

// ──────────────────────────────────────────────────────────────
// 2F gripper taxel layout: 4 rows × 6 cols = 24 taxels / module
// flat_index = row * 6 + col
// Array stride: [x0, y0, z0, x1, y1, z1, ...]  z at flat_index*3+2
// ──────────────────────────────────────────────────────────────
static constexpr int kGrid2FCols = 6;

// Contact taxel masks per grasp type (applied identically to both modules
// to simulate symmetric grip — both finger pads face each other and press
// the same relative region of the object).
std::vector<size_t> Get2FContactTaxels_Sharp() {
  // 뾰족한 물체: small center contact (2×2 inner patch)
  // rows 1-2, cols 2-3
  return {
    1 * kGrid2FCols + 2, 1 * kGrid2FCols + 3,
    2 * kGrid2FCols + 2, 2 * kGrid2FCols + 3,
  };
}

std::vector<size_t> Get2FContactTaxels_Medium() {
  // 약간 두툼한 물체: medium contact (2×4 center band)
  // rows 1-2, cols 1-4
  return {
    1 * kGrid2FCols + 1, 1 * kGrid2FCols + 2, 1 * kGrid2FCols + 3, 1 * kGrid2FCols + 4,
    2 * kGrid2FCols + 1, 2 * kGrid2FCols + 2, 2 * kGrid2FCols + 3, 2 * kGrid2FCols + 4,
  };
}

std::vector<size_t> Get2FContactTaxels_Wide() {
  // 넓은 물체: full pad contact (all 24 taxels)
  std::vector<size_t> t;
  t.reserve(24);
  for (size_t i = 0; i < 24; ++i) {
    t.push_back(i);
  }
  return t;
}

// Grasp contact profile identifier
enum class GraspType { NONE = 0, SHARP = 1, MEDIUM = 2, WIDE = 3 };

// MockState: logical state for display / logging
enum class MockState { GEN = -1, CLEAR = 0, PICK = 1, SLIP = 2, PLACE = 3 };

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
      if (escape) { out.push_back(c); escape = false; continue; }
      if (c == '\\') { out.push_back(c); escape = true; continue; }
      if (c == '"')  { out.push_back(c); in_string = false; continue; }
      if (c == '\n') { out.append("\\n"); continue; }
      if (c == '\r') {
        if (i + 1 < input.size() && input[i + 1] == '\n') { ++i; }
        out.append("\\n"); continue;
      }
      out.push_back(c); continue;
    }
    if (c == '"') { out.push_back(c); in_string = true; continue; }
    out.push_back(c);
  }
  return out;
}

bool IsNumericKey(const std::string &key) {
  if (key.empty()) return false;
  for (char c : key) {
    if (!std::isdigit(static_cast<unsigned char>(c))) return false;
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
    if (random_seed_ != 0) rng_.seed(static_cast<uint32_t>(random_seed_));
    apply_preset();

    if (model_name_.empty()) throw std::runtime_error("model_name parameter is required");
    if (ref_dir_.empty()) {
      ref_dir_ = ament_index_cpp::get_package_share_directory("sim_xela_server") + "/resource";
    }

    LoadReference();

    // ── Register 2F scenario services ──────────────────────────
    // GEN  : return to normal random variation
    // CLEAR: silent / no-contact baseline
    // PICK : grasp contact ramp-up  (3 grasp profiles)
    // PLACE: grasp contact ramp-down (uses last active grasp profile)
    // SLIP : (future) slip oscillation
    using Trigger = std_srvs::srv::Trigger;
    auto make_srv = [this](const std::string &name, auto handler) {
      return this->create_service<Trigger>(
        name, std::bind(handler, this, std::placeholders::_1, std::placeholders::_2));
    };
    srv_gen_   = make_srv("~/scenario_gen",          &WebsocketReplayerServer::HandleScenarioGen);
    srv_clear_ = make_srv("~/scenario_clear",        &WebsocketReplayerServer::HandleScenarioClear);
    srv_pick_sharp_  = make_srv("~/scenario_pick_sharp",
                                &WebsocketReplayerServer::HandleScenarioPickSharp);
    srv_pick_medium_ = make_srv("~/scenario_pick_medium",
                                &WebsocketReplayerServer::HandleScenarioPickMedium);
    srv_pick_wide_   = make_srv("~/scenario_pick_wide",
                                &WebsocketReplayerServer::HandleScenarioPickWide);
    srv_place_ = make_srv("~/scenario_place",        &WebsocketReplayerServer::HandleScenarioPlace);
    srv_slip_  = make_srv("~/scenario_slip",         &WebsocketReplayerServer::HandleScenarioSlip);

    variation_start_time_ = now();
    worker_ = std::thread([this]() { RunServer(); });
    RCLCPP_INFO(get_logger(), "sim_xela_server listening on %s:%d  (services: gen/clear/pick_*/place/slip)",
                bind_host_.c_str(), bind_port_);
  }

  ~WebsocketReplayerServer() override {
    stop_.store(true);
    if (worker_.joinable()) worker_.join();
  }

private:
  // ── Helpers ────────────────────────────────────────────────

  template <typename T>
  void DeclareIfNot(const std::string &name, const T &value) {
    if (!has_parameter(name)) declare_parameter<T>(name, value);
  }

  static double Clamp(double value, double min_v, double max_v) {
    return std::max(min_v, std::min(max_v, value));
  }

  double ApplyZBias(double unit_value) const {
    double power = z_bias_power_;
    if (!std::isfinite(power) || power <= 0.0) power = 1.0;
    double u = std::clamp(unit_value, 0.0, 1.0);
    return (power == 1.0) ? u : std::pow(u, power);
  }

  std::string CurrentVariationMode() {
    std::lock_guard<std::mutex> lock(variation_mutex_);
    return variation_mode_;
  }

  double VariationElapsedSeconds() {
    std::lock_guard<std::mutex> lock(variation_mutex_);
    return (now() - variation_start_time_).seconds();
  }

  // Reset elapsed timer and update mode.
  void SelectVariationMode(
      const std::string &mode,
      const std::string &label,
      const std::shared_ptr<std_srvs::srv::Trigger::Response> &response,
      bool instant = false) {
    {
      std::lock_guard<std::mutex> lock(variation_mutex_);
      variation_mode_ = mode;
      // instant=true: set start_time 10 s in the past so factor = clamp(10/3, 0, 1) = 1.0
      // immediately. Used for PICK transitions so the contact pattern is visible at once.
      variation_start_time_ = instant
        ? (now() - rclcpp::Duration::from_seconds(10.0))
        : now();
    }
    RCLCPP_INFO(get_logger(), "Scenario selected: %s%s",
                label.c_str(), instant ? " [instant]" : "");
    response->success = true;
    response->message = label;
  }

  // Restore all sensor arrays to base reference values (for CLEAR / PLACE)
  void RestoreBaseArrays() {
    for (auto &arrays : sensor_arrays_) {
      if (arrays.calibrated) {
        auto &arr = *arrays.calibrated;
        for (size_t i = 0; i < arrays.base_calibrated.size(); ++i) {
          arr[i] = arrays.base_calibrated[i];
        }
      }
      if (arrays.integer) {
        auto &arr = *arrays.integer;
        for (size_t i = 0; i < arrays.base_integer.size(); ++i) {
          arr[i] = static_cast<int>(std::llround(arrays.base_integer[i]));
        }
      }
      arrays.has_prev_calibrated = false;
      arrays.has_prev_integer = false;
    }
  }

  // Apply Z-force ramp on specified contact taxels (symmetric across all modules).
  // factor: 0.0 = baseline, 1.0 = full z_range
  void ApplyContactForce(const std::vector<size_t> &contact_taxels, double factor) {
    if (factor <= 0.0) return;
    for (auto &arrays : sensor_arrays_) {
      if (arrays.has_calibrated && arrays.calibrated) {
        auto &arr = *arrays.calibrated;
        for (size_t t : contact_taxels) {
          const size_t z_idx = t * 3 + 2;
          if (z_idx >= arr.size()) continue;
          const double base = arrays.base_calibrated[z_idx];
          arr[z_idx] = Clamp(base + calib_z_range_ * factor, base, base + calib_z_range_);
        }
      } else if (arrays.integer) {
        auto &arr = *arrays.integer;
        for (size_t t : contact_taxels) {
          const size_t z_idx = t * 3 + 2;
          if (z_idx >= arr.size()) continue;
          const double base = arrays.base_integer[z_idx];
          arr[z_idx] = static_cast<int>(std::llround(
            Clamp(base + integer_z_range_ * factor, base, base + integer_z_range_)));
        }
      }
    }
  }

  std::vector<size_t> ContactTaxelsForCurrentGrasp() const {
    switch (current_grasp_type_) {
      case GraspType::SHARP:  return Get2FContactTaxels_Sharp();
      case GraspType::MEDIUM: return Get2FContactTaxels_Medium();
      case GraspType::WIDE:   return Get2FContactTaxels_Wide();
      default:                return Get2FContactTaxels_Wide();
    }
  }

  // ── Variation modes ────────────────────────────────────────

  void LoadReference() {
    std::string file = model_name_;
    if (file.size() < 5 || file.substr(file.size() - 5) != ".json") file += ".json";
    std::string path = ref_dir_ + "/" + file;
    base_text_ = ReadFile(path);
    try {
      base_json_ = nlohmann::json::parse(base_text_);
    } catch (const nlohmann::json::parse_error &) {
      std::string sanitized = SanitizeJsonStringLiterals(base_text_);
      base_json_ = nlohmann::json::parse(sanitized);
      base_text_ = std::move(sanitized);
      RCLCPP_WARN(get_logger(), "Reference JSON had unescaped newlines; sanitized.");
    }

    sensor_arrays_.clear();
    for (auto it = base_json_.begin(); it != base_json_.end(); ++it) {
      if (!it.value().is_object() || !IsNumericKey(it.key())) continue;
      auto &sensor = it.value();
      SensorArrays arrays;
      if (sensor.contains("integer") && sensor["integer"].is_array()) {
        arrays.integer = &sensor["integer"];
        for (const auto &v : sensor["integer"]) arrays.base_integer.push_back(v.get<double>());
      }
      if (sensor.contains("calibrated") && sensor["calibrated"].is_array()) {
        arrays.calibrated = &sensor["calibrated"];
        for (const auto &v : sensor["calibrated"]) arrays.base_calibrated.push_back(v.get<double>());
        arrays.has_calibrated = !arrays.base_calibrated.empty();
      }
      sensor_arrays_.push_back(arrays);
    }
    if (sensor_arrays_.empty()) throw std::runtime_error("reference JSON has no numeric sensor objects");
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
          double phase = sine_phase_step_ * static_cast<double>(i / 3);
          double s = std::sin(omega * t + phase);
          double value = base;
          if (axis < 2) {
            value = Clamp(base + calib_xy_range_ * s, base - calib_xy_range_, base + calib_xy_range_);
          } else {
            value = Clamp(base + calib_z_range_ * ApplyZBias(0.5 * (s + 1.0)), base, base + calib_z_range_);
          }
          arr[i] = value;
        }
      } else if (arrays.integer) {
        auto &arr = *arrays.integer;
        for (size_t i = 0; i < arrays.base_integer.size(); ++i) {
          double base = arrays.base_integer[i];
          size_t axis = i % 3;
          double phase = sine_phase_step_ * static_cast<double>(i / 3);
          double s = std::sin(omega * t + phase);
          double value = base;
          if (axis < 2) {
            value = Clamp(base + integer_xy_range_ * s, base - integer_xy_range_, base + integer_xy_range_);
          } else {
            value = Clamp(base + integer_z_range_ * ApplyZBias(0.5 * (s + 1.0)), base, base + integer_z_range_);
          }
          arr[i] = static_cast<int>(std::llround(value));
        }
      }
    }
  }

  void ApplyRandomVariation() {
    double global_strength = std::max(0.0, random_global_strength_);
    double local_strength  = std::max(0.0, random_local_strength_);
    const double sum_strength = global_strength + local_strength;
    if (sum_strength <= 0.0) return;
    if (sum_strength > 1.0) { global_strength /= sum_strength; local_strength /= sum_strength; }
    double alpha     = std::clamp(random_temporal_alpha_, 0.0, 1.0);
    double zero_prob = std::clamp(random_zero_prob_, 0.0, 1.0);
    std::uniform_real_distribution<double> zero_dist(0.0, 1.0);

    for (auto &arrays : sensor_arrays_) {
      bool use_calib = use_calibrated_if_present_ && arrays.has_calibrated;
      if (use_calib && arrays.calibrated) {
        auto &arr = *arrays.calibrated;
        const double gxy = calib_xy_range_ * global_strength;
        const double lxy = calib_xy_range_ * local_strength;
        const double gz  = calib_z_range_  * global_strength;
        const double lz  = calib_z_range_  * local_strength;
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
            value = Clamp(base + ((axis == 0) ? global_x : global_y) + lxy_dist(rng_),
                          base - calib_xy_range_, base + calib_xy_range_);
          } else {
            value = Clamp(base + global_z + ApplyZBias(unit_dist(rng_)) * lz,
                          base, base + calib_z_range_);
          }
          if (zero_dist(rng_) < zero_prob) value = base;
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
        const double gz  = integer_z_range_  * global_strength;
        const double lz  = integer_z_range_  * local_strength;
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
            value = Clamp(base + ((axis == 0) ? global_x : global_y) + lxy_dist(rng_),
                          base - integer_xy_range_, base + integer_xy_range_);
          } else {
            value = Clamp(base + global_z + ApplyZBias(unit_dist(rng_)) * lz,
                          base, base + integer_z_range_);
          }
          if (zero_dist(rng_) < zero_prob) value = base;
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

  // ── 2F-specific scenario variations ────────────────────────

  // CLEAR: silent — restore to reference baseline (no synthetic variation)
  void ApplyScenario2FClear() {
    RestoreBaseArrays();
  }

  // PICK: contact taxels only — baseline for all, Z-force on contact taxels.
  // Non-contact taxels stay at baseline so only the contact pattern is visible.
  // (instant=true in service handler means elapsed_sec >= 10 → factor always 1.0)
  void ApplyScenario2FPick(double elapsed_sec) {
    RestoreBaseArrays();  // all taxels → silent baseline
    const double factor = std::clamp(elapsed_sec / 3.0, 0.0, 1.0);
    ApplyContactForce(ContactTaxelsForCurrentGrasp(), factor);
  }

  // PLACE: ramp-down on contact taxels, non-contact taxels stay at baseline.
  void ApplyScenario2FPlace(double elapsed_sec) {
    RestoreBaseArrays();
    const double factor = std::clamp(1.0 - elapsed_sec / 3.0, 0.0, 1.0);
    ApplyContactForce(ContactTaxelsForCurrentGrasp(), factor);
  }

  // SLIP: one-shot waveform on contact taxels only — force drop then recovery.
  // Phase 0 (0 – 0.3 s): factor 1.0 → 0.0  (sudden loss)
  // Phase 1 (0.3 – 1.0 s): factor 0.0 → 0.5 (partial recovery / re-grip)
  // Phase 2 (1.0 – 1.5 s): factor 0.5 → 1.0 (full recovery)
  // After 1.5 s BuildMessage() reverts to scenario_2f_pick automatically.
  static constexpr double kSlipDurationSec = 1.5;
  void ApplyScenario2FSlip(double elapsed_sec) {
    RestoreBaseArrays();  // non-contact taxels stay silent
    double factor = 1.0;
    if (elapsed_sec < 0.3) {
      factor = 1.0 - (elapsed_sec / 0.3);
    } else if (elapsed_sec < 1.0) {
      factor = 0.5 * ((elapsed_sec - 0.3) / 0.7);
    } else if (elapsed_sec < kSlipDurationSec) {
      factor = 0.5 + 0.5 * ((elapsed_sec - 1.0) / 0.5);
    } else {
      factor = 1.0;
    }
    ApplyContactForce(ContactTaxelsForCurrentGrasp(), factor);
  }

  void ApplyVariation(const std::string &mode, double elapsed_sec) {
    if (mode == "sine") {
      ApplySineVariation(elapsed_sec);
    } else if (mode == "random") {
      ApplyRandomVariation();
    } else if (mode == "scenario_2f_clear") {
      ApplyScenario2FClear();
    } else if (mode == "scenario_2f_pick") {
      ApplyScenario2FPick(elapsed_sec);
    } else if (mode == "scenario_2f_place") {
      ApplyScenario2FPlace(elapsed_sec);
    } else if (mode == "scenario_2f_slip") {
      ApplyScenario2FSlip(elapsed_sec);
    } else {
      ApplyRandomVariation();
    }
  }

  std::string BuildMessage(const rclcpp::Time &node_start_time) {
    const std::string mode    = CurrentVariationMode();
    const double var_elapsed  = VariationElapsedSeconds();

    std::lock_guard<std::mutex> lock(sensor_arrays_mutex_);

    if (mode == "random" || mode == "sine") {
      // Original warmup-based behaviour for background variation modes
      double elapsed = (now() - node_start_time).seconds();
      if (warmup_sec_ <= 0.0 || elapsed >= warmup_sec_) {
        ApplyVariation(mode, elapsed - std::max(0.0, warmup_sec_));
      }
    } else {
      // Auto-revert slip → pick when one-shot waveform is complete
      if (mode == "scenario_2f_slip" && var_elapsed >= kSlipDurationSec) {
        // Revert: set start_time 10 s in the past so pick factor = 1.0 immediately
        {
          std::lock_guard<std::mutex> lock(variation_mutex_);
          variation_mode_       = "scenario_2f_pick";
          variation_start_time_ = now() - rclcpp::Duration::from_seconds(10.0);
        }
        {
          std::lock_guard<std::mutex> lock(state_mutex_);
          current_state_ = MockState::PICK;
        }
        RCLCPP_INFO(get_logger(), "SLIP complete — reverting to PICK (full force)");
        ApplyScenario2FPick(10.0);  // factor = 1.0
      } else {
        // Scenario modes start immediately (no warmup)
        ApplyVariation(mode, var_elapsed);
      }
    }
    return base_json_.dump();
  }

  // ── Service handlers ────────────────────────────────────────

  void HandleScenarioGen(
      const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      current_state_ = MockState::GEN;
    }
    SelectVariationMode("random", "GEN (random variation)", response);
  }

  void HandleScenarioClear(
      const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      current_state_ = MockState::CLEAR;
    }
    SelectVariationMode("scenario_2f_clear", "CLEAR (silent baseline)", response);
  }

  void HandleScenarioPickSharp(
      const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      current_state_       = MockState::PICK;
      current_grasp_type_  = GraspType::SHARP;
    }
    SelectVariationMode("scenario_2f_pick", "PICK sharp (뾰족한 물체: 4 center taxels)", response, true);
  }

  void HandleScenarioPickMedium(
      const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      current_state_      = MockState::PICK;
      current_grasp_type_ = GraspType::MEDIUM;
    }
    SelectVariationMode("scenario_2f_pick", "PICK medium (약간 두툼한 물체: 8 center taxels)", response, true);
  }

  void HandleScenarioPickWide(
      const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      current_state_      = MockState::PICK;
      current_grasp_type_ = GraspType::WIDE;
    }
    SelectVariationMode("scenario_2f_pick", "PICK wide (넓은 물체: all 24 taxels)", response, true);
  }

  void HandleScenarioPlace(
      const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      current_state_ = MockState::PLACE;
      // current_grasp_type_ intentionally kept from last PICK for ramp-down continuity
    }
    SelectVariationMode("scenario_2f_place", "PLACE (ramp-down with last grasp profile)", response);
  }

  void HandleScenarioSlip(
      const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (current_state_ != MockState::PICK) {
        response->success = false;
        response->message = "SLIP rejected: PICK must be active first";
        RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
        return;
      }
      current_state_ = MockState::SLIP;
    }
    SelectVariationMode("scenario_2f_slip", "SLIP one-shot (grasp-pattern slip event)", response);
  }

  // ── WebSocket server loop ───────────────────────────────────

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
      rclcpp::Time node_start_time = now();
      auto period    = std::chrono::milliseconds(std::max(1, publish_period_ms_));
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
            std::string payload = BuildMessage(node_start_time);
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
    if (preset.empty() || preset == "custom") return;
    const std::string base = "replayer_presets." + preset + ".";
    bool applied = false;
    auto try_int = [&](const std::string &key, int &field) {
      if (has_parameter(key)) { field = get_parameter(key).as_int(); applied = true; }
    };
    auto try_dbl = [&](const std::string &key, double &field) {
      if (has_parameter(key)) { field = get_parameter(key).as_double(); applied = true; }
    };
    try_int(base + "publish_period_ms", publish_period_ms_);
    try_dbl(base + "sine_freq_hz",      sine_freq_hz_);
    try_dbl(base + "random_zero_prob",  random_zero_prob_);
    try_dbl(base + "z_bias_power",      z_bias_power_);
    try_dbl(base + "integer_z_range",   integer_z_range_);
    try_dbl(base + "calib_z_range",     calib_z_range_);
    if (applied) {
      RCLCPP_INFO(get_logger(),
                  "Applied preset '%s' (period=%dms, sine_hz=%.3f, z_bias=%.2f)",
                  preset.c_str(), publish_period_ms_, sine_freq_hz_, z_bias_power_);
    }
  }

  // ── Members ─────────────────────────────────────────────────

  std::string model_name_, ref_dir_, bind_host_;
  int bind_port_{5000}, publish_period_ms_{200};
  double warmup_sec_{5.0};
  bool use_calibrated_if_present_{true};
  double integer_xy_range_{1350.0}, integer_z_range_{10085.0};
  double calib_xy_range_{0.8},      calib_z_range_{14.0};
  double z_bias_power_{1.0};
  double sine_freq_hz_{0.05},        sine_phase_step_{0.1};
  int    random_seed_{0};
  double random_global_strength_{0.3}, random_local_strength_{0.7};
  double random_temporal_alpha_{0.7},  random_zero_prob_{0.4};
  std::mt19937 rng_{std::random_device{}()};

  // Scenario state
  MockState  current_state_{MockState::GEN};
  GraspType  current_grasp_type_{GraspType::WIDE};
  std::mutex state_mutex_;

  // Variation control (thread-safe: RunServer thread reads, service threads write)
  std::string    variation_mode_{"random"};
  rclcpp::Time   variation_start_time_{0, 0, RCL_SYSTEM_TIME};
  std::mutex     variation_mutex_;

  // Sensor array access (RunServer writes via Apply*, services can call RestoreBaseArrays)
  std::mutex     sensor_arrays_mutex_;

  std::atomic<bool> stop_{false};
  std::thread worker_;

  std::string base_text_;
  nlohmann::json base_json_;
  std::vector<SensorArrays> sensor_arrays_;

  // Service handles
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_gen_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_clear_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_pick_sharp_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_pick_medium_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_pick_wide_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_place_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_slip_;
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
