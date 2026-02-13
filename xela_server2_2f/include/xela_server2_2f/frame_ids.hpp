#pragma once

#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace xela_server2_2f {

struct ModelFrameConfig {
  std::vector<std::string> names;
  std::optional<size_t> taxel_count;
  bool gen_frame_list = false;
  std::optional<std::string> gen_model;
};

using ModelFrameMap = std::unordered_map<std::string, ModelFrameConfig>;

struct FrameConfig {
  std::string frame_prefix;
  ModelFrameMap models;
};

FrameConfig LoadFrameConfig(const std::string &yaml_path, std::string &error);

}  // namespace xela_server2_2f
