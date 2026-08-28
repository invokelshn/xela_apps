#include "xela_server2_dg5f/dg5f_joint_map.hpp"

#include <algorithm>
#include <cctype>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>

#include "yaml-cpp/yaml.h"

namespace xela_server2_dg5f {
namespace {

// Parses a taxel joint name into a module identity: "x_taxel_<hand>_<module>_..._joint".
// Two module-naming conventions occur in this workspace's mapping YAMLs (mirrors
// xela_taxel_gateway's gateway_node.cpp, which handles the same two formats):
//   Allegro-style:  tokens[3] is a bare numeric module id (e.g. "03"); tokens[4] is the
//                   sensor type name ("model", e.g. "uSCuAH").
//   Named-module style (DG-5F): tokens[3] is not numeric; the module identity spans
//                   tokens[3..size-3] joined by "_" (e.g. "f3_dg5f_ft", "palm_uSPa46").
//                   sensor_pos is then assigned sequentially in first-seen order.
bool ParseJointName(const std::string &name, std::string &module_name,
                    bool &is_numeric_module, std::string &model, std::string &error) {
  std::vector<std::string> tokens;
  tokens.reserve(8);
  std::stringstream ss(name);
  std::string token;
  while (std::getline(ss, token, '_')) {
    tokens.push_back(token);
  }
  if (tokens.size() < 5) {
    error = "joint name does not match expected token count";
    return false;
  }
  if (tokens[0] != "x" || tokens[1] != "taxel" || tokens.back() != "joint") {
    error = "joint name does not match expected prefix/suffix";
    return false;
  }

  const std::string &sensor_token = tokens[3];
  const bool sensor_token_numeric =
      !sensor_token.empty() && std::all_of(sensor_token.begin(), sensor_token.end(),
                                            [](unsigned char c) { return std::isdigit(c) != 0; });

  if (tokens.size() >= 7 && sensor_token_numeric) {
    module_name = sensor_token;
    is_numeric_module = true;
    model = tokens[4];
    if (model.empty()) {
      error = "model token is empty";
      return false;
    }
    return true;
  }

  std::string joined;
  for (size_t i = 3; i + 2 < tokens.size(); ++i) {
    if (!joined.empty()) {
      joined += "_";
    }
    joined += tokens[i];
  }
  if (joined.empty()) {
    error = "resolved empty module name";
    return false;
  }
  module_name = joined;
  is_numeric_module = false;
  model = joined;
  return true;
}

}  // namespace

bool LoadJointMap(const std::string &yaml_path, JointMap &out, std::string &error) {
  out = JointMap{};
  std::vector<std::pair<int, std::string>> entries;
  try {
    YAML::Node root = YAML::LoadFile(yaml_path);
    YAML::Node map_node = root["taxel_joint_map"];
    if (!map_node) {
      map_node = root["server_model_joint_map"];
    }
    if (!map_node || !map_node.IsMap()) {
      error = "mapping key missing or not a map (taxel_joint_map/server_model_joint_map)";
      return false;
    }

    entries.reserve(map_node.size());
    for (const auto &entry : map_node) {
      int key = entry.first.as<int>();
      std::string joint = entry.second.as<std::string>();
      entries.emplace_back(key, joint);
    }
  } catch (const std::exception &e) {
    error = std::string("YAML load error: ") + e.what();
    return false;
  }

  if (entries.empty()) {
    error = "taxel_joint_map is empty";
    return false;
  }

  std::sort(entries.begin(), entries.end(),
            [](const auto &a, const auto &b) { return a.first < b.first; });

  out.flat.reserve(entries.size());
  std::unordered_map<std::string, size_t> module_index;
  int next_auto_sensor_pos = 0;

  for (size_t i = 0; i < entries.size(); ++i) {
    const auto &pair = entries[i];
    const int flat_index = pair.first;
    const std::string &joint_name = pair.second;

    std::string module_name;
    bool is_numeric_module = false;
    std::string model;
    std::string parse_error;
    if (!ParseJointName(joint_name, module_name, is_numeric_module, model, parse_error)) {
      std::ostringstream msg;
      msg << "failed to parse joint '" << joint_name << "': " << parse_error;
      error = msg.str();
      return false;
    }

    auto it = module_index.find(module_name);
    if (it == module_index.end()) {
      const int sensor_pos = is_numeric_module ? std::stoi(module_name) : next_auto_sensor_pos++;

      JointMapEntry entry;
      entry.flat_index = static_cast<size_t>(flat_index);
      entry.sensor_pos = sensor_pos;
      entry.model = model;
      entry.joint_name = joint_name;
      out.flat.push_back(entry);

      ModuleMap module;
      module.sensor_pos = sensor_pos;
      module.model = model;
      module.flat_indices.push_back(entry.flat_index);
      module.frame_ids.push_back(joint_name);
      out.modules.push_back(std::move(module));
      module_index.emplace(module_name, out.modules.size() - 1U);
    } else {
      ModuleMap &module = out.modules[it->second];
      if (module.model != model) {
        std::ostringstream msg;
        msg << "model mismatch for module '" << module_name << "': '"
            << module.model << "' vs '" << model << "'";
        error = msg.str();
        return false;
      }

      JointMapEntry entry;
      entry.flat_index = static_cast<size_t>(flat_index);
      entry.sensor_pos = module.sensor_pos;
      entry.model = model;
      entry.joint_name = joint_name;
      out.flat.push_back(entry);

      module.flat_indices.push_back(entry.flat_index);
      module.frame_ids.push_back(joint_name);
    }
  }

  std::sort(out.modules.begin(), out.modules.end(),
            [](const ModuleMap &a, const ModuleMap &b) {
              return a.sensor_pos < b.sensor_pos;
            });

  return true;
}

void ApplyHandSideToJointMap(JointMap &map, const std::string &hand_side) {
  const std::string side = hand_side;
  std::string prefix_from = "x_taxel_0_";
  std::string prefix_to = "x_taxel_0_";
  if (side == "right" || side == "r") {
    prefix_from = "x_taxel_0_";
    prefix_to = "x_taxel_1_";
  } else if (side == "left" || side == "l") {
    prefix_from = "x_taxel_1_";
    prefix_to = "x_taxel_0_";
  } else {
    return;
  }

  for (auto &entry : map.flat) {
    if (entry.joint_name.rfind(prefix_from, 0) == 0) {
      entry.joint_name.replace(0, prefix_from.size(), prefix_to);
    }
  }
  for (auto &module : map.modules) {
    for (auto &frame_id : module.frame_ids) {
      if (frame_id.rfind(prefix_from, 0) == 0) {
        frame_id.replace(0, prefix_from.size(), prefix_to);
      }
    }
  }
}

}  // namespace xela_server2_dg5f
