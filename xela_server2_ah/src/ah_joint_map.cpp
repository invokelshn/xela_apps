#include "xela_server2_ah/ah_joint_map.hpp"

#include <algorithm>
#include <cctype>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>

#include "yaml-cpp/yaml.h"

namespace xela_server2_ah {
namespace {

bool ParseJointName(const std::string &name, int &sensor_pos, std::string &model,
                    std::string &error) {
  // Expected: x_taxel_<group>_<sensor_pos>_<model>_<index>_joint
  std::vector<std::string> tokens;
  tokens.reserve(8);
  std::stringstream ss(name);
  std::string token;
  while (std::getline(ss, token, '_')) {
    tokens.push_back(token);
  }
  if (tokens.size() < 7) {
    error = "joint name does not match expected token count";
    return false;
  }
  if (tokens[0] != "x" || tokens[1] != "taxel" || tokens.back() != "joint") {
    error = "joint name does not match expected prefix/suffix";
    return false;
  }
  const std::string &sensor_token = tokens[3];
  if (sensor_token.empty() ||
      !std::all_of(sensor_token.begin(), sensor_token.end(),
                   [](unsigned char c) { return std::isdigit(c) != 0; })) {
    error = "sensor_pos token is not numeric";
    return false;
  }
  try {
    sensor_pos = std::stoi(sensor_token);
  } catch (const std::exception &e) {
    error = std::string("sensor_pos parse error: ") + e.what();
    return false;
  }
  model = tokens[4];
  if (model.empty()) {
    error = "model token is empty";
    return false;
  }
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
  std::unordered_map<int, size_t> module_index;

  for (size_t i = 0; i < entries.size(); ++i) {
    const auto &pair = entries[i];
    const int flat_index = pair.first;
    const std::string &joint_name = pair.second;

    int sensor_pos = 0;
    std::string model;
    std::string parse_error;
    if (!ParseJointName(joint_name, sensor_pos, model, parse_error)) {
      std::ostringstream msg;
      msg << "failed to parse joint '" << joint_name << "': " << parse_error;
      error = msg.str();
      return false;
    }

    JointMapEntry entry;
    entry.flat_index = static_cast<size_t>(flat_index);
    entry.sensor_pos = sensor_pos;
    entry.model = model;
    entry.joint_name = joint_name;
    out.flat.push_back(entry);

    auto it = module_index.find(sensor_pos);
    if (it == module_index.end()) {
      ModuleMap module;
      module.sensor_pos = sensor_pos;
      module.model = model;
      module.flat_indices.push_back(entry.flat_index);
      module.frame_ids.push_back(joint_name);
      out.modules.push_back(std::move(module));
      module_index.emplace(sensor_pos, out.modules.size() - 1U);
    } else {
      ModuleMap &module = out.modules[it->second];
      if (module.model != model) {
        std::ostringstream msg;
        msg << "model mismatch for sensor_pos " << sensor_pos << ": '"
            << module.model << "' vs '" << model << "'";
        error = msg.str();
        return false;
      }
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

}  // namespace xela_server2_ah
