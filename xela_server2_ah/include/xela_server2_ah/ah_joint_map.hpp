#pragma once

#include <cstddef>
#include <string>
#include <vector>

namespace xela_server2_ah {

struct JointMapEntry {
  size_t flat_index = 0;
  int sensor_pos = 0;
  std::string model;
  std::string joint_name;
};

struct ModuleMap {
  int sensor_pos = 0;
  std::string model;
  std::vector<size_t> flat_indices;
  std::vector<std::string> frame_ids;
};

struct JointMap {
  std::vector<JointMapEntry> flat;
  std::vector<ModuleMap> modules;
};

bool LoadJointMap(const std::string &yaml_path, JointMap &out, std::string &error);
void ApplyHandSideToJointMap(JointMap &map, const std::string &hand_side);

}  // namespace xela_server2_ah
