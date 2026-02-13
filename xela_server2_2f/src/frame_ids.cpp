#include "xela_server2_2f/frame_ids.hpp"

#include <cctype>
#include <sstream>

#include "yaml-cpp/yaml.h"

namespace xela_server2_2f {
namespace {

std::string Trim(const std::string &value) {
  size_t start = 0;
  while (start < value.size() && std::isspace(static_cast<unsigned char>(value[start])) != 0) {
    ++start;
  }
  size_t end = value.size();
  while (end > start && std::isspace(static_cast<unsigned char>(value[end - 1])) != 0) {
    --end;
  }
  return value.substr(start, end - start);
}

std::vector<std::string> SplitModels(const std::string &key) {
  std::vector<std::string> models;
  std::stringstream ss(key);
  std::string token;
  while (std::getline(ss, token, ',')) {
    std::string trimmed = Trim(token);
    if (!trimmed.empty()) {
      models.push_back(trimmed);
    }
  }
  return models;
}

bool LoadModelConfig(const YAML::Node &node, ModelFrameConfig &out, std::string &error) {
  if (node.IsSequence()) {
    for (const auto &item : node) {
      out.names.push_back(item.as<std::string>());
    }
    return true;
  }

  if (!node.IsMap()) {
    error = "model entry is not a map or sequence";
    return false;
  }

  if (node["gen_frame_list"]) {
    out.gen_frame_list = node["gen_frame_list"].as<bool>();
  }

  if (node["gen_model"]) {
    out.gen_model = node["gen_model"].as<std::string>();
  }

  if (node["taxel_count"]) {
    int count = node["taxel_count"].as<int>();
    if (count <= 0) {
      error = "taxel_count must be positive";
      return false;
    }
    out.taxel_count = static_cast<size_t>(count);
  }

  if (node["names"]) {
    if (!node["names"].IsSequence()) {
      error = "names is not a sequence";
      return false;
    }
    for (const auto &item : node["names"]) {
      out.names.push_back(item.as<std::string>());
    }
  } else if (node["frame_ids"]) {
    if (!node["frame_ids"].IsSequence()) {
      error = "frame_ids is not a sequence";
      return false;
    }
    for (const auto &item : node["frame_ids"]) {
      out.names.push_back(item.as<std::string>());
    }
  } else if (!out.gen_frame_list) {
    error = "names or frame_ids is missing";
    return false;
  }

  return true;
}

}  // namespace

FrameConfig LoadFrameConfig(const std::string &yaml_path, std::string &error) {
  FrameConfig config;
  try {
    YAML::Node root = YAML::LoadFile(yaml_path);
    if (root["global"] && root["global"]["frame_prefix"]) {
      config.frame_prefix = root["global"]["frame_prefix"].as<std::string>();
    }

    YAML::Node models_node = root["models"] ? root["models"] : root;
    if (!models_node.IsMap()) {
      error = "config root is not a map";
      return FrameConfig{};
    }

    for (const auto &entry : models_node) {
      if (!entry.first.IsScalar()) {
        continue;
      }
      std::string key = entry.first.as<std::string>();
      if (models_node == root && key == "global") {
        continue;
      }

      auto models = SplitModels(key);
      if (models.empty()) {
        continue;
      }

      ModelFrameConfig model_config;
      if (!LoadModelConfig(entry.second, model_config, error)) {
        if (!error.empty()) {
          error = key + ": " + error;
        }
        return FrameConfig{};
      }

      if (!model_config.gen_frame_list && model_config.names.empty()) {
        error = key + ": names or frame_ids list is empty";
        return FrameConfig{};
      }

      for (const auto &model : models) {
        if (config.models.find(model) != config.models.end()) {
          error = "duplicate model config: " + model;
          return FrameConfig{};
        }
        config.models.emplace(model, model_config);
      }
    }
  } catch (const std::exception &e) {
    error = e.what();
  }
  return config;
}

}  // namespace xela_server2_2f
