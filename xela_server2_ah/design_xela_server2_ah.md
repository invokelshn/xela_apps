# Design - xela_server2_ah

## 1. Overview
This document describes the design for the `xela_server2_ah` ROS 2 node that
converts WebSocket JSON messages into `xela_taxel_msgs/msg/XTaxelSensorArray`
and publishes them on `/x_taxel_ah` for Allegro Hand (AHv4) visualization.
The design mirrors `xela_server2_2f` to maximize code reuse and operational parity.

## 2. Goals
- Provide a stable WebSocket client that reconnects on failure.
- Parse and validate JSON with clear drop reasons.
- Map data into ROS 2 messages exactly as defined in the FSD.
- Use a deterministic joint map (`server_model_joint_map.yaml`) shared with AHv4 visualization.
- Support Allegro Hand models `XR23AHLCPP` (left) and `XR23AHRCPP` (right).

## 3. Non-Goals
- No buffering or backfill of historical messages.
- No on-disk logging of raw WebSocket data.
- No transformation into URDF/TF beyond `frame_ids` assignment.

## 4. Technology Choices
- Language: C++ (rclcpp)
- WebSocket: `boost::beast` (Boost.Asio)
- JSON: `nlohmann/json` (vendored single-header)
- YAML: `yaml-cpp` for mapping file

### 4.1 Dependencies and Auto-Install
To make fresh clones buildable without manual setup, keep dependencies in
`package.xml` and rely on `rosdep` during setup:
- ROS 2: `rclcpp`, `std_msgs`
- Xela msgs/libs: `xela_taxel_msgs`, `xela_server_ros2`
- System libs: `yaml-cpp`, `boost` (beast/asio), `nlohmann_json`

Recommended setup command (document in README):
```
rosdep install --from-paths src --ignore-src -r -y
```
This installs missing system dependencies automatically in new environments.

## 5. Package Layout (proposed)
- `xela_apps/xela_server2_ah/`
  - `package.xml`
  - `CMakeLists.txt`
  - `config/`
    - `server_model_joint_map.yaml`
  - `include/xela_server2_ah/`
    - `ah_parser.hpp`
    - `ah_joint_map.hpp`
  - `src/`
    - `ah_node.cpp` (main node)
    - `ah_parser.cpp` (JSON to message conversion)
    - `ah_joint_map.cpp` (YAML loading helper)

Naming note:
- Use `ah_` prefixes in this package to avoid confusion with `xela_server2_2f`.
- Renaming `xela_server2_2f` sources to `2f_` is possible but not required
  (would be a breaking refactor with low value).

## 6. Parameters
- `ws_host` (string, default: `localhost`)
- `ws_port` (int, default: `5000`)
- `frame_ids_yaml` (string, default: path to `server2_ah_config.yaml`)
- `hand_side` (string, default: `left`, accepts `left|right|l|r`)
- `header_frame_id` (string, default: empty)
- `use_ros_time_for_sensor_time` (bool, default: `false`)
- `publisher_qos_depth` (int, default: `10`)
- `input_json_path` (string, default: empty; enables file playback mode)
- `playback_interval_ms` (int, default: `100`)
- `playback_loop` (bool, default: `true`)

Config note:
- `server2_ah_config.yaml` is a separate wrapper config (for parity with
  `xela_server2_2f`) and should reference the mapping file.
  `server_model_joint_map.yaml` remains the source of truth for joint mapping.

Recommended `server2_ah_config.yaml` schema:
```
mapping_yaml: server_model_joint_map.yaml
```

## 7. Data Flow
1. WebSocket receives JSON text (or file playback supplies JSON).
2. JSON is parsed and validated.
3. Joint map is loaded once and cached.
4. `XTaxelSensorArray` is constructed.
5. Message is published to `/x_taxel_ah`.

## 8. Parsing and Mapping Details
### 8.1 Module Ordering
- Build modules from the joint map grouped by `sensor_pos`.
- Order modules by ascending `sensor_pos`.
- `md_frame_ids` aligns 1:1 with `x_modules` order.

### 8.2 Mapping Schema
`server_model_joint_map.yaml` format:
```
taxel_joint_map:
  0: <joint_name>
  1: <joint_name>
  ...
  367: <joint_name>
```
The loader sorts the numeric keys and builds a contiguous `frame_ids` list.
The mapping file is not duplicated; it must stay consistent with the AHv4 viz map.

### 8.3 Joint Name Parsing
Joint names follow `x_taxel_<group>_<sensor_pos>_<model>_<index>_joint`.
The parser extracts:
- `sensor_pos` (used for module ordering)
- `model` (used for `md_frame_ids` and `x_modules[i].model`)

The joint list is grouped by `sensor_pos`, and each group preserves the
order from the sorted map (required for integer/calibrated/temp alignment).
In the mapping file, `sensor_pos` is stored as a zero-padded two-digit token
(e.g., `03`, `15`). Parse to an integer for the message field.

### 8.4 Hand Side Prefix Mapping
The mapping file uses `x_taxel_0_` prefixes. For right-hand operation, the
node rewrites `x_taxel_0_` to `x_taxel_1_` after loading the map. This is
controlled by the `hand_side` parameter.

### 8.4 integer -> Taxel
- Use `integer` array if present.
- Convert sequential triples into `Taxel(x,y,z)`.
- If counts mismatch, warn and use the available min count.

Note: Some inputs include a `data` CSV hex string. It is not parsed in this design,
but leave a reference comment near the parsing logic for future fallback work.

### 8.5 calibrated -> Forces
- Use `calibrated` array when present.
- Convert sequential triples into `Forces(x,y,z)`.
- If missing, leave `forces` empty.

### 8.6 temp -> temps
- Use `temp` array (Kelvin) when present.
- Convert to Celsius: `C = K - 273.15`.
- Preserve `frame_ids` order.

## 9. Threading and Concurrency
- WebSocket runs in a dedicated thread with `boost::asio::io_context`.
- ROS 2 node runs in the main thread with a `SingleThreadedExecutor`.
- Use a thread-safe queue (size 1) to hand off raw JSON strings.

## 10. Error Handling Strategy
- JSON parse error: warn and drop.
- Missing required fields: warn and drop.
- Mapping load failure: error and drop.
- Length mismatch: warn and use available count.
- `sensor` not parseable to int: warn and drop.
- WebSocket error/close: log and reconnect after a short delay.

## 11. Logging
- Info: connection established, reconnect attempts, playback enabled.
- Warn: parse/validation failures or count mismatches.
- Error: mapping load failures.

## 12. QoS
- Publisher QoS depth from parameter (default 10).
- Reliability: default rclcpp QoSProfile (reliable).

## 13. Testing Plan (Design-Level)
- Unit tests for integer parsing, calibrated parsing, temp conversion, and map loading.
- Unit test to verify 368 frame IDs are loaded in order.
- Integration test using `XR23AHLCPP_left.json` and file playback.
- Runtime verification with `ros2 topic echo /x_taxel_ah`.

## 14. Open Items
- Confirm whether a separate mapping file is needed for right-hand hardware.
