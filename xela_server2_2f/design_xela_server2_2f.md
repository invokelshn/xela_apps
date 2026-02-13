# Design - xela_server2_2f

## 1. Overview
This document describes the design for the `xela_server2_2f` ROS 2 node that
converts WebSocket JSON messages into `xela_taxel_msgs/msg/XTaxelSensorTArray`
and publishes them on `/x_taxel_2f`. Temperatures from the WebSocket message
are converted from Kelvin to Celsius.

## 2. Goals
- Provide a stable WebSocket client that reconnects on failure.
- Parse and validate JSON with clear drop reasons.
- Map data into ROS 2 messages exactly as defined in the FSD.
- Support model-specific frame configs with optional auto-generated names.

## 3. Non-Goals
- No buffering or backfill of historical messages.
- No on-disk logging of raw WebSocket data.

## 4. Technology Choices
- Language: C++ (rclcpp)
- WebSocket: `boost::beast` (Boost.Asio)
- JSON: `nlohmann/json` (vendored single-header)
- YAML: `yaml-cpp` for frame config

Rationale: Boost is widely available in ROS 2 C++ environments and `beast`
provides a performant WebSocket client with good control over connection
behavior. `nlohmann/json` and `yaml-cpp` are common in ROS 2 stacks.

## 5. Package Layout
- `xela_apps/xela_server2_2f/`
  - `package.xml`
  - `CMakeLists.txt`
  - `include/xela_server2_2f/`
    - `parser.hpp`
    - `frame_ids.hpp`
  - `src/`
    - `node.cpp` (main node)
    - `parser.cpp` (JSON to message conversion)
    - `frame_ids.cpp` (YAML loading helper)

## 6. Parameters
- `ws_host` (string, default: `localhost`)
- `ws_port` (int, default: `5000`)
- `frame_ids_yaml` (string, default: path to `server2_2f_config.yaml`)
- `header_frame_id` (string, default: empty)
- `use_ros_time_for_sensor_time` (bool, default: `false`)
- `publisher_qos_depth` (int, default: `10`)
- `input_json_path` (string, default: empty; enables file playback mode)
- `playback_interval_ms` (int, default: `100`)
- `playback_loop` (bool, default: `true`)

## 7. Data Flow
1. WebSocket receives JSON text (or file playback supplies JSON).
2. JSON is parsed and validated.
3. Model frame config is loaded once and cached.
4. `XTaxelSensorTArray` is constructed.
5. Message is published to `/x_taxel_2f`.

## 8. Parsing and Mapping Details
### 8.1 Sensor Ordering
- Sort sensor keys numerically: `"1"`, `"2"`, ...
- Iterate in ascending order to produce arrays.

### 8.2 Config Schema
The YAML file supports a global section and model entries. Model keys may include
multiple names separated by commas.

- `global.frame_prefix` (optional): prefix used in final frame IDs.
- `taxel_count` (optional): default taxel count for the model.
- `gen_frame_list` (bool): whether to auto-generate frame IDs.
- `gen_model` (optional): model name used for generated frame IDs when `gen_frame_list=true`.
- `names` or `frame_ids` (optional if `gen_frame_list=true`): list of frame IDs.

### 8.3 Frame ID Construction
Final frame IDs are built as:
`frame_prefix + "_" + sensor_id_2digits + "_" + base_name`

- `sensor_id_2digits` comes from `in["i"].sensor` (01, 02, ...).
- `base_name` is either:
  - auto-generated: `<gen_model or model>_<seq(01..taxel_count)>_joint`, or
  - from `names`/`frame_ids` list.

If `frame_prefix` is empty, `sensor_id_2digits` becomes the prefix.

### 8.4 taxel_count Resolution
- Use `in["i"].taxels` if present.
- Else use config `taxel_count`.
- Else use `names.size()` when `gen_frame_list=false`.

### 8.5 integer -> Taxel
- Use `in["i"].integer` array.
- Convert sequential triples into `Taxel(x,y,z)`.
- If counts mismatch, warn and use the available count.

### 8.6 calibrated -> Forces
- Use `in["i"].calibrated` array.
- Convert sequential triples into `Forces(x,y,z)`.
- If counts mismatch, warn and use the available count.

### 8.7 temp -> temps
- Use `in["i"].temp` array in Kelvin.
- Convert to Celsius: `C = K - 273.15`.
- If counts mismatch, warn and use the available count.

## 9. Threading and Concurrency
- WebSocket runs in a dedicated thread with `boost::asio::io_context`.
- ROS 2 node runs in the main thread with a `SingleThreadedExecutor`.
- Use a thread-safe queue (size 1) to hand off raw JSON strings.

Decision: The WebSocket thread only enqueues messages. A ROS timer parses,
builds, and publishes to keep ROS callbacks deterministic.

## 10. Error Handling Strategy
- JSON parse error: warn and drop.
- Missing required fields: warn and drop.
- Model key missing in config: warn and skip that sensor.
- Length mismatch: warn and use the available count.
- `sensor` not parseable to int: warn and drop.
- WebSocket error/close: log and reconnect after a short delay.

## 11. Logging
- Info: connection established, reconnect attempts, playback enabled.
- Warn: parse/validation failures or count mismatches.
- Error: frame config load failures.

## 12. QoS
- Publisher QoS depth from parameter (default 10).
- Reliability: keep default rclcpp QoSProfile (reliable).

## 13. Testing Plan (Design-Level)
- Unit tests for parsing functions (integer arrays, calibrated arrays, temp conversion).
- Unit test for YAML model mapping and aliases.
- Integration test using `tmp/websocket.json` and a local WebSocket sender.
- Runtime verification with `ros2 topic echo /x_taxel_2f`.

## 14. Open Items
- None.
