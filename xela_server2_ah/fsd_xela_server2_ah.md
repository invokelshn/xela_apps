# Functional Specification Document (FSD) - xela_server2_ah

## Phase Checklist
- [ ] Requirements review complete (inputs, outputs, mapping, parameters confirmed)
- [ ] Design review complete (architecture, parsing, QoS, error handling confirmed)
- [ ] Development complete (package structure, node implementation, parameters, logging)
- [ ] Testing complete (unit tests for parsing, integration test with sample JSON)
- [ ] Delivery complete (launch instructions and runtime validation)

## 1. Purpose and Scope
The `xela_server2_ah` app is a ROS 2 node that receives JSON messages over WebSocket
(`ws://localhost:5000`), converts them to `xela_taxel_msgs/msg/XTaxelSensorArray`,
and publishes the result on `/x_taxel_ah` for Allegro Hand (AHv4) visualization
for both left and right hands.

## 2. Inputs
### 2.1 WebSocket
- Host: `localhost`
- Port: `5000`
- Protocol: WebSocket (text frames containing JSON)

### 2.2 JSON Schema (based on XR23AHLCPP_left.json)
Top-level fields:
- `message` (integer)
- `time` (float, seconds)
- `sensors` (integer)
- `"1".."N"`: sensor objects keyed by numeric strings

Sensor object fields (used by the app):
- `time` (float, seconds)
- `sensor` (string or integer)
- `model` (string; supported: `XR23AHLCPP` left, `XR23AHRCPP` right)
- `taxels` (integer, expected 368 for Allegro Hand)
- `integer` (array of uint16 values, length = `taxels * 3`)
- `calibrated` (optional array of floats, length = `taxels * 3`)
- `data` (optional CSV hex string; not parsed, only mentioned for reference)

Other fields (e.g., `temp`, `handedness`, `ups`) may exist and are ignored.

## 3. Outputs
### 3.1 Topic
- Topic: `/x_taxel_ah`
- Message type: `xela_taxel_msgs/msg/XTaxelSensorArray`

### 3.2 Message Fields
- `header.stamp`: ROS time at publish moment
- `header.frame_id`: configurable (default empty string)
- `md_frame_ids`: list of sensor `model` strings in sensor index order
- `x_modules`: array of `xela_taxel_msgs/msg/XTaxelSensor`

## 4. Data Mapping
### 4.1 Joint Name Parsing Rules
Joint names in `server_model_joint_map.yaml` follow the pattern:
`x_taxel_<group>_<sensor_pos>_<model>_<index>_joint`

Example: `x_taxel_0_15_uSCuALHA_02_joint`
- `sensor_pos`: `15`
- `model`: `uSCuALHA`

Note: `sensor_pos` is stored in the mapping file as a zero-padded two-digit token
(e.g., `03`, `15`). It is parsed to an integer for the message field.

### 4.2 Module Ordering
Modules are created in ascending order of `sensor_pos`.

### 4.3 md_frame_ids
`md_frame_ids` is a 1:1 list aligned with `x_modules`, ordered by ascending `sensor_pos`.
Each entry is the parsed `model` token from the joint names for that `sensor_pos`.

### 4.4 XTaxelSensor Fields
For each module `i` (ordered by `sensor_pos`):
- `message`: `in.message`
- `time`: `in."<module_key>".time` (default) or ROS time if override enabled
- `model`: parsed `model` token from joint names (e.g., `uSCuALHA`)
- `sensor_pos`: parsed `sensor_pos` token from joint names (e.g., `15`)
- `frame_ids`: joint names for this `sensor_pos`, in their mapped order
- `taxels`: `integer` values mapped to `frame_ids` order
- `forces`: `calibrated` values mapped to `frame_ids` order (if present)
- `temps`: `temp` values mapped to `frame_ids` order, converted from Kelvin to Celsius

### 4.5 frame_ids Mapping (YAML)
Source: `xela_server2_ah/config/server_model_joint_map.yaml`
- YAML key: `taxel_joint_map`
- Keys: `0..367`
- Values: joint names (e.g., `x_taxel_0_15_uSCuALHA_02_joint`)

The map is converted to a list by sorting keys ascending and taking the values.
The resulting list must have length `taxels` (expected 368).

The joint list is then grouped by `sensor_pos`. Each module uses the sub-list
for its `sensor_pos` in the original order defined by the sorted map.

Input arrays (`integer`, `calibrated`, `temp`) are aligned to this sorted list
by flat index. Index `i` corresponds to the joint name at position `i` in the
sorted map. After mapping each flat index to its joint name, values are regrouped
by `sensor_pos` to form `x_modules`.

Note: `xela_taxel_viz_ahv4/config/taxel_joint_map_new.yaml` must remain identical
so that visualization mapping stays consistent (no duplicate copy is required).

### 4.10 Hand Side Prefix Mapping
The mapping file uses `x_taxel_0_` prefixes. When `hand_side=right`, the node
rewrites these to `x_taxel_1_` to align with right-hand URDF and visualization.

### 4.6 integer -> Taxel[]
- `integer` array (uint16), grouped in triples:
  - taxel 0: values 0-2 -> (x, y, z)
  - taxel 1: values 3-5 -> (x, y, z)
  - ...

If `data` is present in the input JSON, it is ignored. Leave a comment in code
as a reference for potential future fallback behavior.

### 4.7 calibrated -> Forces[]
- `calibrated` is an array of floats grouped in triples:
  - force 0: values 0-2 -> (x, y, z)
  - ...
- If `calibrated` is missing or empty, `forces` is left empty.

### 4.8 temp -> temps
- `temp` is an array of floats in Kelvin, one per taxel.
- Convert each entry to Celsius: `C = K - 273.15`.
- `temps` order must match `frame_ids` order.

### 4.9 Example Mapping (Excerpt)
Given this `taxel_joint_map` excerpt (sorted by key):
```
0:  x_taxel_0_15_uSCuALHA_02_joint
1:  x_taxel_0_15_uSCuALHA_03_joint
62: x_taxel_0_03_uSCuALHA_02_joint
63: x_taxel_0_03_uSCuALHA_03_joint
```
Flat index alignment:
- `integer[0..2]` -> `x_taxel_0_15_uSCuALHA_02_joint`
- `integer[3..5]` -> `x_taxel_0_15_uSCuALHA_03_joint`
- `integer[186..188]` -> `x_taxel_0_03_uSCuALHA_02_joint` (index 62)
- `integer[189..191]` -> `x_taxel_0_03_uSCuALHA_03_joint` (index 63)

Grouping by `sensor_pos` (ascending):
- `sensor_pos=3`:
  - `model=uSCuALHA`
  - `frame_ids=[x_taxel_0_03_uSCuALHA_02_joint, x_taxel_0_03_uSCuALHA_03_joint, ...]`
- `sensor_pos=15`:
  - `model=uSCuALHA`
  - `frame_ids=[x_taxel_0_15_uSCuALHA_02_joint, x_taxel_0_15_uSCuALHA_03_joint, ...]`

`md_frame_ids` aligns 1:1 with `x_modules` in `sensor_pos` order.

## 5. Configuration Parameters
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

## 6. Processing Flow
1. Connect to WebSocket and subscribe to text messages.
2. Parse incoming JSON message.
3. Validate required fields and array lengths.
4. Load `taxel_joint_map` once and cache frame list.
5. Build `XTaxelSensorArray` and publish to `/x_taxel_ah`.

## 7. Validation and Error Handling
- JSON parse failure: warn and drop.
- Missing required fields: warn and drop.
- `taxels` missing or <= 0: warn and drop.
- `integer` length mismatch with `taxels`: warn and use the available min count.
- `calibrated` length mismatch: warn and use the available min count.
- `temp` length mismatch with `taxels`: warn and use the available min count.
- Mapping file missing or wrong length: error and drop.

## 8. Logging and Diagnostics
- Connection established / closed events.
- Drop counters by reason (parse, validation, conversion).
- Optional debug: sensor count, array lengths, mapping size.

## 9. Performance Expectations
- O(N) conversion per message (N = number of taxels).
- Minimal allocations beyond message building.

## 10. Dependencies
- ROS 2 (rclcpp, std_msgs)
- `xela_taxel_msgs`, `xela_server_ros2`
- YAML parser (mapping file)
- WebSocket client library

## 11. Acceptance Criteria
- When fed `XR23AHLCPP.json`, node publishes `/x_taxel_ah` with:
  - 1 `x_modules` entry
  - 368 `taxels` per module
  - `forces` empty if `calibrated` is missing
  - `frame_ids` matching `server_model_joint_map.yaml` (0..367)
  - `md_frame_ids` matching each sensor `model`
- Parameter `use_ros_time_for_sensor_time=true` sets `XTaxelSensor.time` to ROS time.

## 12. Open Items
- Confirm how to handle alternate mapping files for future hardware variants.
