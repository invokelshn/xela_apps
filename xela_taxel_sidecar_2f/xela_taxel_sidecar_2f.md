# Functional Specification
# xela_taxel_sidecar_2f

## 0. Overview
`xela_taxel_sidecar_2f` is a transport-and-render adapter layer between ROS tactile messages and browser-side visualization.

It intentionally separates concerns:
- ROS bridge node converts `/x_taxel_2f` into compact web JSON state
- mode manager provides runtime `grid/urdf` switching without relaunch
- sidecar HTTP + optional rosbridge provide isolated web delivery path

The package is designed to attach to either legacy viz targets or namespaced `std_xela_taxel_viz_2f` targets without changing upstream sensor publishers.

## 1. Purpose
Provide a standalone web-side visualization bridge for Xela 2F tactile stream with runtime mode switch support.

## 2. Scope
- topic bridge (`/x_taxel_2f` -> `/x_taxel_2f/web_state`)
- optional grasp telemetry metadata bridge (`/x_telemetry_2f/grasp_telemetry`, `/x_telemetry_2f/grasp_event`)
- sidecar web server and optional isolated rosbridge
- mode manager service (`grid <-> urdf`)
- compatibility with std viz package namespaced TF topics
- browser UI support for `GridMode`, `XelaModel`, and `RobotModel`

## 3. Interfaces
### 3.1 Input
- `/x_taxel_2f` (`xela_taxel_msgs/msg/XTaxelSensorTArray`)
- optional: `/x_telemetry_2f/grasp_telemetry` (`std_msgs/msg/String`)
- optional: `/x_telemetry_2f/grasp_event` (`std_msgs/msg/String`)

### 3.2 Output
- `/x_taxel_2f/web_state` (`std_msgs/msg/String` JSON)
- service: `/xela_viz_mode_manager/set_mode` (`std_srvs/srv/SetBool`)
- static web assets on `http://<web_host>:<web_port>/`

### 3.3 Integration targets
- bridge parameter service: `/<bridge_node_name>/set_parameters`
- viz mode service: `/<viz_node_name>/set_mode`

## 4. Functional requirements
- FR-01: bridge node shall publish JSON state for web rendering.
- FR-01a: when enabled, bridge node shall attach recent grasp telemetry/event JSON to the web payload.
- FR-02: mode manager shall translate boolean mode request to `viz_mode` parameter updates.
- FR-03: mode manager shall request viz mode service when available.
- FR-04: launch shall support local web server and optional local rosbridge.
- FR-05: launch shall support namespaced TF remap for std-viz integration.
- FR-06: web client shall support `GridMode`, `XelaModel`, and `RobotModel` from the same web payload.
- FR-07: web client shall try websocket candidates in order (`:9090`, `:3201`, `/ros`) unless an explicit `ws` URL is provided.

## 5. Non-functional requirements
- NFR-01: runtime mode switch shall not require process relaunch.
- NFR-02: package shall run independently from MoveIt Pro runtime.
- NFR-03: ROS 2 Humble compatibility.

## 6. Acceptance checks
- AC-01: `xela_taxel_sidecar_cpp.launch.py` starts bridge/manager/web server/rosbridge.
- AC-02: `/x_taxel_2f/web_state` publishes while tactile input is present.
- AC-02a: when grasp metadata inputs are present, recent telemetry/event JSON is embedded in the payload.
- AC-03: mode service call updates bridge mode behavior.
- AC-04: web UI renders `GridMode`, `XelaModel`, and `RobotModel`.
- AC-05: web client can connect through the sidecar rosbridge default path or its documented fallbacks.
