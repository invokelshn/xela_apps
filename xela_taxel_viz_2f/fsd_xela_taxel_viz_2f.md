# Functional Specification Document (FSD) - xela_taxel_viz_2f

## 1. Purpose
`xela_taxel_viz_2f` visualizes tactile data from `/x_taxel_2f` for 2F-gripper class sensors.
It publishes marker arrays for RViz/Web visualization in either `grid` or `urdf` mode.

## 2. Scope
- In scope:
  - Marker generation from `xela_taxel_msgs/msg/XTaxelSensorTArray`
  - Dual-module grid rendering
  - URDF-frame marker rendering
  - Baseline/zeroing and scaling pipeline
  - Runtime mode switching through ROS service
- Out of scope:
  - Sensor acquisition (`xela_server*`)
  - Taxel frame mapping policy source (`xela_server2_2f` config)

## 3. Interfaces
### 3.1 Input
- Topic: `/x_taxel_2f`
- Type: `xela_taxel_msgs/msg/XTaxelSensorTArray`
- Required fields per module:
  - `forces[]` (primary)
  - `taxels[]` (fallback when configured)
  - `frame_ids[]` (URDF mode)

### 3.2 Output
- Topic: `/x_taxel_2f/markers` (default, configurable)
- Type: `visualization_msgs/msg/MarkerArray`
- Optional mirror topic: `legacy_out_topic`.

### 3.3 Service
- Service: `/xela_taxel_viz_2f/set_mode`
- Type: `std_srvs/srv/SetBool`
- Semantics:
  - `false`: switch to `grid`
  - `true`: switch to `urdf`
- Expected behavior:
  - publish `DELETEALL`
  - switch mode in-process without restart

## 4. Functional Requirements
### FR-1 Mode support
- The node shall support `viz_mode` values `grid` and `urdf`.
- Invalid `viz_mode` shall fall back to `grid`.

### FR-2 Grid rendering
- The node shall render two module grids (left/right) with configurable size and spacing.
- Grid cell placement shall support right-module row/column flips.
- Grid index remapping shall support per-module index maps.

### FR-3 URDF rendering
- The node shall render markers in per-taxel frames from `frame_ids[]`.
- If frame id ends with `_joint`, the node shall use corresponding `_link` frame.

### FR-4 Magnitude and direction mapping
- The node shall compute marker magnitude from force/taxel vectors.
- The node shall support axis-normalized magnitude mode and raw magnitude mode.
- The node shall support XY directional arrows and optional Z-direction arrows.

### FR-5 Baseline and deadband
- The node shall collect baseline for configured duration.
- Before baseline completion, output shall be suppressed to zero-equivalent values.
- Deadband shall be applied independently for XY and Z.

### FR-6 Taxel fallback
- If `forces[]` is empty and `use_taxels_when_no_forces=true`, visualization shall use `taxels[]`.
- Separate range parameters shall be used for taxel fallback normalization.

### FR-7 Runtime mode switching
- The node shall accept mode change through `/xela_taxel_viz_2f/set_mode`.
- Mode switching shall not require process restart.

### FR-8 Output compatibility
- Node shall publish to `out_topic`.
- If `legacy_out_topic` is configured and differs from `out_topic`, node shall publish identical marker arrays to both.

## 5. Non-Functional Requirements
- ROS 2 Humble compatibility.
- Stable marker IDs to minimize flicker.
- Configurable durability (`transient_local` or `volatile`).
- Optional publish-rate limiting via `max_publish_rate_hz`.

## 6. Launch Behavior Requirements
### LR-1 Base launch
`xela_taxel_viz_2f.launch.py` shall:
- always start `xela_taxel_viz_2f_node`
- in `urdf` mode additionally start:
  - `robot_state_publisher`
  - `ros2_control_node`
  - `controller_manager spawner xela_taxel_joint_state_publisher`

### LR-2 Real integration launch
`real_all_svc_xela_taxel_viz_2f.launch.py` shall include:
- server bringup (`xela_server2_2f_with_server.launch.py`)
- visualizer launch include
- RViz with mode-based default config

### LR-3 Sim integration launch
`sim_all_svc_xela_taxel_viz_2f.launch.py` shall include:
- replayer bringup (`xela_server2_2f_with_replayer.launch.py`)
- visualizer launch include
- RViz with mode-based default config

## 7. Constraints and Assumptions
- System assumes two tactile modules (left/right).
- Default geometry assumes 4x6 cells per module.
- URDF mode requires valid TF for the frame IDs carried in message.

## 8. Acceptance Criteria
- AC-1 Grid mode renders two module grids and responds to force changes.
- AC-2 URDF mode attaches markers to taxel frames without remapping errors.
- AC-3 Service mode switching changes visualization mode without node restart.
- AC-4 Fallback to taxel data works when force vector array is absent.
- AC-5 Marker output is available on configured output topic(s).

## 9. Verification Checklist
```bash
ros2 topic echo /x_taxel_2f --once
ros2 topic echo /x_taxel_2f/markers --once
ros2 service call /xela_taxel_viz_2f/set_mode std_srvs/srv/SetBool "{data: true}"
ros2 service call /xela_taxel_viz_2f/set_mode std_srvs/srv/SetBool "{data: false}"
```
