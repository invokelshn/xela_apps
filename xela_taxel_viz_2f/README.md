# xela_taxel_viz_2f

`xela_taxel_viz_2f` visualizes `/x_taxel_2f` (`xela_taxel_msgs/msg/XTaxelSensorTArray`) as `visualization_msgs/msg/MarkerArray`.
It supports:
- `grid` mode: 2D dual-module tactile grid
- `urdf` mode: markers attached to per-taxel URDF frames

## Features
- Dual-module (left/right) 4x6 visualization with configurable index mapping.
- Baseline/zeroing with per-axis deadband.
- Runtime mode switch service (`grid <-> urdf`) without process restart.
- Optional grid overlay in URDF mode.
- Configurable sign correction for XY direction in both modes.
- Optional compatibility mirror output via `legacy_out_topic`.

## Build
```bash
cd <your_ws>
colcon build --packages-select xela_taxel_viz_2f
source install/setup.bash
```

## Main Interfaces
- Input topic: `/x_taxel_2f`
- Output topic (default): `/x_taxel_2f/markers`
- Optional mirror output: `legacy_out_topic` (empty by default)
- Mode switch service: `/xela_taxel_viz_2f/set_mode` (`std_srvs/srv/SetBool`)
  - `data: false` -> `grid`
  - `data: true` -> `urdf`

## Launch

### 1) Visualizer only
Grid:
```bash
ros2 launch xela_taxel_viz_2f xela_taxel_viz_2f.launch.py \
  model_name:=uSPr2F viz_mode:=grid style_preset:=cool_steel
```

URDF:
```bash
ros2 launch xela_taxel_viz_2f xela_taxel_viz_2f.launch.py \
  model_name:=uSPr2F viz_mode:=urdf overlay_grid_in_urdf:=false
```

In `urdf` mode this launch also starts:
- `robot_state_publisher`
- `ros2_control_node`
- `controller_manager spawner xela_taxel_joint_state_publisher`

### 2) Real hardware + server + viz + RViz
```bash
ros2 launch xela_taxel_viz_2f real_all_svc_xela_taxel_viz_2f.launch.py \
  model_name:=uSPr2F viz_mode:=grid
```

### 3) Simulation/replayer + viz + RViz
```bash
ros2 launch xela_taxel_viz_2f sim_all_svc_xela_taxel_viz_2f.launch.py \
  model_name:=uSPr2F viz_mode:=grid preset:=normal
```

## Runtime Mode Switch (No Restart)
```bash
# grid
ros2 service call /xela_taxel_viz_2f/set_mode std_srvs/srv/SetBool "{data: false}"

# urdf
ros2 service call /xela_taxel_viz_2f/set_mode std_srvs/srv/SetBool "{data: true}"
```

## Parameter Files
- Base: `config/base.yaml`
- Per-model overrides:
  - `config/models/<model>/grid.yaml`
  - `config/models/<model>/urdf.yaml`
  - `config/models/<model>/ros2_controllers.yaml` (URDF mode)

Supported model folders in this package:
- `uSPr2F`
- `uSPrDS`
- `uSPrHE35`

## Important Parameters
- Mode/output
  - `viz_mode`, `overlay_grid_in_urdf`, `in_topic`, `out_topic`, `legacy_out_topic`
- Timing/transport
  - `marker_stamp_mode` (`keep|now|zero`)
  - `marker_time_offset_sec`
  - `max_publish_rate_hz` (`0` disables rate limit)
  - `publisher_transient_local`
- Grid layout/mapping
  - `grid_rows`, `grid_cols`, `cell_size`, `module_gap`
  - `left_module_index`, `right_module_index`
  - `row_flip_right`, `col_flip_right`
  - `grid_index_map_left/right`, `grid_separator_cols_left/right`
- Baseline/scaling
  - `baseline_duration_sec`
  - `baseline_deadband_xy`, `baseline_deadband_z`
  - `baseline_deadband_taxel_xy`, `baseline_deadband_taxel_z`
  - `use_axis_normalization`, `xy_force_range`, `z_force_range`
  - `use_taxels_when_no_forces`, `xy_taxel_range`, `z_taxel_range`
- Direction sign correction
  - Grid: `left_force_x_sign`, `left_force_y_sign`, `right_force_x_sign`, `right_force_y_sign`
  - URDF: `urdf_left_force_x_sign`, `urdf_left_force_y_sign`, `urdf_right_force_x_sign`, `urdf_right_force_y_sign`

## Quick Validation
```bash
ros2 topic echo /x_taxel_2f --once
ros2 topic echo /x_taxel_2f/markers --once
ros2 service list | rg xela_taxel_viz_2f/set_mode
```

## Troubleshooting
- No markers:
  - Check `/x_taxel_2f` publisher and `in_topic` match.
- URDF mode frame errors:
  - Verify model xacro exists: `description/xela_<model>_2_modules.xacro`.
  - Verify TF chain and `/joint_states` publisher.
- Marker lag:
  - Check `max_publish_rate_hz` (set `0` for no throttle).
  - Check `marker_stamp_mode` and `marker_time_offset_sec`.
- Mixed robot model warnings in integrated systems:
  - Avoid launching an extra taxel-only RSP when another primary robot model is already active in the same graph.
