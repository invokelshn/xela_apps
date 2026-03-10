# std_xela_taxel_viz_ahv4

Standalone visualization for Allegro Hand v4 taxels. Publishes MarkerArray with an isolated TF tree.

## Requirements
- ROS 2
- `xela_models` and `xela_ah_r2c_bringup` (URDF resources)
- `allegro_hand_bringup` (xacro includes)
- `xela_taxel_msgs`

## Dependency Install (rosdep)
1. Register the local rosdep rules (once per machine):
```bash
WS=~/xela_robotics/02_dev_ws
sudo sh -c "echo 'yaml file://${WS}/src/xela_apps/rosdep/xela_taxel_viz_deps.yaml' > /etc/ros/rosdep/sources.list.d/98-xela-taxel-viz.list"
rosdep update
```
2. Install dependencies:
```bash
rosdep install --from-paths ${WS}/src/xela_apps/std_xela_taxel_viz_ahv4 -r -y --ignore-src
```
Note: the rosdep rules assume ROS 2 Humble deb packages (e.g. `ros-humble-xela-models`). If you use source checkouts instead, place those packages in your workspace and `--ignore-src` will skip them.

## Topics
- Input: `/x_taxel_ah`
- Output: `/<ns>/markers` (default `xvizah/markers`)
- Joint states:
  - Global: `/joint_states`
  - Local: `/<ns>/joint_states`

## Quick Start
Grid mode:
```bash
ros2 launch std_xela_taxel_viz_ahv4 std_xela_taxel_viz_ahv4.launch.py viz_mode:=grid
```

URDF mode (local joint_states):
```bash
ros2 launch std_xela_taxel_viz_ahv4 std_xela_taxel_viz_ahv4.launch.py viz_mode:=urdf joint_states_mode:=local
```

Use external joint_states:
```bash
ros2 launch std_xela_taxel_viz_ahv4 std_xela_taxel_viz_ahv4.launch.py viz_mode:=urdf joint_states_mode:=global
```

Sim all-in-one (replayer + viz + rviz):
```bash
ros2 launch std_xela_taxel_viz_ahv4 sim_all_svc_std_xela_taxel_viz_ahv4.launch.py
```

Sim all-in-one options (grid/urdf + joint_states_mode):
```bash
ros2 launch std_xela_taxel_viz_ahv4 sim_all_svc_std_xela_taxel_viz_ahv4.launch.py \
  viz_mode:=urdf joint_states_mode:=local
```

Sim all-in-one with model selection:
```bash
ros2 launch std_xela_taxel_viz_ahv4 sim_all_svc_std_xela_taxel_viz_ahv4.launch.py \
  model_name:=XR23AHRCPP viz_mode:=urdf
```

Sim all-in-one with web sidecar:
```bash
ros2 launch std_xela_taxel_viz_ahv4 sim_all_svc_std_xela_taxel_viz_ahv4.launch.py \
  model_name:=XR23AHLCPP viz_mode:=urdf enable_sidecar:=true
```

Real all-in-one (server + viz + rviz):
```bash
ros2 launch std_xela_taxel_viz_ahv4 real_all_svc_std_xela_taxel_viz_ahv4.launch.py
```

Real all-in-one options (grid/urdf + joint_states_mode):
```bash
ros2 launch std_xela_taxel_viz_ahv4 real_all_svc_std_xela_taxel_viz_ahv4.launch.py \
  viz_mode:=urdf joint_states_mode:=local
```

## Launch Arguments
Common arguments for `std_xela_taxel_viz_ahv4.launch.py`:
- `namespace` (default: `xvizah`)
- `viz_mode` (default: `grid`)
- `overlay_grid_in_urdf` (default: `false`)
- `frame_id` (default: `world`)
- `frame_prefix` (default: empty)
- `urdf_xacro_path` (default: internal AH xacro)
- `sequence` (default: `0`)
- `hand_side` (default: `left`)
- `mapping_yaml` (default: `config/maps/taxel_joint_map_new.yaml`)
- `pattern_yaml` (default: `config/patterns/pattern_lahv4.yaml`)
- `joint_states_mode` (default: `local`)
- `joint_states_device_profile` (default: auto from `hand_side`)

Common arguments for all-svc launch files:
- `rviz_config` (empty uses grid/urdf preset)
- `model_name` (sim/real: determines left/right defaults)
- `bind_host`, `bind_port`, `ws_host`, `ws_port` (sim only)
- `frame_ids_yaml` (sim/real server config)
- `viz_mode` and `joint_states_mode` can be passed to sim all-svc to select grid/urdf and local/global joint states.
- `joint_states_mode` is only meaningful when `viz_mode=urdf`.

## Default Parameters (Basic Launch)
Defaults when running `ros2 launch std_xela_taxel_viz_ahv4 std_xela_taxel_viz_ahv4.launch.py`:
- `namespace`: `xvizah`
- `viz_mode`: `grid`
- `overlay_grid_in_urdf`: `false`
- `frame_id`: `world`
- `frame_prefix`: empty
- `urdf_xacro_path`: internal AH xacro
- `sequence`: `0`
- `hand_side`: `left`
- `mapping_yaml`: `config/maps/taxel_joint_map_new.yaml`
- `pattern_yaml`: `config/patterns/pattern_lahv4.yaml`
- `joint_states_mode`: `local`
- `joint_states_device_profile`: auto from `hand_side`

## Default Node Params (from base config)
Primary defaults in `config/base/std_xela_taxel_viz_ahv4.yaml`:
- `in_topic`: `/x_taxel_ah`
- `out_topic`: `markers`
- `frame_id`: `world`
- `frame_prefix`: empty (overridden by launch if set)
- `viz_mode`: `grid`
- `joint_states_mode`: `local`
- `global_joint_states_topic`: `/joint_states`
- `local_joint_states_topic`: `joint_states`

## Key Parameters
Node parameters in `config/base/std_xela_taxel_viz_ahv4.yaml`:
- `in_topic`, `out_topic`
- `frame_id`, `frame_prefix`
- `viz_mode`, `overlay_grid_in_urdf`
- `grid_rows`, `grid_cols`, `cell_size`
- `baseline_duration_sec`
- `marker_stamp_mode`, `marker_time_offset_sec`

Local joint-state publisher parameters:
- `output_topic` (default: `joint_states`)
- `publish_rate`
- `config_yaml` (device profile list)
- `device_profile`
- `extra_joints` (adds `ah_joint00~33`)
- `joint_names` (optional explicit list)
- `initial_positions` (optional)

## Hand Side Selection
- `model_name` controls left/right defaults for sim/real all-svc launch files.
- You can override explicitly with:
  - `hand_side:=left|right`
  - `urdf_xacro_path:=...`
  - `pattern_yaml:=...`
  - `mapping_yaml:=...`

## Configuration
- Base: `config/base/std_xela_taxel_viz_ahv4.yaml`
- Mapping: `config/maps/taxel_joint_map_new.yaml`
- Patterns: `config/patterns/pattern_lahv4.yaml`, `pattern_rahv4.yaml`
- Joint list profiles: `config/joints/joint_state_profiles.yaml`
- RViz configs: `config/rviz/`

## Troubleshooting
- RViz TF errors: set `marker_stamp_mode:=now` and keep `marker_time_offset_sec` negative.
- No markers: confirm `/x_taxel_ah` is publishing and `viz_mode` matches RViz config.
- URDF mode blank: verify `frame_prefix` and `joint_states_mode` are consistent.
- RobotModel is a white blob: your TF frames are prefixed but RViz is not. Keep `frame_prefix` empty (recommended) or set RViz RobotModel `TF Prefix` to match and use a prefixed `Fixed Frame` (e.g. `xvizah_world`).

## Notes
- Default namespace: `xvizah`
- `/tf` and `/tf_static` are remapped to `/<ns>/tf` and `/<ns>/tf_static` by the launch file.
- If you start RViz manually, remap TF topics:
```bash
rviz2 --ros-args -r /tf:=/xvizah/tf -r /tf_static:=/xvizah/tf_static
```
