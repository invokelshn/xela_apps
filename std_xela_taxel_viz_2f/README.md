# std_xela_taxel_viz_2f

Standalone visualization for Xela 2F fingertip taxels. Publishes MarkerArray, owns its
namespaced TF stream, and exposes a runtime `set_mode` service for `grid/urdf` switching.

## Requirements
- ROS 2
- `xela_models` (URDF resources)
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
rosdep install --from-paths ${WS}/src/xela_apps/std_xela_taxel_viz_2f -r -y --ignore-src
```
Note: the rosdep rules assume ROS 2 Humble deb packages (e.g. `ros-humble-xela-models`). If you use source checkouts instead, place those packages in your workspace and `--ignore-src` will skip them.

## Topics
- Input: `/x_taxel_2f`
- Output: `/<ns>/markers` (default `xviz2f/markers`)
- Service: `/<ns>/std_xela_taxel_viz_2f/set_mode` (`std_srvs/srv/SetBool`)
- Joint states:
  - Global: `/joint_states`
  - Local: `/<ns>/joint_states`

## Quick Start
Grid mode:
```bash
ros2 launch std_xela_taxel_viz_2f std_xela_taxel_viz_2f.launch.py viz_mode:=grid
```

URDF mode (local joint_states):
```bash
ros2 launch std_xela_taxel_viz_2f std_xela_taxel_viz_2f.launch.py viz_mode:=urdf joint_states_mode:=local
```

Use external joint_states:
```bash
ros2 launch std_xela_taxel_viz_2f std_xela_taxel_viz_2f.launch.py viz_mode:=urdf joint_states_mode:=global
```

Sim all-in-one (replayer + viz + rviz):
```bash
ros2 launch std_xela_taxel_viz_2f sim_all_svc_std_xela_taxel_viz_2f.launch.py
```

Sim all-in-one options (grid/urdf + joint_states_mode):
```bash
ros2 launch std_xela_taxel_viz_2f sim_all_svc_std_xela_taxel_viz_2f.launch.py \
  viz_model_name:=uSPr2F viz_mode:=urdf joint_states_mode:=local
```

Sim all-in-one with sidecar web visualization:
```bash
ros2 launch std_xela_taxel_viz_2f sim_all_svc_std_xela_taxel_viz_2f.launch.py \
  viz_model_name:=uSPr2F viz_mode:=urdf joint_states_mode:=local \
  enable_sidecar:=true
```
Then open:
- `http://localhost:8765`

Real all-in-one (server + viz + rviz):
```bash
ros2 launch std_xela_taxel_viz_2f real_all_svc_std_xela_taxel_viz_2f.launch.py
```

Real all-in-one options (grid/urdf + joint_states_mode):
```bash
ros2 launch std_xela_taxel_viz_2f real_all_svc_std_xela_taxel_viz_2f.launch.py \
  viz_model_name:=uSPr2F viz_mode:=urdf joint_states_mode:=local
```

## Launch Arguments
Common arguments for `std_xela_taxel_viz_2f.launch.py`:
- `namespace` (default: `xviz2f`)
- `viz_mode` (default: `grid`)
- `model_name` (default: `uSPr2F`)
- `style_preset` (default: `default`)
- `overlay_grid_in_urdf` (default: `false`)
- `marker_stamp_mode` (default: `now`)
- `marker_time_offset_sec` (default: `-0.12`)
- `frame_id` (default: `world`)
- `frame_prefix` (default: empty)
- `urdf_xacro_path` (default: internal model xacro)
- `joint_states_mode` (default: `local`)
- `joint_states_config_yaml` (default: `config/joints/joint_state_profiles.yaml`)
- `joint_states_device_profile` (default: `<model_name>`)

Common arguments for all-svc launch files:
- `rviz_config` (empty uses grid/urdf preset)
- `viz_model_name` (visualization model, default: `uSPr2F`)
- `server_model_name` (sim only, default: `<viz_model_name>`)
- `bind_host`, `bind_port`, `ws_host`, `ws_port` (sim only)
- `can_port`, `xela_server_exec` (real only)
- `marker_stamp_mode` and `marker_time_offset_sec` can be passed through to the viz node.
- `viz_mode` and `joint_states_mode` can be passed to sim all-svc to select grid/urdf and local/global joint states.
- `joint_states_mode` is only meaningful when `viz_mode=urdf`.
- Sidecar options (sim only):
  - `enable_sidecar` (default: `false`)
  - `sidecar_out_topic` (default: `/x_taxel_2f/web_state`)
  - `sidecar_enable_web_server`, `sidecar_web_host`, `sidecar_web_port`
  - `sidecar_enable_rosbridge`, `sidecar_rosbridge_host`, `sidecar_rosbridge_port`

## Default Parameters (Basic Launch)
Defaults when running `ros2 launch std_xela_taxel_viz_2f std_xela_taxel_viz_2f.launch.py`:
- `namespace`: `xviz2f`
- `viz_mode`: `grid`
- `model_name`: `uSPr2F`
- `style_preset`: `default`
- `overlay_grid_in_urdf`: `false`
- `marker_stamp_mode`: `now`
- `marker_time_offset_sec`: `-0.12`
- `frame_id`: `world`
- `frame_prefix`: empty
- `joint_states_mode`: `local`
- `joint_states_config_yaml`: `config/joints/joint_state_profiles.yaml`
- `joint_states_device_profile`: `<model_name>`

## Default Parameters (All-in-one Launch)
Defaults when running `sim_all_svc_std_xela_taxel_viz_2f.launch.py` or `real_all_svc_std_xela_taxel_viz_2f.launch.py`:
- `viz_model_name`: `uSPr2F`
- `viz_mode`: `grid`
- `joint_states_mode`: `local`
- `rviz_config`: empty (auto-select grid/urdf config)
- `server_model_name`: `<viz_model_name>` (sim only)
- `marker_stamp_mode`: `now`
- `marker_time_offset_sec`: `-0.12`
- `enable_sidecar`: `false` (sim only)
- `sidecar_web_port`: `8765` (sim only)
- `sidecar_rosbridge_port`: `9090` (sim only)

## Default Node Params (from base config)
Primary defaults in `config/base/std_xela_taxel_viz_2f.yaml`:
- `in_topic`: `/x_taxel_2f`
- `out_topic`: `markers`
- `frame_id`: `world`
- `frame_prefix`: empty (overridden by launch if set)
- `viz_mode`: `grid`
- `joint_states_mode`: `local`
- `global_joint_states_topic`: `/joint_states`
- `local_joint_states_topic`: `joint_states`

## Key Parameters
Node parameters in `config/base/std_xela_taxel_viz_2f.yaml`:
- `in_topic`, `out_topic`
- `frame_id`, `frame_prefix`
- `viz_mode`, `overlay_grid_in_urdf`
- `grid_rows`, `grid_cols`, `cell_size`, `module_gap`
- `baseline_duration_sec`, `baseline_deadband_*`
- `marker_stamp_mode`, `marker_time_offset_sec`

Local joint-state publisher parameters:
- `output_topic` (default: `joint_states`)
- `publish_rate`
- `config_yaml` (device profile list)
- `device_profile`
- `extra_joints` (optional)
- `joint_names` (optional explicit list)
- `initial_positions` (optional)

## Configuration
- Base: `config/base/std_xela_taxel_viz_2f.yaml`
- Model overrides:
  - `config/models/<model>/grid.yaml`
  - `config/models/<model>/urdf.yaml`
- Joint list profiles: `config/joints/joint_state_profiles.yaml`
- RViz configs: `config/rviz/`

## Troubleshooting
- RViz TF errors: set `marker_stamp_mode:=now` and keep `marker_time_offset_sec` negative.
- No markers: confirm `/x_taxel_2f` is publishing and `viz_mode` matches RViz config.
- URDF mode blank: verify `frame_prefix` and `joint_states_mode` are consistent.
- Sidecar mode switch has no effect: confirm `/<ns>/std_xela_taxel_viz_2f/set_mode` exists and `viz_node_name` points to it.
- RobotModel is a white blob: your TF frames are prefixed but RViz is not. Keep `frame_prefix` empty (recommended) or set RViz RobotModel `TF Prefix` to match and use a prefixed `Fixed Frame` (e.g. `xviz2f_world`).

## Notes
- Default namespace: `xviz2f`
- Default TF isolation is topic-based: `/tf` and `/tf_static` are remapped to `/<ns>/tf` and `/<ns>/tf_static` by the launch file.
- `frame_prefix` is optional and defaults to empty. Use it only when frame-name isolation is required in addition to namespaced TF topics.
- If you start RViz manually, remap TF topics:
```bash
rviz2 --ros-args -r /tf:=/xviz2f/tf -r /tf_static:=/xviz2f/tf_static
```
