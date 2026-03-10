# xela_taxel_sidecar_2f

Web sidecar package for **Xela 2F** tactile visualization.

## Components
- `xela_taxel_web_bridge_node` (C++)
- `xela_viz_mode_manager_node` (C++)
- `scripts/sidecar_http_server.py`
- optional sidecar rosbridge websocket
- static web UI (`web/taxel_sidecar`)

## Data flow
1. subscribe `/x_taxel_2f` (`xela_taxel_msgs/msg/XTaxelSensorTArray`)
2. optional subscribe `/x_telemetry_2f/grasp_telemetry`, `/x_telemetry_2f/grasp_event` (`std_msgs/msg/String`)
3. publish `/x_taxel_2f/web_state` (`std_msgs/msg/String`, JSON)
4. web UI reads sidecar rosbridge topic and renders `GridMode`, `XelaModel`, or `RobotModel`
5. service `/xela_viz_mode_manager/set_mode` toggles backend `grid/urdf` behavior at runtime

## Recommended launch (C++)
```bash
ros2 launch xela_taxel_sidecar_2f xela_taxel_sidecar_cpp.launch.py
```

Open:
- `http://localhost:8765`
- If Windows-to-WSL localhost forwarding is unavailable, use the WSL host IP instead (for example `http://<wsl-ip>:8765`).

## Main launch defaults (`xela_taxel_sidecar_cpp.launch.py`)
- `in_topic`: `/x_taxel_2f`
- `out_topic`: `/x_taxel_2f/web_state`
- `viz_mode`: `grid`
- `model_name`: `uSPr2F`
- `style_preset`: `cool_steel`
- `frame_id`: `x_taxel_viz`
- `grid_parent_frame`: `world`
- `legacy_out_topic`: `/x_taxel_2f/markers`
- `fixed_frame`: `world`
- `grid_rows`: `4`
- `grid_cols`: `6`
- `cell_size`: `0.015`
- `module_gap`: `0.04`
- `row_flip_right`: `true`
- `col_flip_right`: `true`
- `max_publish_rate_hz`: `20.0`
- `publisher_transient_local`: `false`
- `marker_stamp_mode`: `now`
- `circle_marker_type`: `sphere`
- `emit_urdf_points`: `false`
- `freeze_urdf_positions`: `true`
- `enable_viz_mode_manager`: `true`
- `managed_launch_package`: `ur5e_x2f_140_config2`
- `managed_launch_file`: `xela_viz_safe.launch.py`
- `bridge_node_name`: `/xela_taxel_web_bridge_cpp`
- `viz_node_name`: `/xela_taxel_viz_2f`
- `bridge_tf_topic`: `/tf`
- `bridge_tf_static_topic`: `/tf_static`
- `enable_web_server`: `true`
- `web_host`: `0.0.0.0`
- `web_port`: `8765`
- `enable_sidecar_rosbridge`: `true`
- `sidecar_rosbridge_host`: `0.0.0.0`
- `sidecar_rosbridge_port`: `9090`

## Bridge runtime parameters (not launch-wired by default)
- `include_grasp_meta`: `true`
- `grasp_telemetry_topic`: `/x_telemetry_2f/grasp_telemetry`
- `grasp_event_topic`: `/x_telemetry_2f/grasp_event`

## Web UI runtime notes
- Browser display modes are client-side: `GridMode`, `XelaModel`, `RobotModel`.
- Backend `viz_mode` remains ROS-side `grid/urdf`; the browser can switch views without changing the ROS mode.
- Websocket fallback order is:
  - `ws://<host>:9090`
  - `ws://<host>:3201`
  - `ws://<host>/ros`
- URL query parameters can override runtime behavior, including:
  - `topic`, `ns`
  - `xela_*` and `robot_model_*` TF / robot description endpoints
  - `follow_cam_*` tuning values for RobotModel follow logic
- FollowCam and Analysis logic remain implemented in the web client even when their toolbar buttons are temporarily hidden for test builds.
- Current Analysis timeline behavior:
  - `contact_detected` markers are emitted only during the pinch/grasp phase
  - `slip_warning` markers prefer explicit event payloads over inferred markers
  - tooltip content includes `mode`, `reason`, `close_pos`, and `target_effort`
  - tooltip placement uses a fixed viewport overlay with boundary clamping so long messages are not clipped by the Analysis panel

## FollowCam and Analysis usage
- `FollowCam`
  - available when the UI is in `RobotModel`
  - click `FollowCam` to toggle automatic front-upper camera tracking for the gripper / fingertip region
  - manual mouse orbit temporarily pauses follow for `3s`, then auto-follow resumes
  - click `FollowCam` again to return to normal RobotModel orbit framing
- `Analysis`
  - click `Analysis` to show or hide the lower stats/timeline panel
  - the panel shows force/phase/event summaries, sparkline history, event markers, and hover tooltip inspection
  - timeline controls (`10s`, `30s`, `60s`, `Pause`) affect only the analysis panel and do not change ROS-side behavior

## FollowCam URL parameter contract
- There is no separate `follow_cam_profile` key in the current implementation.
- FollowCam camera behavior is tuned through these query parameters:
  - `follow_cam_forward_distance`: forward camera offset from the resolved tip center
  - `follow_cam_height_offset`: world-Z height offset above the target
  - `follow_cam_look_ahead`: look target offset back toward the sensor area
  - `follow_cam_forward_sign`: flips the resolved forward hemisphere (`1` or `-1`)
- Current built-in baseline (used when the query is absent):
  - `follow_cam_forward_distance=-0.2`
  - `follow_cam_height_offset=0.1`
  - `follow_cam_look_ahead=0.028`
  - `follow_cam_forward_sign=-1.0`
- Example explicit URL using the current baseline:
```text
http://localhost:8765/?follow_cam_forward_distance=-0.2&follow_cam_height_offset=0.1&follow_cam_look_ahead=0.028&follow_cam_forward_sign=-1.0
```

## Runtime mode switching
```bash
# grid -> urdf
ros2 service call /xela_viz_mode_manager/set_mode std_srvs/srv/SetBool "{data: true}"

# urdf -> grid
ros2 service call /xela_viz_mode_manager/set_mode std_srvs/srv/SetBool "{data: false}"
```

## std_xela_taxel_viz_2f integration example
```bash
ros2 launch xela_taxel_sidecar_2f xela_taxel_sidecar_cpp.launch.py   viz_mode:=urdf   model_name:=uSPrDS   viz_node_name:=/xviz2f/std_xela_taxel_viz_2f   bridge_tf_topic:=/xviz2f/tf   bridge_tf_static_topic:=/xviz2f/tf_static
```

## Build
```bash
cd ~/my_moveit_pro/01_wk_xela_mpro_dev_ws
moveit_pro build user_workspace --colcon-args "--packages-select xela_taxel_sidecar_2f"
```

## Troubleshooting
- web disconnected:
  - check `rosbridge_websocket_sidecar` and `sidecar_rosbridge_port`
  - on Windows + WSL, if `http://localhost:8765` is refused but `http://<wsl-ip>:8765` works, the issue is usually WSL localhost forwarding rather than the sidecar process
- no tactile updates:
  - check `/x_taxel_2f` and `/x_taxel_2f/web_state`
- mode switch no effect:
  - check `/xela_viz_mode_manager/set_mode`
  - verify `viz_node_name` service exists for selected integration target
- duplicate sidecar nodes:
  - do not run Python and C++ launch variants together

## Documentation
- Functional spec: `xela_taxel_sidecar_2f.md`
- Design: `design_xela_taxel_sidecar_2f.md`
- Non-Web use cases for grasp telemetry: `telemetry_non_web_use_cases_2f.md`
- Redmine: `README.textile`, `xela_taxel_sidecar_2f.textile`, `design_xela_taxel_sidecar_2f.textile`
