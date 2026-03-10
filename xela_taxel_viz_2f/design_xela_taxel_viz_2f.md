# xela_taxel_viz_2f Design

## 1. Design Goals
- Provide low-latency tactile visualization for 2F models.
- Keep one node supporting both `grid` and `urdf` rendering paths.
- Support runtime mode change without process restart.
- Keep marker IDs stable to reduce flicker in RViz/web clients.

## 2. Runtime Architecture
Single ROS 2 node:
- Node: `xela_taxel_viz_2f`
- Subscribe: `xela_taxel_msgs/msg/XTaxelSensorTArray`
- Publish: `visualization_msgs/msg/MarkerArray`
- Service server: `std_srvs/srv/SetBool` (`/xela_taxel_viz_2f/set_mode`)

Launch-side composition:
- `xela_taxel_viz_2f.launch.py`
  - always starts visualizer node
  - starts taxel URDF stack only when `viz_mode=urdf`
- `real_all_svc_xela_taxel_viz_2f.launch.py`
  - server + visualizer + RViz
- `sim_all_svc_xela_taxel_viz_2f.launch.py`
  - replayer + visualizer + RViz

## 3. Data Pipeline
1. Receive `/x_taxel_2f` message.
2. Resolve timestamp according to `marker_stamp_mode` and `marker_time_offset_sec`.
3. Update/lock baseline state per module.
4. If rate limiting enabled, skip publish when period not elapsed.
5. Build marker array:
   - optional grid background (grid mode or URDF overlay)
   - module circles + arrows in selected mode
6. Publish to `out_topic` and optional `legacy_out_topic`.

## 4. Mode Paths
### 4.1 Grid Mode
- Uses synthetic frame (`frame_id`, default `x_taxel_viz`).
- Computes cell center positions from:
  - `grid_rows`, `grid_cols`, `cell_size`, `module_gap`, `origin_*`
- Supports mapping control:
  - `row_flip_right`, `col_flip_right`
  - `grid_index_map_left/right`
  - `grid_separator_cols_left/right`

### 4.2 URDF Mode
- Uses per-taxel frame IDs from message (`x_modules[].frame_ids[]`).
- `_joint` suffix is converted to `_link` for marker frame resolution.
- Marker pose is local origin of each frame.
- Optional grid overlay controlled by `overlay_grid_in_urdf`.

## 5. Runtime Mode Switch Service
Service: `/xela_taxel_viz_2f/set_mode` (`SetBool`)
- `false` -> `grid`
- `true` -> `urdf`

Implementation behavior:
- If requested mode equals current mode: return success/no-op message.
- Else:
  - publish `DELETEALL`
  - update internal `viz_mode_`
  - continue processing next messages in new mode

This avoids external relaunch-based switching and reduces transition latency.

## 6. Magnitude, Direction, and Baseline
### 6.1 Source selection
- Default uses `forces[]`.
- If `forces[]` is empty and `use_taxels_when_no_forces=true`, use `taxels[]`.

### 6.2 Baseline
- Baseline is collected per module for `baseline_duration_sec`.
- Separate baseline buffers for forces and taxels.
- Before baseline ready, output is effectively suppressed.

### 6.3 Normalization
Two modes:
- axis-normalized (`use_axis_normalization=true`)
- raw magnitude with `max_force` clamp

Ranges:
- force mode: `xy_force_range`, `z_force_range`
- taxel fallback: `xy_taxel_range`, `z_taxel_range`

### 6.4 Direction signs
Independent sign parameters by mode:
- Grid: `left_force_*`, `right_force_*`
- URDF: `urdf_left_force_*`, `urdf_right_force_*`

This is used to align displayed XY direction with physical sensor orientation.

## 7. Marker Construction Strategy
- Grid background markers are prebuilt and reused.
- Dynamic markers are emitted every processed message.
- Stable ID scheme:
  - `grid`: `module_offset + idx`
  - `circle`: `module_offset + 100 + idx`
  - `arrow`: `module_offset + 200 + idx`
  - `module_offset = module_index * 1000`

Benefits:
- deterministic overwrite in RViz
- reduced visual flicker

## 8. QoS and Timing
- Output QoS durability configurable by `publisher_transient_local`.
- Timestamp policy:
  - `keep`: use message stamp (fallback to now)
  - `now`: force current time
  - `zero`: force time zero
- Optional rate cap via `max_publish_rate_hz`.

## 9. Integration Notes
- In larger systems (e.g., full robot stack), URDF mode can introduce model/TF conflicts if a separate primary robot model is already active.
- Recommended pattern:
  - use one authoritative robot model publisher per graph
  - include taxel URDF publishers only when model namespace/ownership is clear

## 10. Known Tradeoffs
- Service mode switch does not change launch-time URDF stack ownership.
  - If URDF support processes are not present, switching to URDF may still lack full TF context.
- Grid mode is more robust under constrained web/TF pipelines.
- URDF mode provides better frame-anchored semantics but is more sensitive to TF timing/coverage.
