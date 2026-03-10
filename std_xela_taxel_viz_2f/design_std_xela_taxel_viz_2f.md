# Design Document
# std_xela_taxel_viz_2f

## 1. Architecture Overview
A standalone ROS 2 visualization node for 2F fingertip taxels that publishes MarkerArray outputs.
The node runs with its **own TF topic namespace** and **local URDF**, isolated from robot-control TF and external `/robot_description`.

Key properties:
- Default TF isolation by topic remap (`/<ns>/tf`, `/<ns>/tf_static`).
- Optional frame-name isolation via `frame_prefix` (default empty).
- Grid mode (config-driven layout) and URDF mode (internal URDF frames).
- Runtime mode switch service at `/<ns>/std_xela_taxel_viz_2f/set_mode`.
- Minimal inputs: `/x_taxel_2f` and joint_states (global/local).

## 2. Components
### 2.1 std_xela_taxel_viz_2f Node
- Subscribes:
  - `/x_taxel_2f`
  - `/joint_states` (global mode) or `/<ns>/joint_states` (local mode)
- Publishes:
  - `/<ns>/markers` (default `xviz2f/markers`)
  - `/<ns>/std_xela_taxel_viz_2f/set_mode` (`std_srvs/srv/SetBool`)
- Internal TF:
  - `robot_state_publisher` publishing on namespaced `tf/tf_static`
  - optional prefixed frames when `frame_prefix` is set

### 2.2 Internal URDF Loader
- Loads local URDF under the app namespace.
- Prefix applied to prevent collisions with robot-control TF.

### 2.3 Configuration System
- Base defaults: `config/base/std_xela_taxel_viz_2f.yaml`
- Model/mode overrides:
  - `config/models/<model>/grid.yaml`
  - `config/models/<model>/urdf.yaml`

## 3. Data Flow
1. Load configuration (base + model/mode override).
2. Load internal URDF and start namespaced TF publisher.
3. Subscribe to joint_states (global or local).
4. Receive `/x_taxel_2f` data.
5. Map taxel indices to frames via:
   - grid index map (grid mode), or
   - URDF taxel link names (URDF mode).
6. Compute baseline and apply deadband.
7. Build markers and publish.
8. Accept `SetBool` requests on `~/set_mode` to switch between `grid` and `urdf` without relaunch.

## 4. TF Isolation Strategy
- Fixed frame: `frame_id` (grid) or internal root (URDF).
- Default isolation uses namespaced TF topics; `frame_prefix` adds frame-name isolation only when needed.
- No external TF is queried; only app-owned TF is used.

## 4.1 Joint States Mode
- **Global**: subscribe to `/joint_states` from external stack.
- **Local**: publish `/<ns>/joint_states` from a local joint-state publisher (no controller).
- Local mode must not depend on external `/joint_states`.

## 5. Visualization Pipeline
### 5.1 Grid Mode
- Uses grid layout: rows, cols, index maps, separators.
- Renders:
  - Grid plane
  - Per-taxel circles (magnitude)
  - Per-taxel arrows (direction)

### 5.2 URDF Mode
- Uses internal URDF frames for pose.
- Marker placement follows taxel link frames.
- Optional overlay grid disabled by default.

## 6. Baseline & Deadband
- Baseline period: configurable warm-up duration.
- Deadband per axis (force + taxel) reduces noise and residual offsets.
- Supports independent XY and Z thresholds.

## 7. Timestamp Policy
- `marker_stamp_mode`: `keep`, `now`, `zero`.
- `marker_time_offset_sec`: small negative offset to avoid TF extrapolation.

## 8. Extensibility
- New model: add URDF + grid/urdf config overrides.
- New sensor type: add input topic, frame map, and layout.

## 9. Risks & Mitigations
- **TF extrapolation**: use `marker_stamp_mode=now` + negative offset.
- **Residual sensor noise**: tune deadband + baseline duration.
- **Frame mismatch**: use consistent `frame_prefix` and model configs.

## 10. Namespaces
- Default namespace: `xviz2f`.
- Marker topic: `/<ns>/markers`.
- Local joint states: `/<ns>/joint_states`.

## 11. Acceptance Alignment
Design follows FSD requirements:
- Independent TF stream under app namespace
- No dependency on external `/robot_description`
- Minimal inputs only
- Stable visualization without TF errors
- Runtime mode switch via `~/set_mode`
