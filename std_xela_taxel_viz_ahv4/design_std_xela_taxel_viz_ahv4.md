# Design Document
# std_xela_taxel_viz_ahv4

## 1. Architecture Overview
A standalone ROS 2 visualization node for Allegro Hand v4 taxels that publishes MarkerArray outputs.
The node runs with its **own TF tree** and **local URDF**, fully isolated from robot-control TF and external `/robot_description`.

Key properties:
- Independent TF prefix via `frame_prefix` (default `xvizah_`).
- Grid mode (pattern-driven layout) and URDF mode (internal URDF frames).
- Minimal inputs: `/x_taxel_ah` and joint_states (global/local).

## 2. Components
### 2.1 std_xela_taxel_viz_ahv4 Node
- Subscribes:
  - `/x_taxel_ah`
  - `/joint_states` (global mode) or `/<ns>/joint_states` (local mode)
- Publishes:
  - `/<ns>/markers` (default `xvizah/markers`)
- Internal TF:
  - `robot_state_publisher` with prefixed frames.

### 2.2 Internal URDF Loader
- Loads local URDF under the app namespace.
- Prefix applied to prevent collisions with robot-control TF.

### 2.3 Configuration System
- Base defaults: `config/base/std_xela_taxel_viz_ahv4.yaml`
- Mapping + pattern:
  - `config/maps/taxel_joint_map_new.yaml`
  - `config/patterns/pattern_lahv4.yaml` / `pattern_rahv4.yaml`

## 3. Data Flow
1. Load configuration (base + mapping + pattern).
2. Load internal URDF and start TF publisher.
3. Subscribe to joint_states (global or local).
4. Receive `/x_taxel_ah` data.
5. Map taxel indices to frames via mapping YAML.
6. Compute baseline.
7. Build markers and publish.

## 4. TF Isolation Strategy
- Fixed frame: `frame_id` (grid) or internal root (URDF).
- All frames used for marker poses are under `frame_prefix`.
- No external TF is queried; only app-owned TF is used.

## 4.1 Joint States Mode
- **Global**: subscribe to `/joint_states` from external stack.
- **Local**: publish `/<ns>/joint_states` from a local joint-state publisher (no controller).
- Local mode must not depend on external `/joint_states`.

## 5. Visualization Pipeline
### 5.1 Grid Mode
- Uses pattern layout (31x26) and index map.
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
- Deadband per axis reduces noise and residual offsets.

## 7. Timestamp Policy
- `marker_stamp_mode`: `keep`, `now`, `zero`.
- `marker_time_offset_sec`: small negative offset to avoid TF extrapolation.

## 8. Extensibility
- New hand pattern: add mapping + pattern YAMLs.
- New URDF: update `urdf_xacro_path` or description files.

## 9. Risks & Mitigations
- **TF extrapolation**: use `marker_stamp_mode=now` + negative offset.
- **Residual sensor noise**: tune baseline duration.
- **Frame mismatch**: keep mapping + URDF consistent, use `frame_prefix`.

## 10. Namespaces
- Default namespace: `xvizah`.
- Marker topic: `/<ns>/markers`.
- Local joint states: `/<ns>/joint_states`.

## 11. Acceptance Alignment
Design follows FSD requirements:
- Independent TF tree
- No dependency on external `/robot_description`
- Minimal inputs only
- Stable visualization without TF errors
