# Functional Specification (FSD)
# std_xela_taxel_viz_2f

## 1. Purpose
A standalone taxel visualization app for 2F sensors that remains independent of robot-control TF/robot_description. The app builds and uses its **own TF stream** under an app namespace, with optional frame prefixing when frame-name isolation is required.

## 2. Scope
- Supports 2F fingertip taxel sensors.
- Uses only sensor data + joint name lists as inputs.
- Outputs visualization markers on a dedicated topic/namespace.
- Maintains an internal TF tree based on a local URDF.

## 3. Non-Goals
- No control, command, or state feedback to robot controllers.
- No modifications to global `/robot_description`, `/tf`, `/joint_states` streams.
- No filtering/smoothing beyond baseline and deadband rules defined here.

## 4. Inputs
### 4.1 Required Topics
- **Sensor data**: `/x_taxel_2f`
- **Joint name list**: `/joint_states` (global mode) or `/<ns>/joint_states` (local mode)

### 4.2 Optional Inputs
- **Model layout YAML**: grid layouts, index maps, separator columns.

## 5. Outputs
- **Markers**: `/xviz2f/markers`
- **Internal TF stream**: `/<ns>/tf` and `/<ns>/tf_static`
- **Runtime mode service**: `/<ns>/std_xela_taxel_viz_2f/set_mode` (`std_srvs/srv/SetBool`)

## 6. Independence Requirements
- Must not subscribe to or publish control TF frames.
- Must not depend on `/robot_description` from external robot stack.
- All TFs used for marker poses must originate from the app’s own URDF.
- Default isolation is achieved by namespaced TF topics; `frame_prefix` is optional and defaults to empty.

## 7. Core Behavior
1. Load base configuration + model/mode overrides.
2. Load local URDF and publish internal TF.
3. Subscribe to sensor topic and joint_states (global or local mode).
4. Map taxel indices to TF frames (grid or URDF mode).
5. Compute baseline and deadband.
6. Render markers and publish.
7. Accept `SetBool` requests on `~/set_mode` to switch `grid <-> urdf` at runtime.

## 8. Visualization Modes
### 8.1 Grid Mode
- Layout defined by model grid (rows/cols) + index map.
- Uses marker grid plane, circles, arrows.

### 8.2 URDF Mode
- Marker poses follow internal URDF taxel link frames.
- No reliance on robot-control TF.

## 9. Baseline & Deadband
- Baseline computed for a configurable warm-up period.
- Deadband applied to remove residual noise.
- Supports independent thresholds for force and taxel values.

## 10. Timestamp Policy
- Marker timestamp modes: `keep`, `now`, `zero`.
- Optional offset `marker_time_offset_sec` to avoid TF extrapolation errors.

## 11. Configuration
### 11.1 Base Config
- `config/base/std_xela_taxel_viz_2f.yaml`

### 11.2 Joint States Mode
- `joint_states_mode`: `global` or `local`.
- `global_joint_states_topic`: default `/joint_states`.
- `local_joint_states_topic`: default `/<ns>/joint_states`.

### 11.3 Namespaces
- `namespace`: `xviz2f`.
- Marker topic defaults to `/<ns>/markers`.
- Local joint states defaults to `/<ns>/joint_states`.

### 11.4 Model Overrides
- `config/models/<model>/grid.yaml`
- `config/models/<model>/urdf.yaml`

## 12. Acceptance Criteria
- Markers render correctly for 2F sensors.
- No dependence on external `/robot_description` or `/tf` frames.
- TF stream remains isolated under the app namespace, with optional frame prefixing when configured.
- Marker stream is stable without TF extrapolation errors.
- Local mode uses only local joint_states and does not depend on external `/joint_states`.
- Runtime `~/set_mode` switches between `grid` and `urdf` without relaunch.
