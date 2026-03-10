# Functional Specification (FSD)
# std_xela_taxel_viz_ahv4

## 1. Purpose
A standalone taxel visualization app for Allegro Hand v4 sensors that remains independent of robot-control TF/robot_description. The app builds and uses its **own TF tree** with a dedicated prefix.

## 2. Scope
- Supports AH v4 taxel sensors.
- Uses only sensor data + joint name lists as inputs.
- Outputs visualization markers on a dedicated topic/namespace.
- Maintains an internal TF tree based on a local URDF.

## 3. Non-Goals
- No control, command, or state feedback to robot controllers.
- No modifications to global `/robot_description`, `/tf`, `/joint_states` streams.
- No filtering/smoothing beyond baseline and deadband rules defined here.

## 4. Inputs
### 4.1 Required Topics
- **Sensor data**: `/x_taxel_ah`
- **Joint name list**: `/joint_states` (global mode) or `/<ns>/joint_states` (local mode)

### 4.2 Optional Inputs
- **Mapping YAML**: `taxel_joint_map_new.yaml`
- **Pattern YAML**: `pattern_lahv4.yaml` / `pattern_rahv4.yaml`

## 5. Outputs
- **Markers**: `/xvizah/markers`
- **Internal TF tree**: names prefixed by `xvizah_`

## 6. Independence Requirements
- Must not subscribe to or publish control TF frames.
- Must not depend on `/robot_description` from external robot stack.
- All TFs used for marker poses must originate from the app’s own URDF.

## 7. Core Behavior
1. Load base configuration + mapping/pattern overrides.
2. Load local URDF and publish internal TF.
3. Subscribe to sensor topic and joint_states (global or local mode).
4. Map taxel indices to TF frames via mapping YAML.
5. Compute baseline and deadband.
6. Render markers and publish.

## 8. Visualization Modes
### 8.1 Grid Mode
- Layout defined by pattern (rows/cols) + index map.
- Uses marker grid plane, circles, arrows.

### 8.2 URDF Mode
- Marker poses follow internal URDF taxel link frames.
- No reliance on robot-control TF.

## 9. Baseline & Deadband
- Baseline computed for a configurable warm-up period.
- Deadband applied to remove residual noise.

## 10. Timestamp Policy
- Marker timestamp modes: `keep`, `now`, `zero`.
- Optional offset `marker_time_offset_sec` to avoid TF extrapolation errors.

## 11. Configuration
### 11.1 Base Config
- `config/base/std_xela_taxel_viz_ahv4.yaml`

### 11.2 Joint States Mode
- `joint_states_mode`: `global` or `local`.
- `global_joint_states_topic`: default `/joint_states`.
- `local_joint_states_topic`: default `/<ns>/joint_states`.

### 11.3 Namespaces
- `namespace`: `xvizah`.
- Marker topic defaults to `/<ns>/markers`.
- Local joint states defaults to `/<ns>/joint_states`.

### 11.4 Mapping & Pattern
- `config/maps/taxel_joint_map_new.yaml`
- `config/patterns/pattern_lahv4.yaml` / `pattern_rahv4.yaml`

## 12. Acceptance Criteria
- Markers render correctly for AH v4 sensors.
- No dependence on external `/robot_description` or `/tf` frames.
- TF tree remains isolated with prefix.
- Marker stream is stable without TF extrapolation errors.
- Local mode uses only local joint_states and does not depend on external `/joint_states`.
