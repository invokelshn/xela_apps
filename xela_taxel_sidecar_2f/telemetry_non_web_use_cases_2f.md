# x_telemetry_2f Non-Web Use Cases

## 1) Topic contract (default)
- `/x_telemetry_2f/grasp_telemetry` (`std_msgs/msg/String`, JSON): continuous grasp status snapshot while close/slip monitor is running
- `/x_telemetry_2f/grasp_event` (`std_msgs/msg/String`, JSON): important state transition events (contact detected, stop reason, timeout, etc.)
- `/x_telemetry_2f/grasp_metric/*` (`std_msgs/msg/Float64`): scalar streams for plotting/threshold logic
  - `force_total`, `force_peak`, `force_fz`
  - `point_peak`, `point_topk`, `point_sigma`, `active_taxels`
  - `point_sparse`, `close_pos`, `gripper_vel`

## 2) Why these topics are useful outside Web UI
- They are ROS-native and lightweight, so any ROS app can subscribe without browser/rosbridge dependency.
- `grasp_event` is event-driven for automation logic, while `grasp_metric/*` is time-series friendly for monitoring and analytics.
- The same telemetry can feed multiple consumers simultaneously (Objective, logger, quality checker, external gateway).

## 3) Concrete usage scenarios

### A. Production pass/fail gate (MES/PLC bridge)
- Goal: stop releasing bad picks to next station.
- Method:
  - Subscribe to `/x_telemetry_2f/grasp_event`.
  - On `contact_detected`, check latest `point_sigma`, `force_peak`, `active_taxels`.
  - If below configured bounds, publish NG signal to MES/PLC bridge and route to retry Objective.
- Benefit: tactile-based quality gate with no Web UI dependency.

### B. Recipe-specific threshold tuning service
- Goal: one line handles paper crane, box, and soft object with different sensitivity.
- Method:
  - A recipe manager app subscribes to `/x_telemetry_2f/grasp_metric/*`.
  - It stores successful grasp distributions per SKU and updates thresholds by service/parameter set before run.
- Benefit: data-driven threshold updates instead of hardcoded constants.

### C. Real-time robot safety interlock
- Goal: prevent over-compression during closing.
- Method:
  - Safety node monitors `force_fz` and `point_sparse`.
  - If rapid force increase with low contact area is detected, it commands pause/hold and logs event.
- Benefit: fast local safety action based on tactile signal shape.

### D. Slip-aware transport supervisor
- Goal: avoid drop during move-to-place.
- Method:
  - During transport phase, monitor `grasp_telemetry` JSON trend (force drop + contact sparsity increase).
  - If slip trend exceeds rule, trigger regrasp waypoint or reduced acceleration profile.
- Benefit: continuous grasp health control during motion, not only at initial contact.

### E. Offline process analytics and troubleshooting
- Goal: improve pick success over time.
- Method:
  - Record `/x_telemetry_2f/*` with rosbag.
  - Correlate tactile traces with camera/object class and final result labels.
  - Build per-object failure signatures (e.g., sharp-tip misses vs broad-face success).
- Benefit: objective root-cause analysis and measurable tuning loop.

## 4) Minimal integration patterns

### Pattern 1: event-only consumer (simple state machine)
```bash
ros2 topic echo /x_telemetry_2f/grasp_event
```
- Use for sequence branching, alarms, and retry triggers.

### Pattern 2: metric consumer (plot/threshold)
```bash
ros2 topic echo /x_telemetry_2f/grasp_metric/point_sigma
ros2 topic echo /x_telemetry_2f/grasp_metric/point_sparse
ros2 topic echo /x_telemetry_2f/grasp_metric/gripper_vel
```
- Use for control limits and trend monitoring.

### Pattern 3: mixed consumer (event + metric snapshot)
- Subscribe to `grasp_event` and cache latest metrics.
- On event arrival, evaluate pass/fail with cached metrics and publish decision topic.

## 5) Operational recommendations
- Keep one canonical namespace for plant integration: `/x_telemetry_2f/...`.
- Prefer event-triggered logic for branching, and metric trends for tuning.
- Store telemetry with run ID/object ID for post-analysis reproducibility.

## 6) Design TODO (FSD-level backlog)
- `fz_left`/`fz_right` split and publication:
  - define deterministic left/right module mapping per model (`uSPa46` first), then publish side-specific force channels.
- left/right symmetry metrics for grasp quality:
  - add `symmetry_score` based on mirrored taxel deltas and side force balance.
  - add `active_taxels_left/right` as contact-area proxy.
- robust contact-area comparison:
  - normalize active-taxel count by available taxels per side/module.
  - validate against sharp-tip and broad-face object datasets.
- slip detector model upgrade:
  - current `slip_warning` is based on shear delta threshold.
  - extend with hysteresis, debounce, and ratio features (`ft/|fz|`) for material-aware robustness.
