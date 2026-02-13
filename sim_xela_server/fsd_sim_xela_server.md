# Functional Specification Document (FSD)
# sim_xela_server WebSocket Replayer Server

## 1. Overview
A ROS 2 node that acts as a WebSocket **server** and repeatedly sends a JSON message using a selected
model reference file from `sim_xela_server/resource/<model>.json`. The node accepts multiple WebSocket client
connections concurrently (e.g., `xela_server2_2f`, `xela_server2_ah`) and applies sinusoidal variations to `integer` or `calibrated`
values after an initial warm-up period.

## 2. Goals
- Provide a repeatable WebSocket test stream to validate `xela_server2_2f` end-to-end.
- Support selectable model references via a parameter.
- Introduce realistic signal variation after a configurable delay.

## 3. Inputs
### 3.1 Parameters
- `model_name` (string, required)
  - Example: `uSPa35`, `uSPa46`, `XR23AHLCPP_left`
- `ref_dir` (string, default: `<pkg_share>/resource`)
- `bind_host` (string, default: `0.0.0.0`)
- `bind_port` (int, default: 5000)
- `publish_period_ms` (int, default: 200)
- `warmup_sec` (double, default: 5.0)
- `use_calibrated_if_present` (bool, default: true)
- `integer_xy_range` (double, default: 1350.0)
- `integer_z_range` (double, default: 10085.0)
- `calib_xy_range` (double, default: 0.8)
- `calib_z_range` (double, default: 14.0)
- `sine_freq_hz` (double, default: 0.05)
- `sine_phase_step` (double, default: 0.1)
- `preset` (string, default: `normal`)
  - Uses `replayer_presets.<preset>` from a params YAML when available.
- `variation_mode` (string, default: `random`)
  - `random` generates independent x/y/z per sensor within the configured ranges.
  - `sine` uses the sinusoidal variation.
- `random_seed` (int, default: 0)
  - 0 uses a non-deterministic seed.
- `random_global_strength` (double, default: 0.3)
- `random_local_strength` (double, default: 0.7)
- `random_temporal_alpha` (double, default: 0.7)
  - 0.0 disables temporal smoothing; closer to 1.0 is smoother.

## 4. Behavior
### 4.1 Message loading
- Load JSON file: `${ref_dir}/${model_name}.json`.
- If missing or invalid, log error and exit.

### 4.2 Publishing
- Listen on `ws://<bind_host>:<bind_port>`.
- When one or more clients connect, send the same JSON text to all connected clients at `publish_period_ms` intervals (broadcast).
- If one client disconnects, continue streaming to remaining clients and accept new connections.

### 4.3 Signal variation
- For `warmup_sec`, send unmodified JSON.
- After `warmup_sec`:
  - If `variation_mode=random`:
    - If `use_calibrated_if_present=true` and `calibrated` exists:
      - Random variation uses global + local components per module and temporal smoothing.
      - X/Y within ±0.8 N, Z within 0..14 N around baseline.
    - Else random integer values within:
      - Random variation uses global + local components per module and temporal smoothing.
      - X/Y: baseline ±1350
      - Z: baseline .. baseline+10085
  - If `variation_mode=sine`:
    - If `use_calibrated_if_present=true` and `calibrated` exists:
      - Apply sine-wave variation to calibrated values.
        - X/Y range: ±0.8 N
        - Z range: 0..14 N
    - Else apply sine-wave variation to integer values:
      - X/Y range: baseline ±1350
      - Z range: baseline .. baseline+10085
- Baseline is the original value in the JSON.
- Each taxel axis uses `sin(2π f t + phase)` with `phase = idx * sine_phase_step` when in `sine` mode.

## 5. Output
- WebSocket stream of JSON messages broadcast to all connected clients.

## 6. Constraints
- The node is packaged within `sim_xela_server` and can run standalone.
- The JSON structure is assumed to match the existing ref files.

## 7. Acceptance Criteria
- Node starts, loads JSON, and begins transmitting at configured rate.
- Two or more clients can connect simultaneously and receive the same stream without blocking each other.
- For the first `warmup_sec`, transmitted message matches the reference JSON.
- After warmup, values change sinusoidally within the specified ranges.
- If `calibrated` exists and enabled, calibrated values change; otherwise integer values change.

## 8. Notes
- `temp` is not modified.
- If calibrated arrays are missing or empty, fallback to integer variation.
