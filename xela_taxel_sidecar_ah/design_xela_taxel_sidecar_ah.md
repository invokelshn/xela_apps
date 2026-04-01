# Design
# xela_taxel_sidecar_ah

## 1. Overview and design objective
The AH sidecar design addresses higher taxel density and model-side mapping complexity compared to 2F.

Design intent:
- keep a stable sidecar service/topic contract while adding AH mapping/pattern semantics
- decouple bridge mode switching from optional viz-node mode service availability
- preserve compatibility with `std_xela_taxel_viz_ahv4` namespace-local TF conventions

This enables incremental AH web visualization improvements without forcing changes in tactile producer or std-viz packages.

## 2. Bridge design
### 2.1 Mapping model
- mapping file provides flat index and frame-name contract
- pattern file provides 2D grid projection of active indices
- hand-side auto resolution switches mapping prefix conversion and default pattern

### 2.2 Signal model
- supports force-first rendering with taxel fallback
- baseline and deadband applied independently for force and taxel paths
- optional module/group filtering (`show_thumb`, `show_module_XX`)

### 2.3 URDF point model
- enabled by `emit_urdf_points`
- frame lookup with TF cache (`tf_cache_ttl_sec`)
- optional frozen URDF coordinates (`freeze_urdf_positions`)

## 3. Mode manager design
- service endpoint fixed at `/xela_viz_mode_manager/set_mode`
- always updates bridge params
- viz service call is controlled by:
  - `use_viz_set_mode`
  - `require_viz_set_mode_service`

This avoids hard failure when integration target does not expose `set_mode`.

## 4. Launch design
`xela_taxel_sidecar_cpp.launch.py`:
- resolves hand/mapping/pattern at launch time (`OpaqueFunction`)
- sets sidecar transport and mode manager options
- supports namespace-local TF remap for std viz integration

`xela_taxel_sidecar.launch.py`:
- thin wrapper for cpp launch

## 5. Analysis feature design

### 5.1 Purpose
Analysis provides real-time event detection and a scrolling timeline visualization for tactile interactions on the Allegro Hand.  It surfaces contact onset/release, slip warnings, and explicit grasp-phase markers in a compact sparkline panel without blocking the primary visualization modes.

### 5.2 Default state
Analysis is **enabled by default** on page load (`setAnalysisVisible(true)`).  The panel can be toggled at any time via the **Analysis** toolbar button.

### 5.3 State model

| Object | Key fields | Description |
|---|---|---|
| `analysisState` | `visible: true` | Panel visibility flag |
| `analysisEventState` | `contactThreshold: 0.14`, `lastContactActive`, `lastSlipActive`, `currentPhase` | Event tracking across frames |
| `timelineState` | `samplePeriodMs: 100`, `windowMs: 30000`, `points[]`, `events[]` | Rolling telemetry ring buffer |

### 5.4 Telemetry computation (`computeAnalysisTelemetry`)
Called every frame on the incoming bridge payload.  Derives per-frame statistics from `payload.urdf.points` (falling back to `payload.grid.points`):

- `normMean` — mean force magnitude across all sensor points
- `fzSum` — sum of vertical (Z) forces
- `activeCount` — count of points exceeding activation threshold 0.12
- `tip03Norm` — peak force on module 03 (thumb tip)
- `tip13Norm` — peak force on module 13 (index tip)

### 5.5 Event classification (`ingestAnalysisEvents`)
Three event source paths are merged into a unified event list:

**Implicit contact events** (inferred from tip force levels):
- `contact_detected` — both `tip03Norm` and `tip13Norm` ≥ `contactThreshold` (0.14), fires only during grasp/pinch phases
- `contact_released` — transition out of active contact state

**Implicit slip events** (from `payload.meta.slip`):
- `slip_warning` — slip reflex output active (`reflex_output_active`)
- `slip_active` — slip condition sustained; tracked by trigger count changes

**Explicit grasp events** (from `payload.meta.grasp_event`):
- Grasp/pinch phase markers (approach, grasp, retreat, home)
- Recovery, abort, success, and impedance/filter change indicators
- Explicit events suppress nearby implicit events within a ±450 ms suppression window to avoid duplicate annotation

Each event record carries: `label`, `kind`, `tag` (single char), `priority`, `explicit`, and `timestampMs`.

### 5.6 Timeline UI
- **Sparkline canvas** (`id="sparkline"`, 500 × 34 px) — plots `normMean` with event markers overlaid as coloured vertical ticks
- **Window controls** — 10 s / 30 s (default) / 60 s rolling window
- **Pause/Resume** — freezes the scrolling view for inspection
- **Event tooltip** (`id="eventTooltip"`) — shown on hover; displays label, phase, source, and elapsed time since event

### 5.7 Visibility control flow
```
setAnalysisVisible(visible)
  → statsRow.style.display = visible ? "" : "none"
  → analysisBtn.classList.toggle("active", visible)
  → if (!visible) setEventTooltipVisible(false)
```

### 5.8 Payload metadata dependencies
| Feature | Required bridge field | Enable condition |
|---|---|---|
| Slip events | `payload.meta.slip` | `include_slip_meta_` = true in bridge node |
| Grasp events | `payload.meta.grasp_event` | `include_grasp_event_meta_` = true in bridge node |
| Contact inference | `payload.urdf.points` or `payload.grid.points` | Always available |

---

## 6. FollowCam feature design

### 6.1 Purpose
FollowCam provides an automatic camera that continuously tracks the **thumb tip (module 03)** and **index tip (module 13)** of the Allegro Hand in RobotModel (URDF) mode.  It eliminates the need for manual camera repositioning during dexterous manipulation tasks.

### 6.2 Availability constraints
- Only active when `state.uiMode === "robot"` (RobotModel mode)
- Button is rendered in the toolbar at all times but is `disabled` when not in RobotModel mode
- Automatically deactivated and camera reset to auto-frame when switching away from RobotModel mode

### 6.3 State model (`followCamState`)

| Field | Default | Description |
|---|---|---|
| `enabled` | `false` | Master on/off flag |
| `manualOrbitActive` | `false` | True while user is orbiting; pauses automatic update |
| `captureViewOnNextUpdate` | `false` | Requests user offset capture on next update cycle |
| `tipModule03Regex` | `/_[01]_03_/` | TF frame selector for thumb tip |
| `tipModule13Regex` | `/_[01]_13_/` | TF frame selector for index tip |
| `tipModule51/52/53Regex` | — | TF frame selectors for palm reference sensors |
| `forwardDistance` | `-0.20` | Camera standoff distance along forward axis (m) |
| `heightOffset` | `0.10` | Camera height offset above target base (m) |
| `lookAhead` | `0.028` | Look-ahead offset applied to camera target point (m) |
| `positionLerp` | `0.24` | Smoothing factor for camera position |
| `targetLerp` | `0.26` | Smoothing factor for camera look-at point |
| `forwardLerp` | `0.22` | Smoothing factor for forward direction vector |
| `maxCamStep` | `0.030` | Maximum camera position movement per update (m) |
| `maxTargetStep` | `0.024` | Maximum look-at target movement per update (m) |
| `jumpRejectDist` | `0.16` | Distance threshold for jump-rejection filter (m) |
| `jumpRejectLimit` | `2` | Consecutive outlier frames before accepting new position |

URL query parameters can override `forwardDistance`, `heightOffset`, `lookAhead`, and `forwardSign` at page load (`runtime_config.js`).

### 6.4 Camera update algorithm (`updateFollowCamera`)

Called every render frame when `followCamState.enabled === true`.

```
1. Guard: URDF renderer must be ready
2. Frame resolution
   - resolve tip03 center  ← module 03 taxel frames via regex
   - resolve tip13 center  ← module 13 taxel frames via regex
   - resolve palm centers  ← modules 51, 52, 53 (optional)
3. Target base = midpoint(tip03, tip13)
4. Jump rejection
   - if |targetBase − filteredTargetBase| > jumpRejectDist
     increment jumpRejectCount; skip update if count < jumpRejectLimit
   - else reset jumpRejectCount
5. Forward direction
   - preferred: palm-centroid → target base vector (robust to wrist rotation)
   - fallback: cross(tip03 − tip13, world-up)
   - apply forwardLerp smoothing; enforce minForwardDeg deadband
6. Local basis construction: right = cross(forward, world-up); up = cross(right, forward)
7. User orbit capture
   - on manualOrbitActive → captureViewOnNextUpdate = true
   - on captureViewOnNextUpdate: decompose current camera into local basis offsets
     (userCameraOffsetLocal, userTargetOffsetLocal)
8. Camera goal computation
   - if user offsets captured: apply offsets in current local basis
   - else: cameraGoal = targetBase + forward * forwardDistance + up * heightOffset
           targetGoal = targetBase + forward * lookAhead
9. Smooth interpolation with per-step clamping
   - lerp camera position; clamp to maxCamStep
   - lerp look-at target; clamp to maxTargetStep
10. Apply to Three.js OrbitControls
```

### 6.5 Manual orbit interaction
When the user drags/scrolls the 3D view while FollowCam is active:
- `manualOrbitActive` is set to `true`
- On the next FollowCam update, the current camera offset relative to the tracked target is captured into `userCameraOffsetLocal` / `userTargetOffsetLocal`
- Subsequent updates apply these offsets in the continuously updated local frame, so the user's preferred viewpoint is maintained while still tracking finger movement

### 6.6 Enable/disable flow
```
setFollowCamEnabled(true)
  → followCamState.enabled = true
  → reset manualOrbitActive, jumpRejectCount, filteredTargetBase, filteredForward
  → hint: "FollowCam enabled: thumb/index fingertip tracking."
  → applyOrbitProfileForMode()
  → updateModeButtons()

setFollowCamEnabled(false)
  → followCamState.enabled = false
  → hint: "FollowCam disabled: RobotModel fixed orbit mode."
  → autoFrameUrdfMeshCamera(true)   ← reset to auto-frame view
  → applyOrbitProfileForMode()
  → updateModeButtons()
```

### 6.7 URL configuration parameters

| Parameter | Default | Description |
|---|---|---|
| `follow_cam_forward_distance` | `-0.20` | Camera standoff from target along forward axis (m) |
| `follow_cam_height_offset` | `0.10` | Camera height above target base (m) |
| `follow_cam_look_ahead` | `0.028` | Look-ahead shift of camera target point (m) |
| `follow_cam_forward_sign` | `-1.0` | Sign of forward vector; flip for different hand orientations |

---

## 7. Risks
- wrong mapping/pattern combination for selected model
- optional viz service disabled while user expects RViz mode switch side-effects
- TF frame mismatch in URDF mode when fixed frame/remaps are incorrect
- Analysis slip/grasp events silent if bridge metadata subscriptions not enabled (`include_slip_meta_`, `include_grasp_event_meta_`)
- FollowCam tip-frame resolution fails silently if URDF is not fully loaded or TF frames are missing; camera freezes at last valid position
- FollowCam jump-rejection may delay tracking recovery after rapid large motions

## 8. Validation
- launch with left and right models
- verify output topic and web updates
- verify mode toggling behavior with `use_viz_set_mode` off/on
- verify URDF points in bridge payload when enabled
- Analysis: confirm panel visible on page load; verify contact/slip events annotated on sparkline during grasp sequence
- Analysis: toggle panel off/on; confirm tooltip appears on hover over event markers
- Analysis: enable bridge `include_slip_meta_` and `include_grasp_event_meta_`; verify explicit grasp events suppress implicit contact events within 450 ms window
- FollowCam: enter RobotModel mode; verify button enabled; click to activate; confirm camera tracks thumb/index tips
- FollowCam: manual orbit while FollowCam active; confirm user offset is captured and maintained across subsequent frames
- FollowCam: switch away from RobotModel; confirm FollowCam deactivates and camera resets to auto-frame
- FollowCam: pass `follow_cam_forward_distance` URL param; verify camera standoff distance matches supplied value
