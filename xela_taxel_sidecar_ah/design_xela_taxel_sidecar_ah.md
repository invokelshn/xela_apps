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

## 5. Risks
- wrong mapping/pattern combination for selected model
- optional viz service disabled while user expects RViz mode switch side-effects
- TF frame mismatch in URDF mode when fixed frame/remaps are incorrect

## 6. Validation
- launch with left and right models
- verify output topic and web updates
- verify mode toggling behavior with `use_viz_set_mode` off/on
- verify URDF points in bridge payload when enabled
