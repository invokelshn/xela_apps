# xela_taxel_viz_core

Shared, dependency-free JavaScript core for rendering Xela taxel sensor data in a browser: rosbridge
connection handling, the URDF/mesh 3D renderer (Three.js), and the TF-resolution + taxel-marker
rendering pipeline (`taxel_marker_renderer.js`, ported verbatim from `xela_taxel_sidecar_dg5f`'s
`index.html`).

This package has **no ROS nodes** and does **not** launch anything on its own — it only installs
static web assets (`web/js/...`, `web/vendor/...`) that other packages load with a `<script>` tag.

## Relationship to `xela_taxel_sidecar_dg5f`

`xela_taxel_sidecar_dg5f` (the combined Admin+Operator DG5F taxel sidecar) is **not modified** and
does **not** depend on this package — the two are unrelated. `xela_taxel_viz_core` was extracted by
copying the relevant rendering/TF-resolution logic out of `xela_taxel_sidecar_dg5f`'s `index.html`
so it can be reused by a separate, operator-only web app (`xela_taxel_operator_dg5f`) without
duplicating or reimplementing that logic.

## Contents

- `web/js/core/` — `app_state.js`, `rosbridge_client.js`, `runtime_config.js`, `ui_feedback.js`,
  `vector_smoothing.js`
- `web/js/render/` — `grid.js`, `grid_vectors.js`, `primitives.js`, `urdf_mesh_renderer.js`,
  `taxel_marker_renderer.js` (TF resolution + 124-taxel marker rendering)
- `web/vendor/` — Three.js and loaders (Collada, STL, TGA, OrbitControls)
- `web/demo/` — standalone demo pages used during development to verify the core renders taxel
  markers correctly against a live rosbridge/TF stream (not part of the production operator UI)

## Consumers

- `xela_taxel_operator_dg5f` — loads this core's JS directly via `<script>` tags to build the
  operator-only 3D view.

## Build

Pure static-asset package (`ament_cmake`, no compiled code):

```bash
colcon build --packages-select xela_taxel_viz_core
```
