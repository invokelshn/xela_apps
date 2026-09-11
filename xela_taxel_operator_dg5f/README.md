# xela_taxel_operator_dg5f

Standalone, operator-only web app for the DG5F taxel sensor visualization — a separate URL
operators/demo staff can open (`http://localhost:8766`) that shows the exact same 124-taxel sensor
markers as the Admin sidecar, but with none of the Admin-only controls (2D grid view, follow-cam,
raw debug panels).

## Why this exists

`xela_taxel_sidecar_dg5f` (the original combined Admin+Operator taxel sidecar, `http://localhost:8765`)
remains **unmodified** — this package is fully independent of it and does not depend on or replace it.
It runs its own dedicated `rosbridge_server` and web server so it can be reached (and torn down)
without touching the Admin sidecar, and both can run at the same time without port conflicts.

## Ports

| | Admin sidecar (`xela_taxel_sidecar_dg5f`) | Operator (`xela_taxel_operator_dg5f`) |
|---|---|---|
| Web UI | 8765 | 8766 |
| rosbridge | 9090 | 9092 |
| (taxel viewer rosbridge) | 9091 | — (shares Admin's, or its own via `xela_atag_taxel_viewer` include) |

Ports are launch args (`web_port`, `rosbridge_port`, etc.) and can be overridden.

## What it's built from

- `xela_taxel_viz_core` — the shared rendering core (rosbridge client, 3D renderer, TF resolution +
  taxel marker rendering), loaded via `<script>` tags, unmodified.
- `xela_atag_taxel_viewer` — session capture, alert cards, filmstrip, and status-bar widgets
  (`op-viz-controls`, `op-modsel`, `taxel_session_panel.js`, etc.), unmodified.
- `xela_taxel_web_bridge_node` and `xela_atag_taxel_viewer_node` — the C++/Python nodes that
  actually produce the taxel marker payload (`/x_taxel_dg5f/web_state`) and live module data. These
  executables live in `xela_taxel_sidecar_dg5f` / `xela_atag_taxel_viewer` respectively and are
  **reused as-is** (exec_depend, no source changes) — see `enable_data_bridge_nodes` below.

Only `xela_taxel_operator_dg5f/web/index.html` (this package) contains new code — it assembles the
above pieces with an operator-focused layout (CSS/DOM only) and no Admin-only widgets.

## Launch arg: `enable_data_bridge_nodes`

- `true` (default) — this package starts its own `xela_taxel_web_bridge_node` and
  `xela_atag_taxel_viewer_node`, so it can run **standalone** (without the Admin sidecar/baseline
  stack running at all) and still show live taxel data.
- `false` — skip starting those nodes, because they're already running elsewhere (e.g. the Admin
  sidecar's own launch file already starts them). Use this when launching Admin + Operator
  together (see `ur7e_xdg5f_atag_right_sim_dev/launch/xela_driver_dev.launch.py`) to avoid
  duplicate/conflicting node instances.

## Running

```bash
ros2 launch xela_taxel_operator_dg5f xela_taxel_operator_dg5f.launch.py
```

Then open `http://localhost:8766`.

To run alongside the Admin sidecar (both reachable at once), use the
`ur7e_xdg5f_atag_right_sim_dev` robot config package, which launches both with
`enable_data_bridge_nodes:=false` for this package.
