# xela_server2_dg5f

ROS 2 WebSocket client that converts XELA taxel server JSON into
`xela_taxel_msgs/XTaxelSensorTArray` and publishes it on `/x_taxel_dg5f`, for the
XELA-taxel-equipped Tesollo DG-5F hand (both hands).

Forked from `xela_server2_ah` (Allegro Hand), following the same per-hand-package
convention already established by `xela_server2_2f` (2F gripper) - the core
websocket/JSON-parsing/joint-map logic (`dg5f_parser.cpp`, `dg5f_joint_map.cpp`,
the websocket/playback loop in `dg5f_node.cpp`) is copied verbatim with zero logic
changes; only package/node/topic naming and the joint-map YAML data are DG-5F's own
(`config/r_server_model_joint_map.yaml`, matching `x_dg5f_hand.xacro`'s sensor
housing link names, e.g. `f3_dg5f_ft`, `palm_uSPa46`).

## Build
```bash
moveit_pro build
```

## Run (WebSocket mode)
```bash
ros2 run xela_server2_dg5f xela_server2_dg5f_node --ros-args \
  -p ws_host:=localhost \
  -p ws_port:=5000 \
  -p hand_side:=right
```
`frame_ids_yaml` defaults to this package's own `config/r_server_model_joint_map.yaml`
(via `hand_side`); override with `-p frame_ids_yaml:=<path>` for a different mapping file.

## Run (File playback mode)
```bash
ros2 run xela_server2_dg5f xela_server2_dg5f_node --ros-args \
  -p input_json_path:=<path/to/recorded.json> \
  -p playback_interval_ms:=200 \
  -p playback_loop:=true
```

## Known gaps
- The Allegro-side `_with_replayer_and_viz.launch.py` variant (which wires up
  `xela_taxel_viz_ahv4` and Allegro-specific pattern YAMLs/URDF xacros) was not ported -
  there is no DG-5F equivalent viz package yet. A DG-5F viz integration is planned once
  a DG-5F reference JSON resource lands under `sim_xela_server`'s `resource/` directory
  (expected next week); `xela_server2_dg5f.launch.py` and
  `_with_server.launch.py`/`_with_replayer.launch.py` (mock-server-driven, no viz) were
  ported as-is since they have no Allegro-specific coupling.
- `x_dg5f_hand.xacro`'s `side="left"` sensor placements (uSPa22/fingertip/uSPa46 offsets)
  are only tuned/verified for the right hand so far, even though this package's left-hand
  joint map (`config/l_server_model_joint_map.yaml`) is now available.
