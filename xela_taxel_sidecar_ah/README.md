# xela_taxel_sidecar_ah

Web sidecar package for **Xela Allegro Hand (AH/AHv4)** tactile visualization.

## Components
- `xela_taxel_web_bridge_node` (C++)
- `xela_viz_mode_manager_node` (C++)
- `scripts/sidecar_http_server.py`
- optional sidecar rosbridge websocket
- static web UI (`web/taxel_sidecar`)

## Data flow
1. subscribe `/x_taxel_ah`
2. bridge maps taxels by `mapping_yaml + pattern_yaml`
3. publish `/x_taxel_ah/web_state` JSON
4. web UI subscribes through sidecar rosbridge
5. mode service `/xela_viz_mode_manager/set_mode` updates bridge mode

## Recommended launch
```bash
ros2 launch xela_taxel_sidecar_ah xela_taxel_sidecar_cpp.launch.py
```

Open:
- `http://localhost:8765`

## Main launch defaults (`xela_taxel_sidecar_cpp.launch.py`)
- `in_topic`: `/x_taxel_ah`
- `out_topic`: `/x_taxel_ah/web_state`
- `viz_mode`: `grid`
- `model_name`: `XR23AHLCPP`
- `hand_side`: `auto`
- `mapping_yaml`: auto from `std_xela_taxel_viz_ahv4`
- `pattern_yaml`: auto from `std_xela_taxel_viz_ahv4` (left/right by model)
- `style_preset`: `cool_steel`
- `fixed_frame`: `world`
- `cell_size`: `0.01`
- `origin_x`: `0.0`
- `origin_y`: `0.0`
- `max_publish_rate_hz`: `20.0`
- `emit_urdf_points`: `false`
- `freeze_urdf_positions`: `true`
- `enable_viz_mode_manager`: `true`
- `bridge_node_name`: `/xela_taxel_web_bridge_cpp`
- `viz_node_name`: `/xvizah/std_xela_taxel_viz_ahv4`
- `use_viz_set_mode`: `false`
- `require_viz_set_mode_service`: `false`
- `bridge_tf_topic`: `/tf`
- `bridge_tf_static_topic`: `/tf_static`
- `enable_web_server`: `true`
- `web_port`: `8765`
- `enable_sidecar_rosbridge`: `true`
- `sidecar_rosbridge_port`: `9090`

## Runtime mode switching
```bash
# grid -> urdf
ros2 service call /xela_viz_mode_manager/set_mode std_srvs/srv/SetBool "{data: true}"

# urdf -> grid
ros2 service call /xela_viz_mode_manager/set_mode std_srvs/srv/SetBool "{data: false}"
```

Note:
- default is bridge-only mode switching (`use_viz_set_mode=false`)
- enable viz service call only when target viz node supports `set_mode`

## Integration examples
With std AH viz namespace-local TF:
```bash
ros2 launch xela_taxel_sidecar_ah xela_taxel_sidecar_cpp.launch.py   viz_mode:=urdf   model_name:=XR23AHRCPP   bridge_tf_topic:=/xvizah/tf   bridge_tf_static_topic:=/xvizah/tf_static
```

With ur5e_xahr2c driver:
```bash
ros2 launch ur5e_xahr2c_config3 xela_driver.launch.py enable_taxel_sidecar:=true
```

## Build
```bash
cd ~/my_moveit_pro/01_wk_xela_mpro_dev_ws
moveit_pro build user_workspace --colcon-args "--packages-select xela_taxel_sidecar_ah"
```

## Troubleshooting
- mapping/pattern load error:
  - check resolved YAML paths and hand-side model selection
- no updates on web:
  - verify `/x_taxel_ah` and `/x_taxel_ah/web_state`
- mode switch no effect on std viz:
  - `use_viz_set_mode` is disabled by default
  - enable only when viz node supports `/set_mode`
- URDF points missing:
  - check `fixed_frame` and TF remap topics

## Documentation
- Functional spec: `xela_taxel_sidecar_ah.md`
- Design: `design_xela_taxel_sidecar_ah.md`
- Redmine: `README.textile`, `xela_taxel_sidecar_ah.textile`, `design_xela_taxel_sidecar_ah.textile`
