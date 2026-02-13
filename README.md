# Xela Apps

`xela_apps` is a collection of ROS 2 packages for the Xela taxel data pipeline (simulator, WebSocket bridge, and RViz visualization).

Development split policy:
- For 2F Gripper work, prioritize `*_2f` packages and configs.
- For Allegro Hand work, prioritize `*_ah` and `*_ahv4` packages and configs.

## Workspace (Generalized)

Use a variable so workspace naming stays generic.

```bash
export XELA_WS=~/xela_robotics/<your_ws>
source /opt/ros/humble/setup.bash
source $XELA_WS/install/setup.bash
```

## Package Map

| Path | Type | Role | Main Topic |
|---|---|---|---|
| `sim_xela_server` | ROS 2 package | JSON replayer WebSocket server | ws://`<host>`:`<port>` |
| `xela_server2_2f` | ROS 2 package | 2F JSON -> ROS bridge | `/x_taxel_2f` |
| `xela_taxel_viz_2f` | ROS 2 package | 2F RViz visualization (grid/urdf) | `/x_taxel_2f/markers` |
| `xela_server2_ah` | ROS 2 package | Allegro JSON -> ROS bridge | `/x_taxel_ah` |
| `xela_taxel_viz_ahv4` | ROS 2 package | Allegro RViz visualization (grid/urdf/demo) | `/x_taxel_ah/markers` |
| `xela_taxel_docs` | docs | Sensor manuals and spec sheets (PDF) | - |

## Common Build

```bash
cd $XELA_WS
colcon build --packages-select \
  sim_xela_server \
  xela_server2_2f xela_taxel_viz_2f \
  xela_server2_ah xela_taxel_viz_ahv4
source $XELA_WS/install/setup.bash
```

## 2F Real Hardware (`uSPr2F`)

Terminal 1:
```bash
ros2 launch xela_taxel_viz_2f real_all_svc_xela_taxel_viz_2f.launch.py model_name:=uSPr2F style_preset:=cool_steel viz_mode:=urdf
```

or

Terminal 1:
```bash
ros2 launch xela_taxel_viz_2f real_all_svc_xela_taxel_viz_2f.launch.py model_name:=uSPr2F style_preset:=cool_steel viz_mode:=grid
```

## 2F Simulation Examples (`*_2f`)

Note: currently available 2F models in this package are `uSPr2F`, `uSPrDS`, and `uSPrHE35`.

### uSPr2F

URDF mode:
```bash
ros2 launch xela_taxel_viz_2f sim_all_svc_xela_taxel_viz_2f.launch.py model_name:=uSPr2F style_preset:=cool_steel viz_mode:=urdf
```

Grid mode:
```bash
ros2 launch xela_taxel_viz_2f sim_all_svc_xela_taxel_viz_2f.launch.py model_name:=uSPr2F style_preset:=cool_steel viz_mode:=grid
```

### uSPrDS

URDF mode:
```bash
ros2 launch xela_taxel_viz_2f sim_all_svc_xela_taxel_viz_2f.launch.py model_name:=uSPrDS style_preset:=cool_steel viz_mode:=urdf
```

Grid mode:
```bash
ros2 launch xela_taxel_viz_2f sim_all_svc_xela_taxel_viz_2f.launch.py model_name:=uSPrDS style_preset:=cool_steel viz_mode:=grid
```

### uSPrHE35

URDF mode:
```bash
ros2 launch xela_taxel_viz_2f sim_all_svc_xela_taxel_viz_2f.launch.py model_name:=uSPrHE35 style_preset:=cool_steel viz_mode:=urdf
```

Grid mode:
```bash
ros2 launch xela_taxel_viz_2f sim_all_svc_xela_taxel_viz_2f.launch.py model_name:=uSPrHE35 style_preset:=cool_steel viz_mode:=grid
```

## Allegro Simulation Examples (`*_ah`, `*_ahv4`)

### XR23AHRCPP (Right)

URDF mode:
```bash
ros2 launch xela_taxel_viz_ahv4 sim_all_svc_xela_taxel_viz_ahv4.launch.py model_name:=XR23AHRCPP viz_mode:=urdf
```

Grid mode:
```bash
ros2 launch xela_taxel_viz_ahv4 sim_all_svc_xela_taxel_viz_ahv4.launch.py model_name:=XR23AHRCPP viz_mode:=grid
```

### XR23AHLCPP (Left)

URDF mode:
```bash
ros2 launch xela_taxel_viz_ahv4 sim_all_svc_xela_taxel_viz_ahv4.launch.py model_name:=XR23AHLCPP viz_mode:=urdf
```

Grid mode:
```bash
ros2 launch xela_taxel_viz_ahv4 sim_all_svc_xela_taxel_viz_ahv4.launch.py model_name:=XR23AHLCPP viz_mode:=grid
```

## Verification

```bash
ros2 topic echo /x_taxel_2f --once
ros2 topic echo /x_taxel_ah --once
```

## Notes

- `xela_taxel_viz_ahv4` loads the hand controller YAML from `config/ros2_controller_hand_xela_taxel_viz_ahv4.yaml`.
- Use `model_name:=XR23AHRCPP` for right hand and `model_name:=XR23AHLCPP` for left hand.
- If `ws_port:=5000` is already in use, replayer/bridge nodes may fail to start.

## Documents

Package-level docs:
- `sim_xela_server/README.md`
- `xela_server2_2f/README.md`
- `xela_server2_ah/README.md`
- `xela_taxel_viz_2f/README.md`
- `xela_taxel_viz_ahv4/README.md`

Sensor docs:
- `xela_taxel_docs/XELA_software_manual_1.8.0_208601.pdf`
- `xela_taxel_docs/Xela_Skin_4x6_Manual_Aug_2023.pdf`
- `xela_taxel_docs/uSPa46_Specsheet_Jul_2025.pdf`
