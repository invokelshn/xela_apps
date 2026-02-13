# xela_taxel_viz_ahv4

RViz2 visualization for `/x_taxel_ah` (`xela_taxel_msgs/XTaxelSensorTArray`).
Supports both grid and URDF marker modes for Allegro Hand (AHv4) left/right.

## Build
```
cd ~/xela_robotics/02_dev_ws
colcon build --packages-select xela_taxel_viz_ahv4
```

## Run
Grid mode (default):
```
source ~/xela_robotics/02_dev_ws/install/setup.bash
ros2 launch xela_taxel_viz_ahv4 xela_taxel_viz_ahv4.launch.py viz_mode:=grid
```

URDF mode:
```
source ~/xela_robotics/02_dev_ws/install/setup.bash
ros2 launch xela_taxel_viz_ahv4 xela_taxel_viz_ahv4.launch.py viz_mode:=urdf
```

## Demo (no /x_taxel_ah source yet)
Starts a synthetic publisher and the visualizer.
```
source ~/xela_robotics/02_dev_ws/install/setup.bash
ros2 launch xela_taxel_viz_ahv4 xela_taxel_viz_ahv4_demo.launch.py
```

Optional: set demo publish rate
```
ros2 launch xela_taxel_viz_ahv4 xela_taxel_viz_ahv4_demo.launch.py publish_rate_hz:=10.0
```

## Sim (replayer + server + viz)
```
source ~/xela_robotics/02_dev_ws/install/setup.bash
ros2 launch xela_taxel_viz_ahv4 sim_all_svc_xela_taxel_viz_ahv4.launch.py \
  model_name:=XR23AHLCPP viz_mode:=urdf
```

## RViz2
Grid mode fixed frame should be `x_taxel_ah_viz`.
URDF mode uses the URDF frames (e.g., `world`/`hand_root`).

`sim_all_svc_xela_taxel_viz_ahv4.launch.py` selects RViz configs automatically:
- grid: `config/grid_xela_taxel_viz_ah.rviz`
- urdf: `config/urdf_xela_taxel_viz_ah.rviz`

## Pattern Conversion (CSV -> YAML)
```
~/xela_robotics/02_dev_ws/install/xela_taxel_viz_ahv4/lib/xela_taxel_viz_ahv4/csv_to_pattern_yaml.py \
  pattern.csv -o pattern.yaml
```

## Key Parameters
See `config/xela_taxel_viz_ahv4.yaml` for defaults.
- `mapping_yaml`: `taxel_joint_map_new.yaml`
- `pattern_yaml`: `pattern_lahv4.yaml`
- `use_forces_if_present`: prefer forces when available
- `xy_range`, `z_range`: normalization ranges
- `hand_side`: `left|right` (prefix rewrite for `x_taxel_0_` vs `x_taxel_1_`)
- `marker_stamp_mode`: `keep|now|zero`
- `marker_time_offset_sec`: default `-0.1` for TF stability

## Left/Right Notes
- `model_name:=XR23AHLCPP` (left) and `model_name:=XR23AHRCPP` (right).
- `pattern_lahv4.yaml` (left) and `pattern_rahv4.yaml` (right) are selected
  automatically by `sim_all_svc_xela_taxel_viz_ahv4.launch.py`.
