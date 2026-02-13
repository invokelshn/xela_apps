# xela_server2_ah

ROS 2 WebSocket client that converts Xela AH JSON into `xela_taxel_msgs/XTaxelSensorTArray`
and publishes it on `/x_taxel_ah` for Allegro Hand (left/right).

## Build
```bash
source /opt/ros/humble/setup.bash
cd /home/invokelee/xela_robotics/02_dev_ws
colcon build --packages-select xela_server2_ah
```

## Run (WebSocket mode)
```bash
source /opt/ros/humble/setup.bash
source /home/invokelee/xela_robotics/02_dev_ws/install/setup.bash
ros2 run xela_server2_ah xela_server2_ah_node --ros-args \
  -p ws_host:=localhost \
  -p ws_port:=5000 \
  -p frame_ids_yaml:=/home/invokelee/xela_robotics/02_dev_ws/src/xela_apps/xela_server2_ah/config/server2_ah_config.yaml \
  -p hand_side:=left
```

## Run (File playback mode)
```bash
source /opt/ros/humble/setup.bash
source /home/invokelee/xela_robotics/02_dev_ws/install/setup.bash
ros2 run xela_server2_ah xela_server2_ah_node --ros-args \
  -p input_json_path:=/home/invokelee/xela_robotics/02_dev_ws/src/xela_apps/sim_xela_server/resource/XR23AHLCPP.json \
  -p playback_interval_ms:=200 \
  -p playback_loop:=true \
  -p hand_side:=left
```

## Run (ROS time override)
```bash
source /opt/ros/humble/setup.bash
source /home/invokelee/xela_robotics/02_dev_ws/install/setup.bash
ros2 run xela_server2_ah xela_server2_ah_node --ros-args \
  -p input_json_path:=/home/invokelee/xela_robotics/02_dev_ws/src/xela_apps/sim_xela_server/resource/XR23AHLCPP.json \
  -p use_ros_time_for_sensor_time:=true \
  -p hand_side:=left
```

## Launch
```bash
source /opt/ros/humble/setup.bash
source /home/invokelee/xela_robotics/02_dev_ws/install/setup.bash
ros2 launch xela_server2_ah xela_server2_ah.launch.py
```

## Launch (Replayer + xela_server2_ah)
Uses `sim_xela_server` with `XR23AHLCPP.json` (left) or `XR23AHRCPP.json` (right).
```bash
source /opt/ros/humble/setup.bash
source /home/invokelee/xela_robotics/02_dev_ws/install/setup.bash
ros2 launch xela_server2_ah xela_server2_ah_with_replayer.launch.py \
  model_name:=XR23AHLCPP
```

## Launch (Replayer + xela_server2_ah + xela_taxel_viz_ahv4)
```bash
source /opt/ros/humble/setup.bash
source /home/invokelee/xela_robotics/02_dev_ws/install/setup.bash
ros2 launch xela_server2_ah xela_server2_ah_with_replayer_and_viz.launch.py \
  model_name:=XR23AHLCPP
```

## Verify
```bash
source /opt/ros/humble/setup.bash
source /home/invokelee/xela_robotics/02_dev_ws/install/setup.bash
ros2 topic echo /x_taxel_ah --once
```

## Notes
- `frame_ids_yaml` should point to `server2_ah_config.yaml`, which references
  `server_model_joint_map.yaml` via `mapping_yaml`.
- `hand_side` controls the prefix mapping (`x_taxel_0_` for left, `x_taxel_1_` for right).
- Input arrays (`integer`, `calibrated`, `temp`) are mapped by joint order from
  `server_model_joint_map.yaml`.
- Replayer server lives in `sim_xela_server` and must be built/available.
