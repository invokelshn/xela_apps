# xela_server2_2f

## Build
```bash
source /opt/ros/humble/setup.bash
cd /home/invokelee/xela_robotics/02_dev_ws
colcon build --packages-select xela_server2_2f
```

## Run (WebSocket mode)
```bash
source /opt/ros/humble/setup.bash
source /home/invokelee/xela_robotics/02_dev_ws/install/setup.bash
ros2 run xela_server2_2f xela_server2_2f_node --ros-args \
  -p ws_host:=localhost \
  -p ws_port:=5000 \
  -p frame_ids_yaml:=/home/invokelee/xela_robotics/02_dev_ws/src/xela_apps/xela_server2_2f/config/server2_2f_config.yaml
```
If the incoming JSON lacks `calibrated`, the node uses `integer` values to fill `taxels` and leaves `forces` empty.

## Run (File playback mode)
```bash
source /opt/ros/humble/setup.bash
source /home/invokelee/xela_robotics/02_dev_ws/install/setup.bash
ros2 run xela_server2_2f xela_server2_2f_node --ros-args \
  -p input_json_path:=/home/invokelee/my_moveit_pro/01_mpro_dev_ws/tmp/websocket.json \
  -p playback_interval_ms:=200 \
  -p playback_loop:=true
```

## Run (ROS time override)
```bash
source /opt/ros/humble/setup.bash
source /home/invokelee/xela_robotics/02_dev_ws/install/setup.bash
ros2 run xela_server2_2f xela_server2_2f_node --ros-args \
  -p input_json_path:=/home/invokelee/my_moveit_pro/01_mpro_dev_ws/tmp/websocket.json \
  -p use_ros_time_for_sensor_time:=true
```

## WebSocket Replayer Server (test)
Starts a WebSocket server that replays a reference JSON to connected clients.
```bash
source /opt/ros/humble/setup.bash
source /home/invokelee/xela_robotics/02_dev_ws/install/setup.bash
ros2 run sim_xela_server sim_xela_server_node --ros-args \
  -p model_name:=XR23AHLCPP_left \
  -p bind_port:=5000
```
The server listens on `ws://<bind_host>:<bind_port>` (default `0.0.0.0:5000`).

## Verify
```bash
source /opt/ros/humble/setup.bash
source /home/invokelee/xela_robotics/02_dev_ws/install/setup.bash
ros2 topic echo /x_taxel_2f --once
```

## Tests
```bash
source /opt/ros/humble/setup.bash
cd /home/invokelee/xela_robotics/02_dev_ws
colcon test --packages-select xela_server2_2f --event-handlers console_cohesion+
colcon test-result --all --verbose
```

## Combined Launch
```bash
source /opt/ros/humble/setup.bash
source /home/invokelee/xela_robotics/02_dev_ws/install/setup.bash
ros2 launch xela_server2_2f xela_server2_2f_with_server.launch.py \
  can_port:=can0 \
  xela_server_exec:=/etc/xela/xela_server
```

## CAN Setup Script
If you prefer pre-configuring CAN before launching, run:
```bash
source /home/invokelee/xela_robotics/02_dev_ws/install/setup.bash
$(ros2 pkg prefix xela_server2_2f)/share/xela_server2_2f/scripts/can_setup.sh can0
```
