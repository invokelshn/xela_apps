# sim_xela_server

WebSocket replayer server for Xela taxel JSON data. This package provides a
standalone WebSocket **server** that streams JSON messages to connected clients
(e.g., `xela_server2_ah`, `xela_server2_2f`) and supports concurrent multi-client broadcast.

## Build
```bash
source /opt/ros/humble/setup.bash
cd /home/invokelee/xela_robotics/02_dev_ws
colcon build --packages-select sim_xela_server
```

## Run
```bash
source /opt/ros/humble/setup.bash
source /home/invokelee/xela_robotics/02_dev_ws/install/setup.bash
ros2 run sim_xela_server sim_xela_server_node --ros-args \
  -p model_name:=XR23AHLCPP_left \
  -p bind_port:=5000 \
  -p integer_z_range:=4000.0 \
  -p calib_z_range:=6.0 \
  -p z_bias_power:=2.0
```

## Launch
```bash
source /opt/ros/humble/setup.bash
source /home/invokelee/xela_robotics/02_dev_ws/install/setup.bash
ros2 launch sim_xela_server sim_xela_server.launch.py \
  model_name:=XR23AHLCPP_left \
  integer_z_range:=4000.0 calib_z_range:=6.0 z_bias_power:=2.0
```

## Notes
- Reference JSON files live under `resource/` in this package.
- Presets are in `config/replayer_presets.yaml`.
- Multiple clients can connect at the same time and receive the same stream.
- Use `integer_z_range`/`calib_z_range` to reduce simulated Z amplitude.
- `z_bias_power` (>1.0) makes small Z values more frequent.
- Presets `H`, `M`, `L` set `z_bias_power` to 3.0/2.0/1.2.
- FSD: `fsd_sim_xela_server.md`.
