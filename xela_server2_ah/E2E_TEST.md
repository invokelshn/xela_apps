# End-to-End Test (Replayer -> xela_server2_ah -> xela_taxel_viz_ahv4)

## 1) Build
```bash
source /opt/ros/humble/setup.bash
cd /home/invokelee/xela_robotics/02_dev_ws
colcon build --packages-select sim_xela_server xela_server2_ah xela_taxel_viz_ahv4
```

## 2) Launch (all-in-one)
```bash
source /opt/ros/humble/setup.bash
source /home/invokelee/xela_robotics/02_dev_ws/install/setup.bash
ros2 launch xela_server2_ah xela_server2_ah_with_replayer_and_viz.launch.py \
  model_name:=XR23AHLCPP_left
```

## 3) Verify topics
```bash
source /opt/ros/humble/setup.bash
source /home/invokelee/xela_robotics/02_dev_ws/install/setup.bash
ros2 topic echo /x_taxel_ah --once
ros2 topic echo /x_taxel_ah/markers --once
```

## Notes
- The replayer serves JSON over WebSocket on `ws://<bind_host>:<bind_port>`.
- `xela_server2_ah` subscribes as a WebSocket client and publishes `/x_taxel_ah`.
- `xela_taxel_viz_ahv4` subscribes to `/x_taxel_ah` and publishes `/x_taxel_ah/markers`.
