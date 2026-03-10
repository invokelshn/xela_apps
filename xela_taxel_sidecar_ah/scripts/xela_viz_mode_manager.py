#!/usr/bin/env python3

import os
import signal
import subprocess
import threading
from typing import List

import rclpy
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from std_srvs.srv import SetBool


class XelaVizModeManager(Node):
    def __init__(self) -> None:
        super().__init__("xela_viz_mode_manager")

        self.declare_parameter("initial_viz_mode", "grid")
        self.declare_parameter("model_name", "XR23AHLCPP")
        self.declare_parameter("style_preset", "cool_steel")
        self.declare_parameter("frame_id", "x_taxel_viz")
        self.declare_parameter("grid_parent_frame", "world")
        self.declare_parameter("legacy_out_topic", "/x_taxel_ah/markers")
        self.declare_parameter("max_publish_rate_hz", 20.0)
        self.declare_parameter("publisher_transient_local", False)
        self.declare_parameter("marker_stamp_mode", "now")
        self.declare_parameter("circle_marker_type", "sphere")
        self.declare_parameter("managed_launch_package", "ur5e_x2f_140_config2")
        self.declare_parameter("managed_launch_file", "xela_viz_safe.launch.py")
        self.declare_parameter("bridge_node_name", "/xela_taxel_web_bridge")

        self.current_mode = self._normalize_mode(
            str(self.get_parameter("initial_viz_mode").value)
        )
        self.model_name = str(self.get_parameter("model_name").value)
        self.style_preset = str(self.get_parameter("style_preset").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.grid_parent_frame = str(self.get_parameter("grid_parent_frame").value)
        self.legacy_out_topic = str(self.get_parameter("legacy_out_topic").value)
        self.max_publish_rate_hz = float(self.get_parameter("max_publish_rate_hz").value)
        self.publisher_transient_local = bool(
            self.get_parameter("publisher_transient_local").value
        )
        self.marker_stamp_mode = str(self.get_parameter("marker_stamp_mode").value)
        self.circle_marker_type = str(self.get_parameter("circle_marker_type").value)
        self.managed_launch_package = str(
            self.get_parameter("managed_launch_package").value
        )
        self.managed_launch_file = str(self.get_parameter("managed_launch_file").value)
        self.bridge_node_name = str(self.get_parameter("bridge_node_name").value)

        self._lock = threading.Lock()
        self._managed_proc: subprocess.Popen | None = None

        self._set_mode_service = self.create_service(
            SetBool, "xela_viz_mode_manager/set_mode", self._on_set_mode
        )
        self._startup_sync_done = False
        self._startup_sync_timer = self.create_timer(1.0, self._sync_initial_bridge_mode)
        self.get_logger().info(
            "xela_viz_mode_manager started. service=/xela_viz_mode_manager/set_mode"
        )

    @staticmethod
    def _normalize_mode(value: str) -> str:
        mode = value.strip().lower()
        if mode == "urdf":
            return "urdf"
        return "grid"

    def _viz_launch_cmd(self, mode: str) -> List[str]:
        return [
            "ros2",
            "launch",
            self.managed_launch_package,
            self.managed_launch_file,
            f"model_name:={self.model_name}",
            f"viz_mode:={mode}",
            f"style_preset:={self.style_preset}",
            f"frame_id:={self.frame_id}",
            f"grid_parent_frame:={self.grid_parent_frame}",
            f"legacy_out_topic:={self.legacy_out_topic}",
            f"max_publish_rate_hz:={self.max_publish_rate_hz}",
            f"publisher_transient_local:={'true' if self.publisher_transient_local else 'false'}",
            f"marker_stamp_mode:={self.marker_stamp_mode}",
            f"circle_marker_type:={self.circle_marker_type}",
        ]

    def _stop_current_viz(self) -> None:
        subprocess.run(["pkill", "-f", "std_xela_taxel_viz_ahv4_node"], check=False)
        subprocess.run(["pkill", "-f", "xela_grid_static_tf"], check=False)

        if self._managed_proc is not None and self._managed_proc.poll() is None:
            try:
                os.killpg(os.getpgid(self._managed_proc.pid), signal.SIGTERM)
                self._managed_proc.wait(timeout=2.0)
            except Exception:
                try:
                    os.killpg(os.getpgid(self._managed_proc.pid), signal.SIGKILL)
                except Exception:
                    pass
            finally:
                self._managed_proc = None

    def _start_viz(self, mode: str) -> None:
        cmd = self._viz_launch_cmd(mode)
        self._managed_proc = subprocess.Popen(
            cmd,
            preexec_fn=os.setsid,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.STDOUT,
        )

    def _set_bridge_mode(self, mode: str) -> None:
        client = self.create_client(
            SetParameters, f"{self.bridge_node_name}/set_parameters"
        )
        if not client.wait_for_service(timeout_sec=0.8):
            return

        emit_urdf_points = mode == "urdf"
        request = SetParameters.Request()
        request.parameters = [
            Parameter(
                name="viz_mode",
                value=ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=mode),
            ),
            Parameter(
                name="emit_urdf_points",
                value=ParameterValue(
                    type=ParameterType.PARAMETER_BOOL, bool_value=emit_urdf_points
                ),
            ),
            Parameter(
                name="freeze_urdf_positions",
                value=ParameterValue(
                    type=ParameterType.PARAMETER_BOOL, bool_value=emit_urdf_points
                ),
            )
        ]
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=0.8)

    def _sync_initial_bridge_mode(self) -> None:
        if self._startup_sync_done:
            return
        try:
            self._set_bridge_mode(self.current_mode)
            self._startup_sync_done = True
            if self._startup_sync_timer is not None:
                self._startup_sync_timer.cancel()
            self.get_logger().info(f"Initial bridge mode synced: {self.current_mode}")
        except Exception:
            # Bridge can be late in startup. Keep timer until it succeeds.
            pass

    def _on_set_mode(self, request: SetBool.Request, response: SetBool.Response):
        target_mode = "urdf" if request.data else "grid"

        with self._lock:
            if target_mode == self.current_mode:
                self._set_bridge_mode(target_mode)
                response.success = True
                response.message = f"Already in {target_mode} mode"
                return response

            self.get_logger().info(
                f"Switch viz mode: {self.current_mode} -> {target_mode}"
            )
            try:
                self._stop_current_viz()
                self._start_viz(target_mode)
                self._set_bridge_mode(target_mode)
                self.current_mode = target_mode
                response.success = True
                response.message = f"Switched to {target_mode}"
            except Exception as exc:
                response.success = False
                response.message = f"Failed to switch mode: {exc}"
                self.get_logger().error(response.message)
        return response


def main() -> None:
    rclpy.init()
    node = XelaVizModeManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:  # pragma: no cover
        if exc.__class__.__name__ != "ExternalShutdownException":
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass


if __name__ == "__main__":
    main()
