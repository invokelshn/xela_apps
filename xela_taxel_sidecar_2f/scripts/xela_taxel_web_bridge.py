#!/usr/bin/env python3

import json
import math
import time
from typing import Dict, List, Optional, Tuple

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, ConnectivityException, ExtrapolationException, LookupException, TransformListener
from xela_taxel_msgs.msg import XTaxelSensorTArray


def _to_link_frame(frame_id: str) -> str:
    if frame_id.endswith("_joint"):
        return frame_id[:-6] + "_link"
    return frame_id


def _safe_xyz(value) -> Tuple[float, float, float]:
    return (
        float(getattr(value, "x", 0.0)),
        float(getattr(value, "y", 0.0)),
        float(getattr(value, "z", 0.0)),
    )


def _as_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return value != 0
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


class XelaTaxelWebBridge(Node):
    def __init__(self) -> None:
        super().__init__("xela_taxel_web_bridge")

        self.declare_parameter("in_topic", "/x_taxel_2f")
        self.declare_parameter("out_topic", "/x_taxel_2f/web_state")
        self.declare_parameter("viz_mode", "grid")
        self.declare_parameter("model_name", "uSPr2F")
        self.declare_parameter("fixed_frame", "world")

        self.declare_parameter("left_module_index", 0)
        self.declare_parameter("right_module_index", 1)

        self.declare_parameter("grid_rows", 4)
        self.declare_parameter("grid_cols", 6)
        self.declare_parameter("cell_size", 0.015)
        self.declare_parameter("module_gap", 0.04)
        self.declare_parameter("origin_x", 0.0)
        self.declare_parameter("origin_y", 0.0)
        self.declare_parameter("row_flip_right", True)
        self.declare_parameter("col_flip_right", True)

        self.declare_parameter("force_scale", 1.0)
        self.declare_parameter("use_fz_only", False)
        self.declare_parameter("use_axis_normalization", True)
        self.declare_parameter("xy_force_range", 0.8)
        self.declare_parameter("z_force_range", 14.0)

        self.declare_parameter("grid_left_force_x_sign", 1.0)
        self.declare_parameter("grid_left_force_y_sign", -1.0)
        self.declare_parameter("grid_right_force_x_sign", -1.0)
        self.declare_parameter("grid_right_force_y_sign", -1.0)

        self.declare_parameter("urdf_left_force_x_sign", 1.0)
        self.declare_parameter("urdf_left_force_y_sign", 1.0)
        self.declare_parameter("urdf_right_force_x_sign", 1.0)
        self.declare_parameter("urdf_right_force_y_sign", 1.0)

        self.declare_parameter("max_publish_rate_hz", 20.0)
        self.declare_parameter("emit_urdf_points", False)
        self.declare_parameter("freeze_urdf_positions", True)
        self.declare_parameter("tf_cache_ttl_sec", 0.5)

        self.in_topic = str(self.get_parameter("in_topic").value)
        self.out_topic = str(self.get_parameter("out_topic").value)
        self.viz_mode = str(self.get_parameter("viz_mode").value).lower()
        if self.viz_mode not in ("grid", "urdf"):
            self.viz_mode = "grid"
        self.model_name = str(self.get_parameter("model_name").value)
        self.fixed_frame = str(self.get_parameter("fixed_frame").value)

        self.left_module_index = int(self.get_parameter("left_module_index").value)
        self.right_module_index = int(self.get_parameter("right_module_index").value)

        self.grid_rows = max(1, int(self.get_parameter("grid_rows").value))
        self.grid_cols = max(1, int(self.get_parameter("grid_cols").value))
        self.cell_size = float(self.get_parameter("cell_size").value)
        self.module_gap = float(self.get_parameter("module_gap").value)
        self.origin_x = float(self.get_parameter("origin_x").value)
        self.origin_y = float(self.get_parameter("origin_y").value)
        self.row_flip_right = bool(self.get_parameter("row_flip_right").value)
        self.col_flip_right = bool(self.get_parameter("col_flip_right").value)

        self.force_scale = float(self.get_parameter("force_scale").value)
        self.use_fz_only = bool(self.get_parameter("use_fz_only").value)
        self.use_axis_normalization = bool(self.get_parameter("use_axis_normalization").value)
        self.xy_force_range = max(1e-9, float(self.get_parameter("xy_force_range").value))
        self.z_force_range = max(1e-9, float(self.get_parameter("z_force_range").value))

        self.grid_left_force_x_sign = float(self.get_parameter("grid_left_force_x_sign").value)
        self.grid_left_force_y_sign = float(self.get_parameter("grid_left_force_y_sign").value)
        self.grid_right_force_x_sign = float(self.get_parameter("grid_right_force_x_sign").value)
        self.grid_right_force_y_sign = float(self.get_parameter("grid_right_force_y_sign").value)

        self.urdf_left_force_x_sign = float(self.get_parameter("urdf_left_force_x_sign").value)
        self.urdf_left_force_y_sign = float(self.get_parameter("urdf_left_force_y_sign").value)
        self.urdf_right_force_x_sign = float(self.get_parameter("urdf_right_force_x_sign").value)
        self.urdf_right_force_y_sign = float(self.get_parameter("urdf_right_force_y_sign").value)

        max_publish_rate_hz = float(self.get_parameter("max_publish_rate_hz").value)
        self.publish_period = 0.0 if max_publish_rate_hz <= 0.0 else 1.0 / max_publish_rate_hz
        self.emit_urdf_points = _as_bool(self.get_parameter("emit_urdf_points").value)
        if self.viz_mode == "urdf":
            self.emit_urdf_points = True
        self.freeze_urdf_positions = bool(self.get_parameter("freeze_urdf_positions").value)
        self.tf_cache_ttl_sec = max(0.0, float(self.get_parameter("tf_cache_ttl_sec").value))

        self.grid_size = self.grid_rows * self.grid_cols
        self.layout_left = self._build_layout(
            base_x=self.origin_x,
            base_y=self.origin_y,
            row_flip=False,
            col_flip=False,
        )
        self.layout_right = self._build_layout(
            base_x=self.origin_x + self.grid_cols * self.cell_size + self.module_gap,
            base_y=self.origin_y,
            row_flip=self.row_flip_right,
            col_flip=self.col_flip_right,
        )

        self.tf_cache: Dict[str, Tuple[float, float, float, float]] = {}
        self.tf_buffer = None
        self.tf_listener = None
        self._configure_tf(self.emit_urdf_points)

        self.last_publish_steady = 0.0

        self.pub = self.create_publisher(String, self.out_topic, 10)
        self.sub = self.create_subscription(
            XTaxelSensorTArray,
            self.in_topic,
            self._on_taxel_array,
            10,
        )
        self._params_cb = self.add_on_set_parameters_callback(self._on_set_parameters)

        self.get_logger().info(
            f"xela_taxel_web_bridge started: in={self.in_topic} out={self.out_topic} fixed_frame={self.fixed_frame}"
        )

    def _on_set_parameters(self, params):
        result = SetParametersResult(successful=True)
        for param in params:
            if param.name == "viz_mode":
                value = str(param.value).strip().lower()
                if value not in ("grid", "urdf"):
                    result.successful = False
                    result.reason = "viz_mode must be 'grid' or 'urdf'"
                    return result
                self.viz_mode = value
                if self.viz_mode == "urdf":
                    self.emit_urdf_points = True
            elif param.name == "model_name":
                self.model_name = str(param.value).strip()
            elif param.name == "emit_urdf_points":
                self.emit_urdf_points = _as_bool(param.value)
            elif param.name == "freeze_urdf_positions":
                self.freeze_urdf_positions = _as_bool(param.value)
        self._configure_tf(self.emit_urdf_points)
        return result

    def _configure_tf(self, enable: bool) -> None:
        if enable and self.tf_buffer is None:
            self.tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
            self.tf_listener = TransformListener(self.tf_buffer, self)

    def _build_layout(self, base_x: float, base_y: float, row_flip: bool, col_flip: bool) -> List[Tuple[float, float]]:
        centers: List[Tuple[float, float]] = []
        for r in range(self.grid_rows):
            for c in range(self.grid_cols):
                draw_r = (self.grid_rows - 1 - r) if row_flip else r
                draw_c = (self.grid_cols - 1 - c) if col_flip else c
                x = base_x + (draw_c + 0.5) * self.cell_size
                y = base_y - (draw_r + 0.5) * self.cell_size
                centers.append((x, y))
        return centers

    def _normalize_force(self, fx: float, fy: float, fz: float) -> float:
        if self.use_fz_only:
            return min(abs(fz) / self.z_force_range, 1.0)

        if self.use_axis_normalization:
            z_clamped = max(0.0, fz)
            nx = fx / self.xy_force_range
            ny = fy / self.xy_force_range
            nz = z_clamped / self.z_force_range
            return min(math.sqrt(nx * nx + ny * ny + nz * nz), 1.0)

        mag = math.sqrt(fx * fx + fy * fy + fz * fz)
        denom = math.sqrt(self.xy_force_range * self.xy_force_range * 2.0 + self.z_force_range * self.z_force_range)
        return min(mag / max(denom, 1e-9), 1.0)

    def _lookup_frame_xyz(self, frame_id: str) -> Optional[Tuple[float, float, float]]:
        if not self.emit_urdf_points or self.tf_buffer is None:
            return None

        now_steady = time.monotonic()
        cached = self.tf_cache.get(frame_id)
        if cached is not None:
            x, y, z, t = cached
            if self.freeze_urdf_positions:
                return (x, y, z)
            if self.tf_cache_ttl_sec <= 0.0 or (now_steady - t) < self.tf_cache_ttl_sec:
                return (x, y, z)

        try:
            tf_msg = self.tf_buffer.lookup_transform(
                self.fixed_frame,
                frame_id,
                Time(),
            )
            tx = float(tf_msg.transform.translation.x)
            ty = float(tf_msg.transform.translation.y)
            tz = float(tf_msg.transform.translation.z)
            self.tf_cache[frame_id] = (tx, ty, tz, now_steady)
            return (tx, ty, tz)
        except (LookupException, ConnectivityException, ExtrapolationException):
            if cached is not None:
                return (cached[0], cached[1], cached[2])
            return None

    def _module_payload(
        self,
        module,
        module_name: str,
        layout: List[Tuple[float, float]],
        grid_sign_x: float,
        grid_sign_y: float,
        urdf_sign_x: float,
        urdf_sign_y: float,
    ) -> Tuple[List[dict], List[dict]]:
        grid_points: List[dict] = []
        urdf_points: List[dict] = []

        force_count = len(module.forces)
        taxel_count = len(module.taxels)
        frame_count = len(module.frame_ids)

        use_taxels = force_count == 0 and taxel_count > 0
        data_count = taxel_count if use_taxels else force_count

        for idx, (cx, cy) in enumerate(layout):
            if idx >= data_count:
                continue

            source = module.taxels[idx] if use_taxels else module.forces[idx]
            fx, fy, fz = _safe_xyz(source)
            fx *= self.force_scale
            fy *= self.force_scale
            fz *= self.force_scale

            g_fx = fx * grid_sign_x
            g_fy = fy * grid_sign_y
            g_norm = self._normalize_force(g_fx, g_fy, fz)
            grid_points.append(
                {
                    "module": module_name,
                    "sensor_index": idx,
                    "x": cx,
                    "y": cy,
                    "fx": g_fx,
                    "fy": g_fy,
                    "fz": fz,
                    "norm": g_norm,
                }
            )

            if self.emit_urdf_points and idx < frame_count:
                frame_id = _to_link_frame(module.frame_ids[idx])
                xyz = self._lookup_frame_xyz(frame_id)
                if xyz is not None:
                    u_fx = fx * urdf_sign_x
                    u_fy = fy * urdf_sign_y
                    u_norm = self._normalize_force(u_fx, u_fy, fz)
                    urdf_points.append(
                        {
                            "module": module_name,
                            "sensor_index": idx,
                            "frame_id": frame_id,
                            "x": xyz[0],
                            "y": xyz[1],
                            "z": xyz[2],
                            "fx": u_fx,
                            "fy": u_fy,
                            "fz": fz,
                            "norm": u_norm,
                        }
                    )

        return grid_points, urdf_points

    def _on_taxel_array(self, msg: XTaxelSensorTArray) -> None:
        now_steady = time.monotonic()
        if self.publish_period > 0.0 and (now_steady - self.last_publish_steady) < self.publish_period:
            return
        self.last_publish_steady = now_steady

        modules_count = len(msg.x_modules)
        if modules_count == 0:
            return

        grid_points: List[dict] = []
        urdf_points: List[dict] = []

        left_name = "left"
        right_name = "right"

        if self.left_module_index >= 0 and self.left_module_index < modules_count:
            left_grid, left_urdf = self._module_payload(
                module=msg.x_modules[self.left_module_index],
                module_name=left_name,
                layout=self.layout_left,
                grid_sign_x=self.grid_left_force_x_sign,
                grid_sign_y=self.grid_left_force_y_sign,
                urdf_sign_x=self.urdf_left_force_x_sign,
                urdf_sign_y=self.urdf_left_force_y_sign,
            )
            grid_points.extend(left_grid)
            urdf_points.extend(left_urdf)

        if self.right_module_index >= 0 and self.right_module_index < modules_count:
            right_grid, right_urdf = self._module_payload(
                module=msg.x_modules[self.right_module_index],
                module_name=right_name,
                layout=self.layout_right,
                grid_sign_x=self.grid_right_force_x_sign,
                grid_sign_y=self.grid_right_force_y_sign,
                urdf_sign_x=self.urdf_right_force_x_sign,
                urdf_sign_y=self.urdf_right_force_y_sign,
            )
            grid_points.extend(right_grid)
            urdf_points.extend(right_urdf)

        stamp = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        payload = {
            "stamp": stamp,
            "fixed_frame": self.fixed_frame,
            "grid": {
                "rows": self.grid_rows,
                "cols": self.grid_cols,
                "cell_size": self.cell_size,
                "module_gap": self.module_gap,
                "points": grid_points,
            },
            "urdf": {
                "points": urdf_points,
            },
            "meta": {
                "source_topic": self.in_topic,
                "module_count": modules_count,
                "requested_viz_mode": self.viz_mode,
                "sidecar_render_mode": (
                    "urdf-fixed-3d" if (self.viz_mode == "urdf" and self.emit_urdf_points) else "grid-2d"
                ),
                "sidecar_grid_only": not (self.viz_mode == "urdf" and self.emit_urdf_points),
                "model_name": self.model_name,
                "emit_urdf_points": self.emit_urdf_points,
                "freeze_urdf_positions": self.freeze_urdf_positions if self.emit_urdf_points else False,
                "urdf_points_count": len(urdf_points),
                "xy_force_range": self.xy_force_range,
                "z_force_range": self.z_force_range,
                "use_fz_only": self.use_fz_only,
                "force_scale": self.force_scale,
            },
        }

        out = String()
        out.data = json.dumps(payload, separators=(",", ":"))
        self.pub.publish(out)


def main() -> None:
    rclpy.init()
    node = XelaTaxelWebBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:  # pragma: no cover - shutdown race protection
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
