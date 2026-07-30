import socket

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import LogInfo
from launch.actions import OpaqueFunction
from launch.actions import RegisterEventHandler
from launch.actions import TimerAction
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _strip_ws_scheme(host: str) -> str:
    if host.startswith("ws://"):
        return host[len("ws://"):]
    if host.startswith("wss://"):
        return host[len("wss://"):]
    return host


def _wait_for_websocket(context, *args, **kwargs):
    ws_host = LaunchConfiguration("ws_host").perform(context)
    ws_port = int(LaunchConfiguration("ws_port").perform(context))
    ws_host = _strip_ws_scheme(ws_host)

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(0.5)
    try:
        sock.connect((ws_host, ws_port))
    except OSError:
        return [
            TimerAction(
                period=1.0,
                actions=[OpaqueFunction(function=_wait_for_websocket)],
            )
        ]
    finally:
        sock.close()

    return [
        Node(
            package="xela_server2_ah",
            executable="xela_server2_ah_node",
            name="xela_server2_ah",
            parameters=[{
                "ws_host": LaunchConfiguration("ws_host"),
                "ws_port": LaunchConfiguration("ws_port"),
                "hand_side": LaunchConfiguration("hand_side"),
                "header_frame_id": LaunchConfiguration("header_frame_id"),
                "use_ros_time_for_sensor_time": LaunchConfiguration("use_ros_time_for_sensor_time"),
                "publisher_qos_depth": LaunchConfiguration("publisher_qos_depth"),
                "input_json_path": LaunchConfiguration("input_json_path"),
                "playback_interval_ms": LaunchConfiguration("playback_interval_ms"),
                "playback_loop": LaunchConfiguration("playback_loop"),
            }],
            prefix="chrt --rr 40",
            output="screen",
        )
    ]


def _on_can_init_exit(event, context):
    if event.returncode != 0:
        return [LogInfo(msg="CAN init failed; skipping xela_server2_ah launch")]

    return [
        ExecuteProcess(
            cmd=[LaunchConfiguration("xela_server_exec")],
            output="screen",
        ),
        LogInfo(msg="Waiting for WebSocket server..."),
        OpaqueFunction(function=_wait_for_websocket),
    ]


def _launch_can_init(context, *args, **kwargs):
    port0 = LaunchConfiguration("can_port").perform(context)
    port1 = LaunchConfiguration("can_port_1").perform(context)

    def _can_up_cmd(port):
        return (
            f"sudo ip link set {port} down"
            f" && sudo ip link set {port} type can bitrate 1000000"
            f" && sudo ip link set {port} up"
        )

    cmd = _can_up_cmd(port0)
    if port1:
        cmd += f" && {_can_up_cmd(port1)}"

    init_can = ExecuteProcess(cmd=["bash", "-lc", cmd], output="screen")

    return [
        init_can,
        RegisterEventHandler(
            OnProcessExit(
                target_action=init_can,
                on_exit=_on_can_init_exit,
            )
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("can_port", default_value="can1",
                                  description="Primary CAN interface for Xela taxel sensors (e.g. mcan0)"),
            DeclareLaunchArgument("can_port_1", default_value="",
                                  description="Secondary CAN interface for Xela taxel sensors (e.g. mcan1); leave empty if unused"),
            DeclareLaunchArgument("ws_host", default_value="localhost",
                                  description="WebSocket host where xela_server publishes data"),
            DeclareLaunchArgument("ws_port", default_value="5000",
                                  description="WebSocket port"),
            DeclareLaunchArgument("hand_side", default_value="left",
                                  description="Hand side: left or right"),
            DeclareLaunchArgument("header_frame_id", default_value=""),
            DeclareLaunchArgument("use_ros_time_for_sensor_time", default_value="false"),
            DeclareLaunchArgument("publisher_qos_depth", default_value="10"),
            DeclareLaunchArgument("input_json_path", default_value=""),
            DeclareLaunchArgument("playback_interval_ms", default_value="100"),
            DeclareLaunchArgument("playback_loop", default_value="true"),
            DeclareLaunchArgument("xela_server_exec", default_value="/etc/xela/xela_server",
                                  description="Path to xela_server binary that talks to taxel CAN"),
            OpaqueFunction(function=_launch_can_init),
        ]
    )
