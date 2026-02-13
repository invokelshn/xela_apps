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
from launch.substitutions import PythonExpression
from launch.substitutions import TextSubstitution
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
            package="xela_server2_2f",
            executable="xela_server2_2f_node",
            name="xela_server2_2f",
            parameters=[
                f"{get_package_share_directory('xela_server2_2f')}/config/xela_server2_2f_params.yaml",
                {
                    "ws_host": LaunchConfiguration("ws_host"),
                    "ws_port": LaunchConfiguration("ws_port"),
                    "frame_ids_yaml": LaunchConfiguration("frame_ids_yaml"),
                }
            ],
            output="screen",
        )
    ]


def _on_can_init_exit(event, context):
    if event.returncode != 0:
        return [LogInfo(msg="CAN init failed; skipping xela_server2_2f launch")]

    return [
        ExecuteProcess(
            cmd=[LaunchConfiguration("xela_server_exec")],
            output="screen",
        ),
        LogInfo(msg="Waiting for WebSocket server..."),
        OpaqueFunction(function=_wait_for_websocket),
    ]


def generate_launch_description():
    xela_share = get_package_share_directory("xela_server2_2f")
    default_config = f"{xela_share}/config/server2_2f_config.yaml"

    can_port = LaunchConfiguration("can_port")
    can_command = PythonExpression(
        [
            TextSubstitution(text="'sudo ip link set ' + '"),
            can_port,
            TextSubstitution(text="' + ' down && sudo ip link set ' + '"),
            can_port,
            TextSubstitution(text="' + ' type can bitrate 1000000 && sudo ip link set ' + '"),
            can_port,
            TextSubstitution(text="' + ' up'"),
        ]
    )

    init_can = ExecuteProcess(
        cmd=[
            "bash",
            "-lc",
            can_command,
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("can_port", default_value="can0"),
            DeclareLaunchArgument("ws_host", default_value="localhost"),
            DeclareLaunchArgument("ws_port", default_value="5000"),
            DeclareLaunchArgument("frame_ids_yaml", default_value=default_config),
            DeclareLaunchArgument("xela_server_exec", default_value="/etc/xela/xela_server"),
            init_can,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=init_can,
                    on_exit=_on_can_init_exit,
                )
            ),
        ]
    )
