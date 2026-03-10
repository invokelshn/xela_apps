from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _resolve_hand_params(context):
    model_name = LaunchConfiguration("model_name").perform(context).strip().lower()
    hand_side_raw = LaunchConfiguration("hand_side").perform(context).strip().lower()

    is_right_model = model_name in ("xr23ahrcpp", "xr23ahrcpp_right")
    if hand_side_raw in ("left", "l"):
        hand_side = "left"
    elif hand_side_raw in ("right", "r"):
        hand_side = "right"
    else:
        hand_side = "right" if is_right_model else "left"

    pattern_override = LaunchConfiguration("pattern_yaml").perform(context).strip()
    mapping_override = LaunchConfiguration("mapping_yaml").perform(context).strip()

    std_share = FindPackageShare("std_xela_taxel_viz_ahv4").perform(context)
    default_mapping = f"{std_share}/config/maps/taxel_joint_map_new.yaml"
    default_pattern = (
        f"{std_share}/config/patterns/pattern_rahv4.yaml"
        if hand_side == "right"
        else f"{std_share}/config/patterns/pattern_lahv4.yaml"
    )

    return [
        SetLaunchConfiguration("resolved_hand_side", hand_side),
        SetLaunchConfiguration("resolved_mapping_yaml", mapping_override or default_mapping),
        SetLaunchConfiguration("resolved_pattern_yaml", pattern_override or default_pattern),
    ]


def generate_launch_description() -> LaunchDescription:
    pkg = FindPackageShare("xela_taxel_sidecar_ah")
    web_root = PathJoinSubstitution([pkg, "web", "taxel_sidecar"])

    bridge_node = Node(
        package="xela_taxel_sidecar_ah",
        executable="xela_taxel_web_bridge_node",
        name="xela_taxel_web_bridge_cpp",
        output="screen",
        remappings=[
            ("/tf", LaunchConfiguration("bridge_tf_topic")),
            ("/tf_static", LaunchConfiguration("bridge_tf_static_topic")),
        ],
        parameters=[
            {
                "in_topic": LaunchConfiguration("in_topic"),
                "out_topic": LaunchConfiguration("out_topic"),
                "viz_mode": LaunchConfiguration("viz_mode"),
                "model_name": LaunchConfiguration("model_name"),
                "fixed_frame": LaunchConfiguration("fixed_frame"),
                "mapping_yaml": LaunchConfiguration("resolved_mapping_yaml"),
                "pattern_yaml": LaunchConfiguration("resolved_pattern_yaml"),
                "hand_side": LaunchConfiguration("resolved_hand_side"),
                "cell_size": LaunchConfiguration("cell_size"),
                "origin_x": LaunchConfiguration("origin_x"),
                "origin_y": LaunchConfiguration("origin_y"),
                "grid_force_x_sign": LaunchConfiguration("grid_force_x_sign"),
                "grid_force_y_sign": LaunchConfiguration("grid_force_y_sign"),
                "urdf_force_x_sign": LaunchConfiguration("urdf_force_x_sign"),
                "urdf_force_y_sign": LaunchConfiguration("urdf_force_y_sign"),
                "force_scale": LaunchConfiguration("force_scale"),
                "xy_force_range": LaunchConfiguration("xy_force_range"),
                "z_force_range": LaunchConfiguration("z_force_range"),
                "baseline_deadband_xy": LaunchConfiguration("baseline_deadband_xy"),
                "baseline_deadband_z": LaunchConfiguration("baseline_deadband_z"),
                "max_publish_rate_hz": LaunchConfiguration("max_publish_rate_hz"),
                "emit_urdf_points": LaunchConfiguration("emit_urdf_points"),
                "freeze_urdf_positions": LaunchConfiguration("freeze_urdf_positions"),
            }
        ],
    )

    mode_manager_node = Node(
        package="xela_taxel_sidecar_ah",
        executable="xela_viz_mode_manager_node",
        name="xela_viz_mode_manager_cpp",
        output="screen",
        parameters=[
            {
                "initial_viz_mode": LaunchConfiguration("viz_mode"),
                "model_name": LaunchConfiguration("model_name"),
                "style_preset": LaunchConfiguration("style_preset"),
                "frame_id": LaunchConfiguration("frame_id"),
                "grid_parent_frame": LaunchConfiguration("grid_parent_frame"),
                "legacy_out_topic": LaunchConfiguration("legacy_out_topic"),
                "max_publish_rate_hz": LaunchConfiguration("max_publish_rate_hz"),
                "publisher_transient_local": LaunchConfiguration("publisher_transient_local"),
                "marker_stamp_mode": LaunchConfiguration("marker_stamp_mode"),
                "circle_marker_type": LaunchConfiguration("circle_marker_type"),
                "managed_launch_package": LaunchConfiguration("managed_launch_package"),
                "managed_launch_file": LaunchConfiguration("managed_launch_file"),
                "bridge_node_name": LaunchConfiguration("bridge_node_name"),
                "viz_node_name": LaunchConfiguration("viz_node_name"),
                "use_viz_set_mode": LaunchConfiguration("use_viz_set_mode"),
                "require_viz_set_mode_service": LaunchConfiguration("require_viz_set_mode_service"),
            }
        ],
        condition=IfCondition(LaunchConfiguration("enable_viz_mode_manager")),
    )

    web_server = ExecuteProcess(
        cmd=[
            FindExecutable(name="python3"),
            PathJoinSubstitution([pkg, "scripts", "sidecar_http_server.py"]),
            "--host",
            LaunchConfiguration("web_host"),
            "--port",
            LaunchConfiguration("web_port"),
            "--web-root",
            web_root
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_web_server")),
    )

    sidecar_rosbridge = Node(
        package="rosbridge_server",
        executable="rosbridge_websocket",
        name="rosbridge_websocket_sidecar",
        parameters=[
            {
                "port": LaunchConfiguration("sidecar_rosbridge_port"),
                "address": LaunchConfiguration("sidecar_rosbridge_host"),
                "max_message_size": 1000000000000,
                "call_services_in_new_thread": True,
                "send_action_goals_in_new_thread": True,
            }
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_sidecar_rosbridge")),
    )

    return LaunchDescription([
        DeclareLaunchArgument("in_topic", default_value="/x_taxel_ah"),
        DeclareLaunchArgument("out_topic", default_value="/x_taxel_ah/web_state"),
        DeclareLaunchArgument("viz_mode", default_value="grid"),
        DeclareLaunchArgument("model_name", default_value="XR23AHLCPP"),
        DeclareLaunchArgument("hand_side", default_value="auto"),
        DeclareLaunchArgument("mapping_yaml", default_value=""),
        DeclareLaunchArgument("pattern_yaml", default_value=""),
        DeclareLaunchArgument("style_preset", default_value="cool_steel"),
        DeclareLaunchArgument("frame_id", default_value="world"),
        DeclareLaunchArgument("grid_parent_frame", default_value="world"),
        DeclareLaunchArgument("legacy_out_topic", default_value="/x_taxel_ah/markers"),
        DeclareLaunchArgument("fixed_frame", default_value="world"),
        DeclareLaunchArgument("cell_size", default_value="0.01"),
        DeclareLaunchArgument("origin_x", default_value="0.0"),
        DeclareLaunchArgument("origin_y", default_value="0.0"),
        DeclareLaunchArgument("grid_force_x_sign", default_value="1.0"),
        DeclareLaunchArgument("grid_force_y_sign", default_value="1.0"),
        DeclareLaunchArgument("urdf_force_x_sign", default_value="1.0"),
        DeclareLaunchArgument("urdf_force_y_sign", default_value="1.0"),
        DeclareLaunchArgument("force_scale", default_value="1.0"),
        DeclareLaunchArgument("xy_force_range", default_value="1350.0"),
        DeclareLaunchArgument("z_force_range", default_value="16000.0"),
        DeclareLaunchArgument("baseline_deadband_xy", default_value="10.0"),
        DeclareLaunchArgument("baseline_deadband_z", default_value="60.0"),
        DeclareLaunchArgument("max_publish_rate_hz", default_value="20.0"),
        DeclareLaunchArgument("publisher_transient_local", default_value="false"),
        DeclareLaunchArgument("marker_stamp_mode", default_value="keep"),
        DeclareLaunchArgument("circle_marker_type", default_value="sphere"),
        DeclareLaunchArgument("emit_urdf_points", default_value="false"),
        DeclareLaunchArgument("freeze_urdf_positions", default_value="true"),
        DeclareLaunchArgument("enable_viz_mode_manager", default_value="true"),
        DeclareLaunchArgument("managed_launch_package", default_value="xela_taxel_sidecar_ah"),
        DeclareLaunchArgument("managed_launch_file", default_value=""),
        DeclareLaunchArgument("bridge_node_name", default_value="/xela_taxel_web_bridge_cpp"),
        DeclareLaunchArgument("viz_node_name", default_value="/xvizah/std_xela_taxel_viz_ahv4"),
        DeclareLaunchArgument("use_viz_set_mode", default_value="false"),
        DeclareLaunchArgument("require_viz_set_mode_service", default_value="false"),
        DeclareLaunchArgument("bridge_tf_topic", default_value="/tf"),
        DeclareLaunchArgument("bridge_tf_static_topic", default_value="/tf_static"),
        DeclareLaunchArgument("enable_web_server", default_value="true"),
        DeclareLaunchArgument("web_host", default_value="0.0.0.0"),
        DeclareLaunchArgument("web_port", default_value="8765"),
        DeclareLaunchArgument("enable_sidecar_rosbridge", default_value="true"),
        DeclareLaunchArgument("sidecar_rosbridge_host", default_value="0.0.0.0"),
        DeclareLaunchArgument("sidecar_rosbridge_port", default_value="9090"),
        OpaqueFunction(function=_resolve_hand_params),
        sidecar_rosbridge,
        bridge_node,
        mode_manager_node,
        web_server,
    ])
