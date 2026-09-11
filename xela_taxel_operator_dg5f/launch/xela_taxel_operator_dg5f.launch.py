"""Launch xela_taxel_operator_dg5f: a standalone operator-only web app for DG5F.

Structure mirrors xela_taxel_sidecar_dg5f/launch/xela_taxel_sidecar_cpp.launch.py's
rosbridge + web-server pattern.

2026-09-10 regression fix: this launch file used to start NO bridge/mode-manager/viewer ROS
nodes of its own, on the assumption (see git history) that xela_taxel_web_bridge_node,
xela_viz_mode_manager_node, and xela_atag_taxel_viewer_node already run as part of the
existing baseline stack (ur7e_xdg5f_atag_right_common's xela_driver.launch.py). That
assumption only holds when the DG5F Admin stack is running alongside this Operator page. The
whole point of ur7e_xdg5f_atag_right_sim_dev (Phase 3.5) is to run the Operator page WITHOUT
Admin, which left /x_taxel_dg5f/web_state (and the taxel-viewer's session/alert data) with no
publisher at all -- confirmed via `ros2 topic hz` (0 messages) during real usage testing of
`moveit_pro run -c ur7e_xdg5f_atag_right_sim_dev`.

Fix: this package now launches xela_taxel_web_bridge_node and xela_atag_taxel_viewer_node
itself, reusing the already-built executables from xela_taxel_sidecar_dg5f and
xela_atag_taxel_viewer via exec_depend (no source copy/reimplementation -- those packages
remain unmodified, read-only dependencies). Parameters mirror the values
ur7e_xdg5f_atag_right_common/launch/xela_driver.launch.py passes into
xela_taxel_sidecar_dg5f/launch/xela_taxel_sidecar_cpp.launch.py for the DG5F sim case, so
Operator's own bridge instance behaves identically to the one Admin would have started.

xela_viz_mode_manager_node is intentionally NOT started here: it exists to let the sidecar's
own web UI toggle between "grid" and "urdf" visualization modes at runtime (service
/xela_viz_mode_manager_cpp/set_mode) and to manage restarting the owning launch file. The
Operator page has no grid/urdf mode toggle -- it always renders in a fixed "urdf" mode -- so
there is nothing in this package that would ever call that service. Running it would be inert
extra process weight, not a functional gap. If Operator ever grows a mode-switching UI, revisit
this.

std_xela_taxel_viz_dg5f (TF/URDF publisher) is launched separately, by
ur7e_xdg5f_atag_right_sim_dev/launch/xela_driver_dev.launch.py -- it is shared infrastructure
both Admin and Operator pages need and is unaffected by this fix.

Ports default to 8766 (web) / 9092 (rosbridge), deliberately distinct from
xela_taxel_sidecar_dg5f's 8765/9090/9091 so both packages can run at the same time without
colliding. All ports are launch args so they can be overridden if needed.

2026-09-10 addendum: the intended deployment is Admin (xela_taxel_sidecar_dg5f) and Operator
(this package) running SIMULTANEOUSLY from one robot config package
(ur7e_xdg5f_atag_right_sim_dev), not Operator standalone. When both run together, Admin already
starts xela_taxel_web_bridge_node (name xela_taxel_web_bridge_cpp) and
xela_atag_taxel_viewer_node itself, so this package must NOT start its own copies -- doing so
would start two nodes with the same name. The new `enable_data_bridge_nodes` launch arg (default
"true") controls this: leave it "true" for standalone Operator-only launches (no Admin running),
set it to "false" when Operator is included alongside Admin so only one instance of each node
exists.
"""
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetLaunchConfiguration,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    FindExecutable,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _resolve_hand_params(context):
    # Mirrors xela_taxel_sidecar_dg5f/launch/xela_taxel_sidecar_cpp.launch.py's
    # _resolve_hand_params: DG-5F is right-hand only today, so hand_side always resolves to
    # "right", but mapping_yaml/pattern_yaml launch args are still honored if explicitly
    # overridden.
    hand_side = "right"

    pattern_override = LaunchConfiguration("pattern_yaml").perform(context).strip()
    mapping_override = LaunchConfiguration("mapping_yaml").perform(context).strip()

    server_share = FindPackageShare("xela_server2_dg5f").perform(context)
    std_share = FindPackageShare("std_xela_taxel_viz_dg5f").perform(context)
    default_mapping = f"{server_share}/config/r_server_model_joint_map.yaml"
    default_pattern = f"{std_share}/config/patterns/pattern_rdg5f.yaml"

    return [
        SetLaunchConfiguration("resolved_hand_side", hand_side),
        SetLaunchConfiguration("resolved_mapping_yaml", mapping_override or default_mapping),
        SetLaunchConfiguration("resolved_pattern_yaml", pattern_override or default_pattern),
    ]


def generate_launch_description() -> LaunchDescription:
    pkg = FindPackageShare("xela_taxel_operator_dg5f")
    web_root = PathJoinSubstitution([pkg, "web"])

    operator_rosbridge = Node(
        package="rosbridge_server",
        executable="rosbridge_websocket",
        name="rosbridge_websocket_operator_dg5f",
        parameters=[
            {
                "port": LaunchConfiguration("rosbridge_port"),
                "address": LaunchConfiguration("rosbridge_host"),
                "max_message_size": 1000000000000,
                "call_services_in_new_thread": True,
                "send_action_goals_in_new_thread": True,
            }
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_rosbridge")),
    )

    web_server = ExecuteProcess(
        cmd=[
            FindExecutable(name="python3"),
            PathJoinSubstitution([pkg, "scripts", "operator_http_server.py"]),
            "--host",
            LaunchConfiguration("web_host"),
            "--port",
            LaunchConfiguration("web_port"),
            "--web-root",
            web_root,
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_web_server")),
    )

    # Reuses xela_taxel_sidecar_dg5f's already-built xela_taxel_web_bridge_node executable
    # (exec_depend only -- xela_taxel_sidecar_dg5f itself is not modified). Parameter values
    # mirror what ur7e_xdg5f_atag_right_common/launch/xela_driver.launch.py passes for the
    # DG5F sim case, so this standalone instance behaves the same as the baseline one.
    bridge_node = Node(
        package="xela_taxel_sidecar_dg5f",
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
        condition=IfCondition(
            PythonExpression([
                "'", LaunchConfiguration("enable_taxel_bridge"), "' == 'true' and '",
                LaunchConfiguration("enable_data_bridge_nodes"), "' == 'true'",
            ])
        ),
    )

    # Reuses xela_atag_taxel_viewer's already-built node (exec_depend only). Feeds the
    # Operator page's session/filmstrip/alert widgets (/xela_atag_taxel_viewer_node/*).
    # xela_atag_taxel_viewer.launch.py declares exactly one Node and no rosbridge/web server
    # of its own, so it is safe to IncludeLaunchDescription wholesale here without any port
    # collision risk.
    taxel_viewer_launch = PathJoinSubstitution([
        FindPackageShare("xela_atag_taxel_viewer"), "launch", "xela_atag_taxel_viewer.launch.py",
    ])

    return LaunchDescription([
        DeclareLaunchArgument("enable_web_server", default_value="true"),
        DeclareLaunchArgument("web_host", default_value="0.0.0.0"),
        DeclareLaunchArgument("web_port", default_value="8766"),
        DeclareLaunchArgument("enable_rosbridge", default_value="true"),
        DeclareLaunchArgument("rosbridge_host", default_value="0.0.0.0"),
        DeclareLaunchArgument("rosbridge_port", default_value="9092"),

        # xela_taxel_web_bridge_node launch args -- defaults match
        # ur7e_xdg5f_atag_right_common/launch/xela_driver.launch.py's DG5F sim values.
        DeclareLaunchArgument("enable_taxel_bridge", default_value="true"),
        # When Operator is launched alongside Admin (xela_taxel_sidecar_dg5f), Admin already
        # starts xela_taxel_web_bridge_node and xela_atag_taxel_viewer_node -- set this to
        # "false" in that case to avoid starting duplicate nodes with the same name. Defaults to
        # "true" so standalone Operator launches (no Admin) keep working unchanged.
        DeclareLaunchArgument("enable_data_bridge_nodes", default_value="true"),
        DeclareLaunchArgument("in_topic", default_value="/atag/taxel_data"),
        DeclareLaunchArgument("out_topic", default_value="/x_taxel_dg5f/web_state"),
        DeclareLaunchArgument("viz_mode", default_value="urdf"),
        DeclareLaunchArgument("model_name", default_value="XDG5FR"),
        DeclareLaunchArgument("mapping_yaml", default_value=""),
        DeclareLaunchArgument("pattern_yaml", default_value=""),
        DeclareLaunchArgument("fixed_frame", default_value="world"),
        DeclareLaunchArgument("cell_size", default_value="0.01"),
        DeclareLaunchArgument("origin_x", default_value="0.0"),
        DeclareLaunchArgument("origin_y", default_value="0.0"),
        DeclareLaunchArgument("grid_force_x_sign", default_value="1.0"),
        DeclareLaunchArgument("grid_force_y_sign", default_value="1.0"),
        DeclareLaunchArgument("urdf_force_x_sign", default_value="1.0"),
        DeclareLaunchArgument("urdf_force_y_sign", default_value="1.0"),
        DeclareLaunchArgument("force_scale", default_value="1.0"),
        DeclareLaunchArgument("xy_force_range", default_value="0.8"),
        DeclareLaunchArgument("z_force_range", default_value="3.5"),
        DeclareLaunchArgument("baseline_deadband_xy", default_value="0.02"),
        DeclareLaunchArgument("baseline_deadband_z", default_value="0.05"),
        DeclareLaunchArgument("max_publish_rate_hz", default_value="20.0"),
        DeclareLaunchArgument("emit_urdf_points", default_value="false"),
        DeclareLaunchArgument("freeze_urdf_positions", default_value="false"),
        DeclareLaunchArgument("bridge_tf_topic", default_value="/xvizdg5f/tf"),
        DeclareLaunchArgument("bridge_tf_static_topic", default_value="/xvizdg5f/tf_static"),

        # xela_atag_taxel_viewer_node launch args.
        DeclareLaunchArgument("enable_taxel_viewer", default_value="true"),
        DeclareLaunchArgument("taxel_viewer_topic", default_value="/atag/taxel_data"),

        OpaqueFunction(function=_resolve_hand_params),
        operator_rosbridge,
        web_server,
        bridge_node,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(taxel_viewer_launch),
            launch_arguments={
                "taxel_topic": LaunchConfiguration("taxel_viewer_topic"),
                "taxel_model_name": LaunchConfiguration("model_name"),
            }.items(),
            condition=IfCondition(
                PythonExpression([
                    "'", LaunchConfiguration("enable_taxel_viewer"), "' == 'true' and '",
                    LaunchConfiguration("enable_data_bridge_nodes"), "' == 'true'",
                ])
            ),
        ),
    ])
