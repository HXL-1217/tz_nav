"""Main ROS 2 launch entry point for simulation and real-robot remapping."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def _as_bool(value):
    return value.lower() in ("1", "true", "yes", "on")


def _setup(context):
    scan_share = get_package_share_directory("scan_planner")
    planner_yaml = os.path.join(scan_share, "config", "planner.yaml")
    controllers_yaml = os.path.join(scan_share, "config", "controllers.yaml")
    is_real = _as_bool(LaunchConfiguration("is_real_world").perform(context))
    use_sim_time = _as_bool(LaunchConfiguration("use_sim_time").perform(context))
    sensor_type = LaunchConfiguration("sensor_type").perform(context)
    controller_mode = LaunchConfiguration("controller_mode").perform(context)
    keypoints_file = LaunchConfiguration("keypoints_file").perform(context)
    navi_mode = int(LaunchConfiguration("navi_mode").perform(context))
    lidar_height = LaunchConfiguration("lidar_height").perform(context)
    if sensor_type not in ("lidar", "depth"):
        raise RuntimeError("sensor_type must be 'lidar' or 'depth'")
    if controller_mode not in ("open_loop", "closed_loop"):
        raise RuntimeError("controller_mode must be 'open_loop' or 'closed_loop'")
    if navi_mode not in (1, 2, 3):
        raise RuntimeError("navi_mode must be 1, 2, or 3")
    if navi_mode == 2 and (not keypoints_file or not os.path.isfile(keypoints_file)):
        raise RuntimeError(
            "navi_mode=2 requires keypoints_file to reference a ROS 2 parameter YAML"
        )

    if is_real:
        body_pose = "/baselink_odom"
        sensor_pose = "/fastlio/odom"
        cloud = "/fastlio/cloud_output_world"
        depth = "/camera/aligned_depth_to_color/image_raw"
        cloud_is_world = True
        need_extrinsic = False
        intrinsics = {
            "grid_map.cx": 317.19183349609375,
            "grid_map.cy": 256.4806823730469,
            "grid_map.fx": 609.5884399414062,
            "grid_map.fy": 609.22021484375,
        }
    else:
        body_pose = "/quad_0/body_pose"
        sensor_pose = "/quad_0/camera_pose" if sensor_type == "depth" else "/quad_0/lidar_pose"
        cloud = "/quad_0/cloud"
        depth = "/quad_0/depth"
        cloud_is_world = True
        need_extrinsic = False
        intrinsics = {}

    common = {"use_sim_time": use_sim_time}
    # In real mode the planner's poses come from /baselink_odom expressed in
    # camera_init, so the map/markers/sliding_map must be labelled camera_init
    # (the static world->camera_init TF then lets RViz place them with world
    # z=0 at the ground). In simulation everything stays in "world".
    frame_id = "camera_init" if is_real else "world"
    planner_overrides = {
        **common,
        **intrinsics,
        "fsm.navi_mode": navi_mode,
        "grid_map.sensor_type": sensor_type,
        "grid_map.cloud_is_world": cloud_is_world,
        "grid_map.need_extrinsic": need_extrinsic,
        "grid_map.frame_id": frame_id,
    }
    actions = []
    if is_real:
        # Convert fast_lio's body odometry to base_link odometry for the planner.
        actions.append(
            Node(
                package="scan_planner",
                executable="baselink_odom_transform",
                name="baselink_odom_transform",
                output="screen",
                parameters=[
                    common,
                    {
                        "input_odom_topic": "/fastlio/odom",
                        "output_odom_topic": "/baselink_odom",
                        "parent_frame": "camera_init",
                        "source_child_frame": "body",
                        "child_frame": "base_link",
                        "transform_timeout": 0.05,
                    },
                ],
            )
        )
        # Bridge the planner's "world" frame to fast_lio's "camera_init" frame.
        # lidar_height lifts camera_init (the LiDAR origin) above world so that
        # world z=0 sits on the ground; set it to the LiDAR's height above the
        # ground. 0.0 keeps camera_init == world (no vertical shift).
        # Skipped in navi_mode 3, where the global planner works in "map" and
        # the map -> camera_init edge below is used instead, so that camera_init
        # has a single parent.
        if navi_mode != 3:
            actions.append(
                Node(
                    package="tf2_ros",
                    executable="static_transform_publisher",
                    name="static_tf_world_camera_init",
                    arguments=[
                        "--frame-id", "world",
                        "--child-frame-id", "camera_init",
                        "--z", lidar_height,
                    ],
                )
            )
    if is_real and navi_mode == 3:
        # In navi_mode 3 the global planner (OctoPlanner3D) works in the "map"
        # frame, which coincides with camera_init. Publish an identity map ->
        # camera_init TF so map becomes camera_init's only parent (the world ->
        # camera_init edge above is skipped in mode 3 to avoid a double parent).
        # Real mode only: camera_init is the localization frame here; in
        # simulation the planner uses "world" instead.
        actions.append(
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="static_tf_map_camera_init",
                arguments=[
                    "--frame-id", "map",
                    "--child-frame-id", "camera_init",
                ],
            )
        )
    actions.append(
        Node(
            package="scan_planner",
            executable="scan_planner_node",
            name="scan_planner_node",
            output="screen",
            parameters=[planner_yaml] + ([keypoints_file] if keypoints_file else []) + [planner_overrides],
            remappings=[
                ("body_pose", body_pose),
                ("sensor_pose", sensor_pose),
                ("cloud", cloud),
                ("depth", depth),
                ("move_base_simple/goal", "/move_base_simple/goal"),
                ("initial_path", "/smoothed_path"),
            ],
        )
    )
    if not is_real:
        go2_share = get_package_share_directory("go2_description")
        actions.append(
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="go2_robot_state_publisher",
                output="screen",
                parameters=[
                    common,
                    {
                        "robot_description": Command(
                            ["xacro ", os.path.join(go2_share, "xacro", "robot.xacro"),
                             " use_gazebo:=false"]
                        )
                    },
                ],
            )
        )

    if controller_mode == "open_loop":
        actions.append(
            Node(
                package="scan_planner",
                executable="open_loop_controller",
                name="open_loop_controller",
                output="screen",
                parameters=[controllers_yaml, common],
                remappings=[
                    ("planning/bspline", "/planning/bspline"),
                    ("body_pose", body_pose),
                ],
            )
        )
    else:
        actions.append(
            Node(
                package="scan_planner",
                executable="closed_loop_controller",
                name="closed_loop_controller",
                output="screen",
                parameters=[controllers_yaml, common],
                remappings=[
                    ("body_pose", body_pose),
                    ("cmd_vel", "/cmd_vel" if is_real else "/quad_0/cmd_vel"),
                ],
            )
        )
        if not is_real:
            actions.append(
                Node(
                    package="scan_planner",
                    executable="go2_kinematic_sim",
                    name="go2_kinematic_sim",
                    output="screen",
                    parameters=[
                        controllers_yaml,
                        common,
                        {
                            "init_x": float(LaunchConfiguration("init_x").perform(context)),
                            "init_y": float(LaunchConfiguration("init_y").perform(context)),
                            "init_z": float(LaunchConfiguration("init_z").perform(context)),
                            "publish_tf": False,
                        },
                    ],
                    remappings=[
                        ("body_pose", "/quad_0/body_pose"),
                        ("cmd_vel", "/quad_0/cmd_vel"),
                    ],
                )
            )

    if not is_real:
        actions.extend(
            [
                Node(
                    package="scan_planner",
                    executable="go2_gait_publisher",
                    name="go2_gait_publisher",
                    output="screen",
                    parameters=[controllers_yaml, common],
                    remappings=[("body_pose", body_pose)],
                ),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(scan_share, "launch", "simulator.launch.py")
                    ),
                    launch_arguments={
                        name: LaunchConfiguration(name)
                        for name in (
                            "is_real_world",
                            "sensor_type",
                            "use_gpu",
                            "use_pcd_map",
                            "pcd_map_file",
                            "map_size_x",
                            "map_size_y",
                            "map_size_z",
                            "use_sim_time",
                        )
                    }.items(),
                ),
            ]
        )
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("is_real_world", default_value="false"),
            DeclareLaunchArgument("navi_mode", default_value="1"),
            DeclareLaunchArgument("sensor_type", default_value="lidar"),
            DeclareLaunchArgument("controller_mode", default_value="closed_loop"),
            DeclareLaunchArgument("keypoints_file", default_value=""),
            DeclareLaunchArgument("use_gpu", default_value="false"),
            DeclareLaunchArgument("use_pcd_map", default_value="false"),
            DeclareLaunchArgument("pcd_map_file", default_value=""),
            DeclareLaunchArgument("map_size_x", default_value="40.0"),
            DeclareLaunchArgument("map_size_y", default_value="40.0"),
            DeclareLaunchArgument("map_size_z", default_value="5.0"),
            DeclareLaunchArgument("init_x", default_value="-19.0"),
            DeclareLaunchArgument("init_y", default_value="1.0"),
            DeclareLaunchArgument("init_z", default_value="0.3"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument(
                "lidar_height",
                default_value="0.4",
                description="LiDAR height above the ground (m). Real mode only: lifts "
                "camera_init above world so world z=0 sits on the ground. "
                "0.0 keeps world == camera_init.",
            ),
            OpaqueFunction(function=_setup),
        ]
    )
