"""One-shot launch: local planner (run.launch.py) + RViz2 (rviz.launch.py),
optionally together with the OctoPlanner3D global planner visualization node and
its own RViz2 window.

Edit the values in PLANNER_ARGS and the OCTO_* path variables below to change how
the planners start, so you don't have to pass them on the command line every time.
Command-line overrides of the same names still work, e.g.:

    ros2 launch scan_planner bringup.launch.py navi_mode:=2

三个独立开关（默认均为 true），可按需单独关闭：

    ros2 launch scan_planner bringup.launch.py use_global_planner:=false  # 关全局规划节点
    ros2 launch scan_planner bringup.launch.py use_local_rviz:=false      # 关局部规划 RViz 窗口
    ros2 launch scan_planner bringup.launch.py use_global_rviz:=false     # 关全局规划 RViz 窗口
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    scan_share = get_package_share_directory("scan_planner")
    launch_dir = os.path.join(scan_share, "launch")

    # ------------------------------------------------------------------
    # 局部规划器 (run.launch.py) 默认启动参数：按需修改这里的取值，
    # 无需在命令行再传参。命令行同名参数仍可覆盖这些默认值。
    # ------------------------------------------------------------------
    PLANNER_ARGS = {
        "is_real_world": "true",      # true: 实车 / false: 仿真
        "navi_mode": "3",              # 1: RViz 目标点  2: 预设航点  3: 全局路径
        "sensor_type": "lidar",        # lidar 或 depth
        "controller_mode": "closed_loop",  # open_loop 或 closed_loop
        "keypoints_file": "",          # navi_mode=2 时需指向 ROS 2 参数 YAML
        "use_gpu": "false",
        "use_pcd_map": "false",
        "pcd_map_file": "",            # use_pcd_map=true 时需指向绝对路径的 .pcd
        "init_x": "-19.0",             # 仅仿真 (closed_loop) 生效
        "init_y": "1.0",
        "init_z": "0.3",
        "use_sim_time": "false",
        "lidar_height": "0.1",         # 仅实车：camera_init 抬离 world 的高度 (m)
    }

    # ------------------------------------------------------------------
    # 全局规划器 (OctoPlanner3D) 配置：把下面两个绝对路径改成你机器上的真实路径。
    # 注：octo_planner_rviz_node 自身不会拉起 RViz，需另起一个 rviz2 进程显示。
    # ------------------------------------------------------------------
    OCTO_INPUT_PCD = "/home/user/cj_fast_lio_debug/company_2F_checkroom_innerground_0.5_post/post_full_map.pcd"
    OCTO_SCENE_CACHE_DIR = "/home/user/octo-scanplanner-main/src/OctoPlanner3D-stair_detection/octomap/cache/"
    OCTO_RVIZ_CONFIG = (
        "/home/user/octo-scanplanner-main/src/OctoPlanner3D-stair_detection/octo_planner.rviz"
    )

    return LaunchDescription(
        [
            # 把上面选好的值作为默认值声明出来，命令行覆盖仍能透传给 run.launch.py
            *[
                DeclareLaunchArgument(name, default_value=value)
                for name, value in PLANNER_ARGS.items()
            ],
            DeclareLaunchArgument(
                "use_global_planner",
                default_value="true",
                description="是否启动 OctoPlanner3D 全局规划可视化节点 (octo_planner_rviz_node)。",
            ),
            DeclareLaunchArgument(
                "use_local_rviz",
                default_value="true",
                description="是否启动局部规划器 RViz2 窗口 (rviz.launch.py)。",
            ),
            DeclareLaunchArgument(
                "use_global_rviz",
                default_value="true",
                description="是否启动全局规划器 RViz2 窗口 (octo_planner.rviz)。",
            ),
            # 1) 局部规划器 + 仿真器
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, "run.launch.py")
                ),
                launch_arguments={
                    name: LaunchConfiguration(name) for name in PLANNER_ARGS
                }.items(),
            ),
            # 2) 局部规划器 RViz2（use_sim_time 与规划器保持一致）
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, "rviz.launch.py")
                ),
                launch_arguments={
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                }.items(),
                condition=IfCondition(LaunchConfiguration("use_local_rviz")),
            ),
            # 3) OctoPlanner3D 全局规划可视化节点
            #    等价于：
            #    ros2 run octo_planner3d octo_planner_rviz_node \
            #      -p input_pcd:=... -p scene_cache_dir:=... -p rebuild_map:=false
            Node(
                package="octo_planner3d",
                executable="octo_planner_rviz_node",
                name="octo_planner_rviz_node",
                output="screen",
                parameters=[{
                    "input_pcd": OCTO_INPUT_PCD,
                    "scene_cache_dir": OCTO_SCENE_CACHE_DIR,
                    "rebuild_map": False,
                }],
                condition=IfCondition(LaunchConfiguration("use_global_planner")),
            ),
            # 4) 全局规划器 RViz2（独立窗口、独立节点名，避免与上面 2) 的 rviz2 冲突）
            #    等价于：rviz2 -d <OCTO_RVIZ_CONFIG>
            Node(
                package="rviz2",
                executable="rviz2",
                name="octo_planner_rviz2",
                output="screen",
                arguments=["-d", OCTO_RVIZ_CONFIG],
                condition=IfCondition(LaunchConfiguration("use_global_rviz")),
            ),
        ]
    )
