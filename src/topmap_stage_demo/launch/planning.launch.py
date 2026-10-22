#!/usr/bin/env python3
"""One-command interactive TopMap planning in RViz.

Default (new command):
    source /home/robot/topmap_ws/install/setup.bash
    ros2 launch topmap_stage_demo planning.launch.py

The default starts the real production ``topmap_ros planner_node`` with the
synthetic ``stages.topmap.bin`` archive and the isolated
``/topmap_stage_demo/plan_path`` service, plus the RViz click bridge.  Use RViz's
2D Pose Estimate (``/topmap_stage_demo/initialpose``) for the start and 2D Goal Pose
(``/topmap_stage_demo/goal_pose``) for the goal; every valid endpoint change replans.

Opt-in fixed replay (never started together with the interactive bridge):
    ros2 launch topmap_stage_demo planning.launch.py interactive:=false
    # or independently:
    ros2 launch topmap_stage_demo replay.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PACKAGE = "topmap_stage_demo"
SERVICE_NAME = "/topmap_stage_demo/plan_path"


def generate_launch_description() -> LaunchDescription:
    share = get_package_share_directory(PACKAGE)
    default_stages = os.path.join(share, "data", "stages.json")
    default_map = os.path.join(share, "data", "stages.topmap.bin")
    default_config = os.path.join(share, "data", "demo_config.yaml")
    default_planning = os.path.join(share, "data", "planning.json")
    default_pcd = os.path.join(share, "data", "scene_h_ground_original_ramp.pcd")
    default_rviz = os.path.join(share, "rviz", "planning.rviz")

    interactive = LaunchConfiguration("interactive")
    rviz = LaunchConfiguration("rviz")
    stages_path = LaunchConfiguration("stages_path")
    map_path = LaunchConfiguration("map_path")
    config_path = LaunchConfiguration("config_path")
    planning_path = LaunchConfiguration("planning_path")
    pcd_path = LaunchConfiguration("pcd_path")
    service_name = LaunchConfiguration("service_name")
    rviz_config = LaunchConfiguration("rviz_config")

    reference_frame = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="topmap_planning_demo_reference_frame",
        output="screen",
        arguments=["--x", "0", "--y", "0", "--z", "0",
                   "--roll", "0", "--pitch", "0", "--yaw", "0",
                   "--frame-id", "map", "--child-frame-id", "topmap_planning_demo_origin"],
    )

    # Production planner, unchanged algorithm.  Its default service/publisher
    # names are remapped into this demo so a real navigation stack is untouched.
    planner_backend = Node(
        package="topmap_ros",
        executable="planner_node",
        name="topmap_stage_demo_planner",
        output="screen",
        condition=IfCondition(interactive),
        parameters=[{
            "map_path": map_path,
            "config_file": config_path,
            "frame_id": "map",
        }],
        remappings=[
            ("/topmap/plan_path", service_name),
            ("/topmap/final_path", "/topmap_stage_demo/planner/final_path"),
            ("/topmap/topology_path", "/topmap_stage_demo/planner/topology_path"),
            ("/topmap/corridor", "/topmap_stage_demo/planner/corridor"),
            ("/topmap/start_goal", "/topmap_stage_demo/planner/start_goal"),
        ],
    )

    interactive_bridge = Node(
        package=PACKAGE,
        executable="interactive_planner",
        name="topmap_interactive_planner",
        output="screen",
        condition=IfCondition(interactive),
        arguments=[
            "--stages", stages_path,
            "--pcd", pcd_path,
            "--service", service_name,
        ],
    )

    replay_publisher = Node(
        package=PACKAGE,
        executable="planning_publisher",
        name="topmap_planning_demo",
        output="screen",
        condition=UnlessCondition(interactive),
        arguments=[
            "--stages", stages_path,
            "--planning", planning_path,
            "--pcd", pcd_path,
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        condition=IfCondition(rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "interactive", default_value="true",
            description="Real replanning from RViz clicks (default); false replays planning.json."),
        DeclareLaunchArgument(
            "rviz", default_value="true",
            description="Start RViz2; set rviz:=false for a headless run."),
        DeclareLaunchArgument(
            "stages_path", default_value=default_stages,
            description="stages.json whose actual mesh resolves click XY -> surface Z."),
        DeclareLaunchArgument(
            "map_path", default_value=default_map,
            description="Loadable MapArchive with the same synthetic mesh/topology."),
        DeclareLaunchArgument(
            "config_path", default_value=default_config,
            description="TopmapConfig YAML for the production planner_node."),
        DeclareLaunchArgument(
            "planning_path", default_value=default_planning,
            description="Fixed replay result used only when interactive:=false."),
        DeclareLaunchArgument(
            "pcd_path", default_value=default_pcd,
            description="Original synthetic PCD for the static cloud."),
        DeclareLaunchArgument(
            "service_name", default_value=SERVICE_NAME,
            description="Isolated PlanPath service name (avoids /topmap/plan_path)."),
        DeclareLaunchArgument(
            "rviz_config", default_value=default_rviz,
            description="Path to the verified planning RViz configuration."),
        reference_frame,
        planner_backend,
        interactive_bridge,
        replay_publisher,
        rviz_node,
    ])
