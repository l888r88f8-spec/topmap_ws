#!/usr/bin/env python3
"""Independent fixed planning replay (never runs the interactive bridge).

    source /home/robot/topmap_ws/install/setup.bash
    ros2 launch topmap_stage_demo replay.launch.py

This is the previous verified replay behavior.  ``planning.launch.py`` defaults
to interactive replanning; use ``interactive:=false`` there for the same fixed
replay without starting a second launch file.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PACKAGE = "topmap_stage_demo"


def generate_launch_description() -> LaunchDescription:
    share = get_package_share_directory(PACKAGE)
    default_stages = os.path.join(share, "data", "stages.json")
    default_planning = os.path.join(share, "data", "planning.json")
    default_pcd = os.path.join(share, "data", "scene_h_ground_original_ramp.pcd")
    default_rviz = os.path.join(share, "rviz", "planning.rviz")

    stages_path = LaunchConfiguration("stages_path")
    planning_path = LaunchConfiguration("planning_path")
    pcd_path = LaunchConfiguration("pcd_path")
    rviz_config = LaunchConfiguration("rviz_config")
    rviz = LaunchConfiguration("rviz")

    reference_frame = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="topmap_planning_demo_reference_frame",
        output="screen",
        arguments=["--x", "0", "--y", "0", "--z", "0",
                   "--roll", "0", "--pitch", "0", "--yaw", "0",
                   "--frame-id", "map", "--child-frame-id", "topmap_planning_demo_origin"],
    )

    publisher = Node(
        package=PACKAGE,
        executable="planning_publisher",
        name="topmap_planning_demo",
        output="screen",
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
            "rviz", default_value="true",
            description="Start RViz2; set rviz:=false for a headless run."),
        DeclareLaunchArgument(
            "stages_path", default_value=default_stages,
            description="Path to stages.json (final regions / portal chords)."),
        DeclareLaunchArgument(
            "planning_path", default_value=default_planning,
            description="Path to planning.json (real offline planner replay)."),
        DeclareLaunchArgument(
            "pcd_path", default_value=default_pcd,
            description="Path to the original ASCII XYZ PCD point cloud."),
        DeclareLaunchArgument(
            "rviz_config", default_value=default_rviz,
            description="Path to the planning RViz configuration."),
        reference_frame,
        publisher,
        rviz_node,
    ])
