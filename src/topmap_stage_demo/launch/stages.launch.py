#!/usr/bin/env python3
"""Launch the overlaid TopMap stage demo: publisher + RViz.

All paths come from this package's installed share directory, never from the
old ``/home/robot/paper_figures`` tree.

Usage:
    source /home/robot/topmap_ws/install/setup.bash
    ros2 launch topmap_stage_demo stages.launch.py
    ros2 launch topmap_stage_demo stages.launch.py rviz:=false   # headless
    ros2 launch topmap_stage_demo stages.launch.py \
        data_path:=/abs/stages.json pcd_path:=/abs/scene.pcd
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
    default_data = os.path.join(share, "data", "stages.json")
    default_pcd = os.path.join(share, "data", "scene_h_ground_original_ramp.pcd")
    default_rviz = os.path.join(share, "rviz", "stages.rviz")

    data_path = LaunchConfiguration("data_path")
    pcd_path = LaunchConfiguration("pcd_path")
    rviz_config = LaunchConfiguration("rviz_config")
    rviz = LaunchConfiguration("rviz")

    reference_frame = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="topmap_stage_demo_reference_frame",
        output="screen",
        arguments=["--x", "0", "--y", "0", "--z", "0",
                   "--roll", "0", "--pitch", "0", "--yaw", "0",
                   "--frame-id", "map", "--child-frame-id", "topmap_stage_demo_origin"],
    )

    publisher = Node(
        package=PACKAGE,
        executable="stage_publisher",
        name="topmap_stage_demo",
        output="screen",
        arguments=["--data", data_path, "--pcd", pcd_path],
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
            "data_path", default_value=default_data,
            description="Path to the stages.json algorithm output."),
        DeclareLaunchArgument(
            "pcd_path", default_value=default_pcd,
            description="Path to the original ASCII XYZ PCD point cloud."),
        DeclareLaunchArgument(
            "rviz_config", default_value=default_rviz,
            description="Path to the RViz layer configuration."),
        reference_frame,
        publisher,
        rviz_node,
    ])
