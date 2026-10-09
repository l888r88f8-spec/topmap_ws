"""Offline map build launch for the topmap stack (T32).

Starts the ``map_builder_node`` executable, which serves ``/topmap/build_map``
and publishes the raw cloud, region/portal markers and map stats. The custom
algorithm YAML is passed as the node's *string* ``config_file`` parameter (not
as a ROS parameter file), so ``topmap::loadTopmapConfig`` performs the strict
key validation.

    ros2 launch topmap_bringup topmap_offline_build.launch.py \
        pcd_path:=test_data/synthetic/scene_hall_narrow_hall.pcd \
        output_map_path:=test_data/maps/hall.bin
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

_DEFAULT_CONFIG = PathJoinSubstitution(
    [FindPackageShare("topmap_bringup"), "config", "topmap.yaml"]
)


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=_DEFAULT_CONFIG,
                description="Path to the topmap algorithm YAML (string node parameter).",
            ),
            DeclareLaunchArgument(
                "pcd_path",
                default_value="",
                description="Input GroundMap PCD for the offline build.",
            ),
            DeclareLaunchArgument(
                "output_map_path",
                default_value="",
                description="Destination path of the versioned topmap_map.bin archive.",
            ),
            Node(
                package="topmap_ros",
                executable="map_builder_node",
                name="topmap_map_builder",
                output="screen",
                parameters=[
                    {
                        "config_file": LaunchConfiguration("config_file"),
                        "pcd_path": LaunchConfiguration("pcd_path"),
                        "output_map_path": LaunchConfiguration("output_map_path"),
                        "frame_id": "map",
                    }
                ],
            ),
        ]
    )
