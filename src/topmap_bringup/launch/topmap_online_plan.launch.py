"""Online plan launch for the topmap stack (T32).

Starts the ``planner_node`` executable, which loads a previously built map
archive and serves ``/topmap/plan_path`` while publishing the final path and
the topology/corridor/start-goal markers. As with the offline launch, the
algorithm YAML travels as the node's *string* ``config_file`` parameter.

    ros2 launch topmap_bringup topmap_online_plan.launch.py \
        map_path:=test_data/maps/hall.bin
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
                "map_path",
                default_value="",
                description="Path of the versioned topmap_map.bin archive to plan on.",
            ),
            Node(
                package="topmap_ros",
                executable="planner_node",
                name="topmap_planner",
                output="screen",
                parameters=[
                    {
                        "config_file": LaunchConfiguration("config_file"),
                        "map_path": LaunchConfiguration("map_path"),
                        "frame_id": "map",
                    }
                ],
            ),
        ]
    )
