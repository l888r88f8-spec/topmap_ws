"""RViz2 launch for the topmap stack (T32).

Loads the packaged ``rviz/topmap.rviz`` layout, which subscribes to the frozen
visualization contract:

  * ``sensor_msgs/PointCloud2``   ``/topmap/original_cloud``
  * ``visualization_msgs/MarkerArray`` ``/topmap/regions``
  * ``visualization_msgs/MarkerArray`` ``/topmap/portals``
  * ``visualization_msgs/MarkerArray`` ``/topmap/topology_path``
  * ``visualization_msgs/MarkerArray`` ``/topmap/corridor``
  * ``visualization_msgs/MarkerArray`` ``/topmap/start_goal``
  * ``nav_msgs/Path``             ``/topmap/final_path``

    ros2 launch topmap_bringup topmap_rviz.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

_DEFAULT_RVIZ_CONFIG = PathJoinSubstitution(
    [FindPackageShare("topmap_bringup"), "rviz", "topmap.rviz"]
)


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "rviz_config",
                default_value=_DEFAULT_RVIZ_CONFIG,
                description="Path to the RViz2 layout to load.",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", LaunchConfiguration("rviz_config")],
            ),
        ]
    )
