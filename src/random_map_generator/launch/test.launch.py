"""Stand-alone random_map generator launch (ROS 2 Jazzy).

Starts the random_map_node with its default parameter file and an rviz2
window using the migrated default.rviz config. Useful for sanity-checking
the map generator in isolation without the planner stack.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("random_map_generator")

    map_topic = LaunchConfiguration("map_topic")
    polygon_topic = LaunchConfiguration("polygon_topic")
    use_rviz = LaunchConfiguration("use_rviz")

    map_yaml = PathJoinSubstitution([pkg_share, "params", "map.yaml"])
    rviz_config = PathJoinSubstitution([pkg_share, "rviz", "default.rviz"])

    random_map_node = Node(
        package="random_map_generator",
        executable="random_map",
        name="random_map_node",
        output="screen",
        emulate_tty=True,
        parameters=[map_yaml],
        remappings=[
            ("global_cloud", map_topic),
            ("global_polygon", polygon_topic),
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        condition=IfCondition(use_rviz),
        arguments=["-d", rviz_config],
    )

    return LaunchDescription([
        DeclareLaunchArgument("map_topic", default_value="/global_map"),
        DeclareLaunchArgument("polygon_topic", default_value="/global_map_polygon"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        random_map_node,
        rviz_node,
    ])
