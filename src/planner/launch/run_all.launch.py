"""Tracailer full-stack ROS 2 Jazzy launch file.

Brings up the random map generator, the trailer simulator, the MPC
controller, the planner and an rviz2 visualizer, wired together through
the same topic conventions as the legacy ROS 1 launch:

  /global_map         (PointCloud2)  global obstacles
  /local_map          (PointCloud2)  sensor window around the tractor
  /global_map_polygon (Float64MultiArray) obstacle polygons
  /trailer_odom       (planner/TrailerState)  full trailer state
  /sensor_odom        (nav_msgs/Odometry)     odometry for map sensor
  /trailer_cmd        (ackermann_msgs/AckermannDrive)  control command
  /arc_trailer_traj   (planner/ArcTrailerTraj)  planned trajectory

Parameter files live under share/planner/params and share/random_map_generator/params
and use the ROS 2 `<node>: { ros__parameters: ... }` layout with dot-style
groups (`trailer.*`, `grid_map.*`, `hybrid_astar.*`, `arc_opt.*`, `sim.*`,
`mpc.*`, `map.*`).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    planner_share = FindPackageShare("planner")
    random_map_share = FindPackageShare("random_map_generator")

    polygon_topic = LaunchConfiguration("polygon_topic")
    map_topic = LaunchConfiguration("map_topic")
    local_map_topic = LaunchConfiguration("local_map_topic")
    odom_topic = LaunchConfiguration("odom_topic")
    sensor_odom_topic = LaunchConfiguration("sensor_odom_topic")
    cmd_topic = LaunchConfiguration("cmd_topic")
    arc_traj_topic = LaunchConfiguration("arc_traj_topic")
    use_sim = LaunchConfiguration("use_sim")
    use_rviz = LaunchConfiguration("use_rviz")

    trailer_yaml = PathJoinSubstitution([planner_share, "params", "trailer.yaml"])
    grid_map_yaml = PathJoinSubstitution([planner_share, "params", "grid_map.yaml"])
    hybrid_astar_yaml = PathJoinSubstitution([planner_share, "params", "hybrid_astar.yaml"])
    optimizer_yaml = PathJoinSubstitution([planner_share, "params", "optimizer.yaml"])
    controller_yaml = PathJoinSubstitution([planner_share, "params", "controller.yaml"])
    simulator_yaml = PathJoinSubstitution([planner_share, "params", "simulator.yaml"])
    map_yaml = PathJoinSubstitution([random_map_share, "params", "map.yaml"])
    rviz_config = PathJoinSubstitution([random_map_share, "rviz", "default.rviz"])

    simulator_node = Node(
        package="planner",
        executable="simulator_node",
        name="simulator_node",
        output="screen",
        emulate_tty=True,
        condition=IfCondition(use_sim),
        parameters=[simulator_yaml, trailer_yaml],
        remappings=[
            ("cmd", cmd_topic),
            ("odom", odom_topic),
            ("sensor_odom", sensor_odom_topic),
        ],
    )

    mpc_node = Node(
        package="planner",
        executable="mpc_node",
        name="mpc_node",
        output="screen",
        emulate_tty=True,
        condition=IfCondition(use_sim),
        parameters=[controller_yaml, trailer_yaml],
        remappings=[
            ("cmd", cmd_topic),
            ("odom", odom_topic),
            ("arc_traj", arc_traj_topic),
        ],
    )

    random_map_node = Node(
        package="random_map_generator",
        executable="random_map",
        name="random_map_node",
        output="screen",
        emulate_tty=True,
        parameters=[map_yaml],
        remappings=[
            ("local_cloud", local_map_topic),
            ("global_cloud", map_topic),
            ("global_polygon", polygon_topic),
            ("odom", sensor_odom_topic),
        ],
    )

    planner_node = Node(
        package="planner",
        executable="planner_node",
        name="planner_node",
        output="screen",
        emulate_tty=True,
        parameters=[
            trailer_yaml,
            grid_map_yaml,
            hybrid_astar_yaml,
            optimizer_yaml,
        ],
        remappings=[
            ("odom", odom_topic),
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
        DeclareLaunchArgument("polygon_topic", default_value="/global_map_polygon"),
        DeclareLaunchArgument("map_topic", default_value="/global_map"),
        DeclareLaunchArgument("local_map_topic", default_value="/local_map"),
        DeclareLaunchArgument("odom_topic", default_value="/trailer_odom"),
        DeclareLaunchArgument("sensor_odom_topic", default_value="/sensor_odom"),
        DeclareLaunchArgument("cmd_topic", default_value="/trailer_cmd"),
        DeclareLaunchArgument("arc_traj_topic", default_value="/arc_trailer_traj"),
        DeclareLaunchArgument("use_sim", default_value="true"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        simulator_node,
        mpc_node,
        random_map_node,
        planner_node,
        rviz_node,
    ])
