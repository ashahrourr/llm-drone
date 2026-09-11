"""Bring up the whole graph: vehicle, guard, planner.

    ros2 launch drone_agent mission.launch.py \
        mission:="take off to 8m, fly 15m north and 10m east, circle it, come home"

Swap `sim_node` for PX4 SITL to fly the same graph against real firmware — the
topics and the NED conventions are unchanged.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    mission = LaunchConfiguration("mission")
    use_llm = LaunchConfiguration("use_llm")

    return LaunchDescription([
        DeclareLaunchArgument(
            "mission",
            default_value="take off to 8m, fly 15m north and 10m east, circle it, come home",
            description="the mission, in plain English"),
        DeclareLaunchArgument("use_llm", default_value="false",
                              description="use an LLM planner instead of the offline one"),
        DeclareLaunchArgument("max_altitude", default_value="30.0"),
        DeclareLaunchArgument("geofence_radius", default_value="60.0"),

        Node(package="drone_agent", executable="sim_node", name="drone_sim",
             output="screen"),
        Node(package="drone_agent", executable="guard_node", name="drone_guard",
             output="screen",
             parameters=[{"max_altitude": LaunchConfiguration("max_altitude"),
                          "geofence_radius": LaunchConfiguration("geofence_radius")}]),
        Node(package="drone_agent", executable="planner_node", name="drone_planner",
             output="screen", parameters=[{"use_llm": use_llm}]),

        # Give the graph a moment to finish discovery before the mission lands,
        # otherwise the planner can miss a latched-looking first message.
        TimerAction(period=3.0, actions=[
            ExecuteProcess(
                cmd=["ros2", "topic", "pub", "--once", "/drone/mission",
                     "std_msgs/msg/String", ["{data: '", mission, "'}"]],
                output="screen"),
        ]),
    ])
