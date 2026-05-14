"""Launch the Contest 3 PickPlace demo: gz sim + Piper via xacro + ros2_control."""
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default=True)
    world_sdf = "/data/nedo/gazebo_sim/worlds/pickplace_contest3.sdf"
    piper_xacro = "/data/nedo/gazebo_sim/urdf/piper_gz.urdf.xacro"

    robot_description_content = Command(
        [PathJoinSubstitution([FindExecutable(name="xacro")]), " ", piper_xacro]
    )
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": True}],
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])
        ]),
        launch_arguments=[("gz_args", f" -r -s -v 2 {world_sdf}")],
    )

    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic", "robot_description",
            "-name", "piper",
            "-x", "0", "-y", "0", "-z", "0",
            "-allow_renaming", "false",
        ],
    )

    jsb = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )
    arm_ctl = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller"],
    )
    grip_ctl = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller"],
    )

    # Bridge clock + cinematic camera image
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ],
        output="screen",
    )
    image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["/cinematic"],
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        gz_sim,
        rsp,
        bridge,
        image_bridge,
        TimerAction(period=4.0, actions=[spawn]),
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[jsb])),
        RegisterEventHandler(OnProcessExit(target_action=jsb, on_exit=[arm_ctl, grip_ctl])),
    ])
