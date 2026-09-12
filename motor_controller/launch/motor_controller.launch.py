from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_params = PathJoinSubstitution(
        [FindPackageShare("motor_controller"), "config", "motor_controller.yaml"]
    )

    params_file = LaunchConfiguration("params_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="Path to the motor controller parameters file",
            ),
            Node(
                package="motor_controller",
                executable="motor_controller_node",
                name="motor_controller",
                output="screen",
                parameters=[params_file],
            ),
        ]
    )
