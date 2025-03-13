from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription

# from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node

# from launch_ros.parameter_descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    control_package_path = get_package_share_path("homer_control")

    sim_time_arg = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="false",
        choices=["true", "false"],
        description="Flag to enable use simulation time",
    )

    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(control_package_path / "launch/rplidar.launch.py")
        ),
    )

    static_tf_footprint_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--x",
            "0",
            "--y",
            "0",
            "--z",
            "-0.0375",
            "--yaw",
            "0",
            "--pitch",
            "0",
            "--roll",
            "0",
            "--frame-id",
            "base_link",
            "--child-frame-id",
            "base_footprint",
        ],
    )

    driver_node = Node(package="homer_control", executable="driver")

    return LaunchDescription(
        [
            sim_time_arg,
            static_tf_footprint_node,
            driver_node,
            rplidar_launch,
        ]
    )
