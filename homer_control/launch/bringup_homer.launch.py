from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    urdf_package_path = get_package_share_path("homer_description")
    control_package_path = get_package_share_path("homer_control")
    model_path = urdf_package_path / "urdf/homer.urdf.xacro"
    rviz_config_path = urdf_package_path / "rviz/homer.rviz"

    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=str(model_path),
        description="Absolute path to robot urdf file",
    )
    sim_time_arg = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="false",
        choices=["true", "false"],
        description="Flag to enable use simulation time",
    )
    rviz_arg = DeclareLaunchArgument(
        name="rvizconfig",
        default_value=str(rviz_config_path),
        description="Absolute path to rviz config file",
    )

    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(control_package_path / "launch/rplidar.launch.py")
        ),
    )

    driver_node = Node(package="homer_control", executable="driver")

    robot_description = ParameterValue(
        Command(["xacro ", LaunchConfiguration("model")]), value_type=str
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "robot_description": robot_description,
            }
        ],
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rvizconfig")],
    )

    return LaunchDescription(
        [
            sim_time_arg,
            model_arg,
            rviz_arg,
            driver_node,
            rplidar_launch,
            # joint_state_publisher_node,
            # robot_state_publisher_node,
            # rviz_node,
        ]
    )
