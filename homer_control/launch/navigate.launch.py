from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    homer_control_package_path = get_package_share_path('homer_control')
    slam_config_path = homer_control_package_path / 'configs/localization_params.yaml'
    nav_config_path = homer_control_package_path / 'configs/nav2_params.yaml'
    rviz_config_path = homer_control_package_path / 'rviz/localization.rviz'

    sim_time_arg = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='false',
        choices=['true', 'false'],
        description="Flag to enable use simulation time",
    )
    rviz_arg = DeclareLaunchArgument(
        name='rvizconfig',
        default_value=str(rviz_config_path),
        description="Absolute path to rviz config file",
    )

    launch_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(get_package_share_path('slam_toolbox') / 'launch/localization_launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'slam_params_file': str(slam_config_path),
        }.items()    
    )

    launch_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(get_package_share_path('nav2_bringup') / 'launch/navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': str(nav_config_path),
        }.items()
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    return LaunchDescription(
        [
            sim_time_arg,
            rviz_arg,
            launch_slam,
            rviz_node,
            launch_navigation,
        ]
    )
