import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    rviz_config = os.path.join(
        get_package_share_directory('basic_slam'),
        'config',
        'slam_config2.rviz'
    )

    ekf_config = os.path.join(
        get_package_share_directory('basic_slam'),
        'config',
        'ekf.yaml'
    )


    model_env = SetEnvironmentVariable(
        name='TURTLEBOT3_MODEL',
        value='burger'
    )

    imu_odom_node = Node(
        package='basic_slam',
        executable='imu_odom',
        name='imu_odom',
        parameters=[{'use_sim_time': True}]
    )

    map_node = Node(
        package='basic_slam',
        executable='map_node',
        name='map_node',
        parameters=[{'use_sim_time': True}]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[{'use_sim_time': True}, ekf_config],
    )


    amazon_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            "/home/mihai/amazon_map/aws-robomaker-small-house-world/launch/small_house.launch.py"
    )
)

    return LaunchDescription([
        model_env,
        amazon_world,
        imu_odom_node,
        map_node,
        rviz_node,
        ekf_node,
    ])