import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    rviz_config = os.path.join(get_package_share_directory('basic_slam'), 'config', 'slam_config2.rviz')

    imu_odom_node = Node(
        package='basic_slam',
        executable='imu_odom',
        name='imu_odom'
    )

    map_node = Node(
        package='basic_slam',
        executable='map_node',
        name='map_node'
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    return LaunchDescription([
        imu_odom_node,
        map_node,
        rviz_node
    ])