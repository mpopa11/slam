import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    rviz_config = os.path.join(get_package_share_directory('basic_slam'), 'config', 'slam_config2.rviz')
    ekf_config = os.path.join(get_package_share_directory('basic_slam'), 'config', 'ekf.yaml')

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

    # Static transforms to view the tf tree in rviz from rosbag
    base_footprint_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '-0.010', '0', '0', '0', 'base_link', 'base_footprint'],
        parameters=[{'use_sim_time': True}]
    )

    imu_link_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['-0.032', '0', '0.068', '0', '0', '0', 'base_link', 'imu_link'],
        parameters=[{'use_sim_time': True}]
    )

    base_scan_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['-0.032', '0', '0.172', '0', '0', '0', 'base_link', 'base_scan'],
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        base_footprint_tf,
        imu_link_tf,
        base_scan_tf,
        imu_odom_node,
        map_node,
        rviz_node,
        ekf_node,
    ])