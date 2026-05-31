import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    rviz_config = os.path.join(get_package_share_directory('basic_slam'), 'config', 'slam_config2.rviz')

    # gz world name -> ground-truth pose topic. Note: this launch has no gz sim
    # (it replays bags), so /ground_truth only populates if a live gz sim is
    # publishing /world/<world>/dynamic_pose/info. If GT is silent, run
    # `gz topic -l | grep dynamic_pose` and pass world:=<name>.
    world = LaunchConfiguration('world')
    world_arg = DeclareLaunchArgument('world', default_value='default')

    gt_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gt_pose_bridge',
        arguments=[['/world/', world, '/dynamic_pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V']],
        remappings=[(['/world/', world, '/dynamic_pose/info'], '/gz_pose_tf')],
        parameters=[{'use_sim_time': True}]
    )

    ground_truth_node = Node(
        package='basic_slam',
        executable='ground_truth',
        name='ground_truth',
        output='screen',
        parameters=[{'use_sim_time': True, 'child_frame': 'burger', 'frame_id': 'map'}]
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

    metrics_node = Node(
        package='basic_slam',
        executable='metrics',
        name='map_metrics',
        output='screen',
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
        world_arg,
        base_footprint_tf,
        imu_link_tf,
        base_scan_tf,
        imu_odom_node,
        map_node,
        rviz_node,
        metrics_node,
        gt_bridge,
        ground_truth_node,
    ])