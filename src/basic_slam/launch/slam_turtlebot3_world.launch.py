import os
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    rviz_config = os.path.join(get_package_share_directory('basic_slam'), 'config', 'slam_config2.rviz')
    turtlebot3_world = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch', 'turtlebot3_world.launch.py')
    ekf_config = os.path.join(get_package_share_directory('basic_slam'), 'config', 'ekf.yaml')

    # gz world name -> ground-truth pose topic. If GT is silent, run
    # `gz topic -l | grep dynamic_pose` and pass world:=<name>.
    world = LaunchConfiguration('world')
    world_arg = DeclareLaunchArgument('world', default_value='default')

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

    # Relays /odom -> /odom_cov (the topic the EKF's odom0 subscribes to).
    odom_cov_node = Node(
        package='basic_slam',
        executable='odom_cov',
        name='odom_cov',
        parameters=[{'use_sim_time': True}]
    )

    # Bridge gz true model poses -> ROS TFMessage on /gz_pose_tf.
    gt_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gt_pose_bridge',
        arguments=[['/world/', world, '/dynamic_pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V']],
        remappings=[(['/world/', world, '/dynamic_pose/info'], '/gz_pose_tf')],
        parameters=[{'use_sim_time': True}]
    )

    # Pick the robot's transform and republish it as /ground_truth odometry.
    ground_truth_node = Node(
        package='basic_slam',
        executable='ground_truth',
        name='ground_truth',
        output='screen',
        parameters=[{'use_sim_time': True, 'child_frame': 'burger', 'frame_id': 'map'}]
    )

    metrics_node = Node(
        package='basic_slam',
        executable='metrics',
        name='map_metrics',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    gazebo = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        turtlebot3_world
    )
)

    return LaunchDescription([
        world_arg,
        model_env,
        imu_odom_node,
        map_node,
        rviz_node,
        ekf_node,
        odom_cov_node,
        gazebo,
        gt_bridge,
        ground_truth_node,
        metrics_node,
    ])