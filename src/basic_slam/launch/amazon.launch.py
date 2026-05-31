import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    rviz_config = os.path.join(
        get_package_share_directory('basic_slam'),
        'config',
        'slam_config2.rviz'
    )

    # gz world name -> ground-truth pose topic. If GT is silent, run
    # `gz topic -l | grep dynamic_pose` and pass world:=<name>.
    world = LaunchConfiguration('world')
    world_arg = DeclareLaunchArgument('world', default_value='default')

    model_env = SetEnvironmentVariable(
        name='TURTLEBOT3_MODEL',
        value='burger'
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

    amazon_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            "/home/mihai/amazon_map/aws-robomaker-small-house-world/launch/small_house.launch.py"
    )
)

    return LaunchDescription([
        world_arg,
        model_env,
        amazon_world,
        imu_odom_node,
        map_node,
        rviz_node,
        gt_bridge,
        ground_truth_node,
        metrics_node,
    ])