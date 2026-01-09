#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Paths to packages and configurations
    ugv_bringup_dir = get_package_share_directory('ugv_bringup')
    ugv_vision_dir = get_package_share_directory('ugv_vision')
    ugv_description_dir = get_package_share_directory('ugv_description')
    mqtt_bridge_dir = get_package_share_directory('mqtt_bridge')
    ldlidar_dir = get_package_share_directory('ldlidar')
    
    # Configuration paths
    mqtt_config_path = os.path.join(mqtt_bridge_dir, 'config', 'params.yaml')
    
    # 2. Declare Arguments
    pub_odom_tf_arg = DeclareLaunchArgument(
        'pub_odom_tf', 
        default_value='true',
        description='Whether to publish the tf from the original odom'
    )
    
    robot_id_arg = DeclareLaunchArgument(
        'robot_id',
        default_value='robot_ugv_beast_v1',
        description='Unique ID for the Cyberwave cloud'
    )

    use_lidar_arg = DeclareLaunchArgument(
        'use_lidar',
        default_value='false',
        description='Whether to start the LiDAR driver'
    )

    # 3. Core Hardware Node (Unified Bringup)
    # Handles Serial communication for Telemetry and Commands
    bringup_node = Node(
        package='ugv_bringup',
        executable='ugv_bringup',
        name='ugv_bringup',
        output='screen',
        remappings=[
            ('cmd_vel', '/cmd_vel'),
            ('ugv/pt_ctrl', '/ugv/pt_ctrl'),
            ('ugv/led_ctrl', '/ugv/led_ctrl'),
            ('voltage', '/voltage'),
            ('imu/data_raw', '/imu/data_raw'),
            ('imu/mag', '/imu/mag'),
            ('odom/odom_raw', '/odom/odom_raw'),
        ]
    )

    # 4. IMU Filtering
    # Processes raw IMU data into a stable orientation
    imu_filter_node = Node(
        package='imu_complementary_filter',
        executable='complementary_filter_node',
        name='complementary_filter_gain_node',
        output='screen',
        parameters=[
            {'do_bias_estimation': True},
            {'do_adaptive_gain': True},
            {'use_mag': False},
            {'gain_acc': 0.01},
            {'gain_mag': 0.01},
        ]
    )

    # 5. Lidar Driver
    # Includes the dedicated lidar launch file
    from launch.conditions import IfCondition
    laser_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ldlidar_dir, 'launch', 'ldlidar.launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('use_lidar'))
    )

    # 6. Robot Description & Transforms
    # Publishes the 3D model and static transforms
    robot_state_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ugv_description_dir, 'launch', 'display.launch.py')
        ),
        launch_arguments={'use_rviz': 'false'}.items()
    )

    # 7. Odometry Calculator
    # Computes raw odometry from wheel encoders
    base_node = Node(
        package='ugv_base_node',
        executable='base_node',
        name='base_node',
        parameters=[{'pub_odom_tf': LaunchConfiguration('pub_odom_tf')}],
        remappings=[
            ('imu/data', '/imu/data'),
            ('odom/odom_raw', '/odom/odom_raw'),
            ('odom', '/odom')
        ]
    )

    # 8. Cloud Connectivity (MQTT Bridge)
    mqtt_bridge_node = Node(
        package='mqtt_bridge',
        executable='mqtt_bridge_node',
        name='mqtt_bridge_node',
        parameters=[mqtt_config_path, {'robot_id': LaunchConfiguration('robot_id')}],
        output='screen'
    )

    # 9. Video Streaming (Camera)
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ugv_vision_dir, 'launch', 'camera.launch.py')
        )
    )

    return LaunchDescription([
        pub_odom_tf_arg,
        robot_id_arg,
        use_lidar_arg,
        bringup_node,
        imu_filter_node,
        laser_launch,
        robot_state_launch,
        base_node,
        mqtt_bridge_node,
        camera_launch
    ])
