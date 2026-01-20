import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    pkg_my_robot = get_package_share_directory('my_robot')
    
    # 1. Start LiDAR FIRST (With Auto-Restart)
    # If it crashes due to power dip, it will respawn automatically
    lidar_node = Node(
        package='ldlidar_ros2',
        executable='ldlidar_ros2_node',
        name='ldlidar_publisher_ld19',
        output='screen',
        respawn=True,       # <--- CRITICAL: Auto-restart if it dies
        respawn_delay=2.0,  # Wait 2s before retrying
        parameters=[
            {'product_name': 'LDLiDAR_LD19'},
            {'laser_scan_topic_name': 'scan'},
            {'point_cloud_2d_topic_name': 'pointcloud2d'},
            {'frame_id': 'base_laser'},
            {'port_name': '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0'},
            {'serial_baudrate': 230400},
            {'laser_scan_dir': True},
            {'range_min': 0.02},
            {'range_max': 12.0}
        ]
    )

    # 2. Static Transform
    tf_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments = ['--x', '0.10', '--y', '0', '--z', '0.18', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'base_link', '--child-frame-id', 'base_laser']
    )

    # 3. Start Hardware Driver (Motors) -- DELAYED BY 5 SECONDS
    # This gives the power supply time to stabilize after Lidar start
    driver_node = Node(
        package='my_robot',
        executable='hardware_driver',
        output='screen'
    )
    
    delayed_driver = TimerAction(
        period=5.0,
        actions=[driver_node]
    )

    # 4. Start SLAM -- DELAYED BY 8 SECONDS
    slam_params_file = os.path.join(pkg_my_robot, 'config', 'mapper_params_online_async.yaml')
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_file, {'use_sim_time': False}]
    )
    
    delayed_slam = TimerAction(
        period=8.0,
        actions=[slam_toolbox]
    )

    return LaunchDescription([
        lidar_node,
        tf_laser,
        delayed_driver,
        delayed_slam
    ])
