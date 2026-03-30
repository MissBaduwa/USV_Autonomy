#!/usr/bin/env python3
"""
USV JETSON NANO LAUNCH FILE
GPS + Accelerometer + Camera Navigation
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Model path
    model_path = os.path.expanduser('~/jetson_usv_final/models/water_surface_detection.onnx')
    
    return LaunchDescription([
        # GPS Driver (NMEA)
        Node(
            package='nmea_navsat_driver',
            executable='nmea_serial_driver',
            name='gps_driver',
            output='screen',
            parameters=[{
                'port': '/dev/ttyUSB0',
                'baud': 9600,
                'frame_id': 'gps_link',
                'use_imu': False
            }]
        ),
        
        # Camera Driver
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='camera_driver',
            output='screen',
            parameters=[{
                'video_device': '/dev/video0',
                'image_width': 640,
                'image_height': 480,
                'pixel_format': 'yuyv',
                'framerate': 30.0
            }]
        ),
        
        # Perception Node
        Node(
            package='usv_autonomy',
            executable='perception_node.py',
            name='perception_node',
            output='screen',
            parameters=[{
                'model_path': model_path,
                'confidence_threshold': 0.25,
                'image_topic': '/camera/image_raw',
                'img_size': 640
            }]
        ),
        
        # Navigation Node (GPS version)
        Node(
            package='usv_autonomy',
            executable='navigation_node.py',
            name='navigation_node',
            output='screen'
        ),
        
        # Supervisor Node
        Node(
            package='usv_autonomy',
            executable='supervisor_node.py',
            name='supervisor_node',
            output='screen'
        )
    ])
