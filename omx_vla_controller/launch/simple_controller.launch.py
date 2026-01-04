#!/usr/bin/env python3
#
# Simple Controller Launch File
#
# 핵심 동작만 수행하는 간단한 컨트롤러를 실행합니다:
# 1. 5개 joint를 const 값으로 이동
# 2. Gripper 닫기 → 열기
# 3. 카메라 이미지를 localhost:8080 서버에 API 요청
#
# 실행 방법:
#   ros2 launch omx_vla_controller simple_controller.launch.py \
#       joint_positions:="[0.0, -1.57, 1.57, 1.57, 0.0]" \
#       api_url:=http://localhost:8080/api/camera

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Launch arguments
    declared_arguments = [
        DeclareLaunchArgument(
            'joint_positions',
            default_value='[0.0, -1.57, 1.57, 1.57, 0.0]',
            description='Target joint positions (5 joints)',
        ),
        DeclareLaunchArgument(
            'api_url',
            default_value='http://localhost:8080/api/camera',
            description='API server URL',
        ),
        DeclareLaunchArgument(
            'api_timeout',
            default_value='5.0',
            description='API request timeout in seconds',
        ),
        DeclareLaunchArgument(
            'image_topic',
            default_value='/camera/image_raw',
            description='Camera image topic',
        ),
        DeclareLaunchArgument(
            'gripper_close_pos',
            default_value='0.0',
            description='Gripper close position (radians)',
        ),
        DeclareLaunchArgument(
            'gripper_open_pos',
            default_value='1.0',
            description='Gripper open position (radians)',
        ),
        DeclareLaunchArgument(
            'joint_move_duration',
            default_value='3.0',
            description='Joint movement duration in seconds',
        ),
        DeclareLaunchArgument(
            'gripper_move_duration',
            default_value='1.0',
            description='Gripper movement duration in seconds',
        ),
    ]

    # Launch configurations
    joint_positions = LaunchConfiguration('joint_positions')
    api_url = LaunchConfiguration('api_url')
    api_timeout = LaunchConfiguration('api_timeout')
    image_topic = LaunchConfiguration('image_topic')
    gripper_close_pos = LaunchConfiguration('gripper_close_pos')
    gripper_open_pos = LaunchConfiguration('gripper_open_pos')
    joint_move_duration = LaunchConfiguration('joint_move_duration')
    gripper_move_duration = LaunchConfiguration('gripper_move_duration')

    # Simple Controller Node
    simple_controller_node = Node(
        package='omx_vla_controller',
        executable='simple_controller',
        name='simple_controller',
        output='screen',
        parameters=[{
            'joint_positions': joint_positions,
            'api_url': api_url,
            'api_timeout': api_timeout,
            'image_topic': image_topic,
            'gripper_close_pos': gripper_close_pos,
            'gripper_open_pos': gripper_open_pos,
            'joint_move_duration': joint_move_duration,
            'gripper_move_duration': gripper_move_duration,
        }]
    )

    return LaunchDescription(declared_arguments + [simple_controller_node])

