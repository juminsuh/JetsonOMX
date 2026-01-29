#!/usr/bin/env python3
#
# VLA Full System Launch File
#
# 원격 VLA API 서버와 연동하는 전체 시스템을 실행합니다:
# 1. USB 카메라 노드
# 2. VLA Bridge 노드 (원격 API 연동)
# 3. 로봇 제어 노드 (선택적)
#
# 실행 방법:
#   ros2 launch omx_vla_controller vla_full_system.launch.py \
#       api_url:=http://your-server:8000/api/vla/infer

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Launch arguments
    declared_arguments = [
        # 카메라 설정
        DeclareLaunchArgument(
            'camera_name',
            default_value='camera',
            description='Name of the camera node',
        ),
        DeclareLaunchArgument(
            'video_device',
            default_value='/dev/video0',
            description='Video device path (e.g., /dev/video0)',
        ),
        DeclareLaunchArgument(
            'image_width',
            default_value='640',
            description='Camera image width',
        ),
        DeclareLaunchArgument(
            'image_height',
            default_value='480',
            description='Camera image height',
        ),
        # VLA Bridge 설정
        DeclareLaunchArgument(
            'api_url',
            default_value='http://localhost:8000/api/vla/infer',
            description='VLA API server URL',
        ),
        DeclareLaunchArgument(
            'api_timeout',
            default_value='5.0',
            description='API request timeout in seconds',
        ),
        DeclareLaunchArgument(
            'prompt',
            default_value='Move the robot arm to pick up the object while keeping the gripper vertical to the ground and avoiding unnecessary rotation',
            description='VLA prompt text',
        ),
        DeclareLaunchArgument(
            'request_rate',
            default_value='2.0',
            description='Maximum requests per second',
        ),
        # 로봇 제어 설정
        DeclareLaunchArgument(
            'use_robot_controller',
            default_value='false',
            description='Whether to launch robot controller node',
        ),
    ]

    # Launch configurations
    camera_name = LaunchConfiguration('camera_name')
    video_device = LaunchConfiguration('video_device')
    image_width = LaunchConfiguration('image_width')
    image_height = LaunchConfiguration('image_height')
    api_url = LaunchConfiguration('api_url')
    api_timeout = LaunchConfiguration('api_timeout')
    prompt = LaunchConfiguration('prompt')
    request_rate = LaunchConfiguration('request_rate')
    use_robot_controller = LaunchConfiguration('use_robot_controller')

    # 1. USB 카메라 노드
    camera_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='camera_node',
        parameters=[{
            'video_device': video_device,
            'image_width': image_width,
            'image_height': image_height,
            'framerate': 30.0,
            'pixel_format': 'yuyv',
            'camera_name': camera_name,
        }],
        remappings=[
            ('image_raw', '/camera/image_raw'),
            ('camera_info', '/camera/camera_info'),
        ],
        output='screen'
    )

    # 2. VLA Bridge 노드 (원격 API 연동)
    vla_bridge_node = Node(
        package='omx_vla_controller',
        executable='vla_bridge',
        name='vla_bridge',
        output='screen',
        parameters=[{
            'api_url': api_url,
            'api_timeout': api_timeout,
            'image_topic': '/camera/image_raw',
            'output_topic': '/llm_command',
            'prompt': prompt,
            'request_rate': request_rate,
            'enable_image_compression': True,
            'image_quality': 85,
        }]
    )

    # 3. 로봇 제어 노드 (선택적)
    # 기존 로봇 제어 노드를 여기에 추가할 수 있습니다
    # robot_controller_node = Node(
    #     package='your_robot_controller_package',
    #     executable='robot_controller',
    #     name='robot_controller',
    #     output='screen',
    #     condition=IfCondition(use_robot_controller),
    # )

    nodes = [
        camera_node,
        vla_bridge_node,
    ]

    return LaunchDescription(declared_arguments + nodes)

