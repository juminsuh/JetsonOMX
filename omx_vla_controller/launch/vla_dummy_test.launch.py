#!/usr/bin/env python3
#
# VLA Dummy Test Launch File
#
# 이 Launch 파일은 다음을 통합 실행합니다:
# 1. USB 카메라 노드 (usb_cam)
# 2. VLA Dummy Node (가짜 추론 노드)
# 3. 로봇 컨트롤러 (기존 제어 노드 - 선택적)
#
# 실행 방법:
#   ros2 launch omx_vla_controller vla_dummy_test.launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Launch arguments
    declared_arguments = [
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
    use_robot_controller = LaunchConfiguration('use_robot_controller')

    # 1. USB 카메라 노드 (표준 ROS 2 패키지)
    # usb_cam 패키지의 카메라 노드 실행
    # 
    # 참고: 기존 open_manipulator_bringup/launch/camera_usb_cam.launch.py를
    # 사용하려면 카메라 이름을 'camera'로 설정하거나, VLA Dummy Node에서
    # 해당 토픽명을 구독하도록 수정해야 합니다.
    camera_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='camera_node',
        parameters=[{
            'video_device': video_device,
            'image_width': image_width,
            'image_height': image_height,
            'framerate': 30.0,
            'pixel_format': 'yuyv',  # 또는 'mjpeg'
            'camera_name': camera_name,
        }],
        remappings=[
            # 카메라 토픽을 /camera/image_raw로 통일
            # VLA Dummy Node가 이 토픽을 구독합니다
            ('image_raw', '/camera/image_raw'),
            ('camera_info', '/camera/camera_info'),
        ],
        output='screen'
    )

    # 2. VLA Dummy Node (가짜 추론 노드)
    vla_dummy_node = Node(
        package='omx_vla_controller',
        executable='vla_dummy',
        name='vla_dummy',
        output='screen',
        parameters=[{
            # 필요시 파라미터 추가
        }]
    )

    # 3. 로봇 컨트롤러 노드 (선택적)
    # 기존 LLM 레포에서 가져온 컨트롤러를 실행하려면
    # 여기에 추가할 수 있습니다.
    # 예시:
    # robot_controller_node = Node(
    #     package='open_manipulator_teleop',
    #     executable='test_control_node',
    #     name='omx_controller',
    #     output='screen',
    #     condition=IfCondition(use_robot_controller),
    # )

    nodes = [
        camera_node,
        vla_dummy_node,
    ]

    return LaunchDescription(declared_arguments + nodes)

