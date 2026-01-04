#!/usr/bin/env python3
"""
Simple Controller Node - 핵심 동작만 수행
1. 5개 joint를 const 값으로 설정하여 이동
2. Gripper 닫기 → 열기
3. 카메라 이미지를 localhost:8080 서버에 API 요청
"""

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import GripperCommand
from control_msgs.msg import GripperCommand as GripperCommandMsg
from rclpy.action import ActionClient
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import base64
import requests
import math
import time


class SimpleControllerNode(Node):
    """간단한 로봇 제어 노드"""

    def __init__(self):
        super().__init__('simple_controller_node')

        # 파라미터 선언
        self.declare_parameter('joint_positions', [0.0, -1.57, 1.57, 1.57, 0.0])
        self.declare_parameter('api_url', 'http://localhost:8080/api/camera')
        self.declare_parameter('api_timeout', 5.0)
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('gripper_close_pos', 0.0)  # 라디안
        self.declare_parameter('gripper_open_pos', 1.0)  # 라디안
        self.declare_parameter('joint_move_duration', 3.0)  # 초
        self.declare_parameter('gripper_move_duration', 1.0)  # 초

        # 파라미터 로드
        self.joint_positions = self.get_parameter('joint_positions').value
        self.api_url = self.get_parameter('api_url').value
        self.api_timeout = self.get_parameter('api_timeout').value
        image_topic = self.get_parameter('image_topic').value
        self.gripper_close_pos = self.get_parameter('gripper_close_pos').value
        self.gripper_open_pos = self.get_parameter('gripper_open_pos').value
        self.joint_move_duration = self.get_parameter('joint_move_duration').value
        self.gripper_move_duration = self.get_parameter('gripper_move_duration').value

        # Publisher for arm joint control
        self.arm_publisher = self.create_publisher(
            JointTrajectory,
            '/arm_controller/joint_trajectory',
            10
        )

        # Action client for GripperCommand
        self.gripper_client = ActionClient(
            self,
            GripperCommand,
            '/gripper_controller/gripper_cmd'
        )

        # 카메라 이미지 구독
        self.subscription = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            10
        )

        # CV Bridge
        self.cv_bridge = CvBridge()
        self.latest_image = None

        # 상태 관리
        self.state = 'idle'  # idle, moving_joints, closing_gripper, opening_gripper, sending_api
        self.joint_move_complete = False
        self.gripper_close_complete = False
        self.gripper_open_complete = False

        self.get_logger().info("🚀 Simple Controller Node Started!")
        self.get_logger().info(f"📡 API URL: {self.api_url}")
        self.get_logger().info(f"📸 Image topic: {image_topic}")
        self.get_logger().info(f"🎯 Target joint positions: {self.joint_positions}")

        # 시작: Joint 이동
        self.get_logger().info("Starting sequence: Moving joints...")
        self.move_joints()

    def move_joints(self):
        """5개 joint를 설정된 위치로 이동"""
        self.get_logger().info(f"Moving joints to: {self.joint_positions}")

        traj = JointTrajectory()
        traj.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']

        pt = JointTrajectoryPoint()
        pt.positions = self.joint_positions
        pt.time_from_start.sec = int(self.joint_move_duration)
        pt.time_from_start.nanosec = int((self.joint_move_duration % 1) * 1e9)

        traj.points.append(pt)
        self.arm_publisher.publish(traj)

        self.state = 'moving_joints'
        self.get_logger().info("Joint trajectory published")

        # Joint 이동 완료 대기 (타이머 사용)
        self.create_timer(self.joint_move_duration + 0.5, self.on_joint_move_complete, oneshot=True)

    def on_joint_move_complete(self):
        """Joint 이동 완료 후 Gripper 닫기"""
        self.get_logger().info("✅ Joint movement completed. Closing gripper...")
        self.close_gripper()

    def close_gripper(self):
        """Gripper 닫기"""
        self.get_logger().info(f"Closing gripper to position: {self.gripper_close_pos}")

        goal = GripperCommand.Goal()
        goal.command = GripperCommandMsg()
        goal.command.position = self.gripper_close_pos
        goal.command.max_effort = 1.0

        self.gripper_client.wait_for_server(timeout_sec=5.0)
        send_goal_future = self.gripper_client.send_goal_async(goal)
        send_goal_future.add_done_callback(self.gripper_close_response_callback)

        self.state = 'closing_gripper'

    def gripper_close_response_callback(self, future):
        """Gripper 닫기 응답 콜백"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Gripper close goal rejected")
            return

        self.get_logger().info("Gripper close goal accepted")
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.gripper_close_result_callback)

    def gripper_close_result_callback(self, future):
        """Gripper 닫기 결과 콜백"""
        result = future.result().result
        self.get_logger().info("✅ Gripper closed. Opening gripper...")
        self.gripper_close_complete = True

        # Gripper 열기
        time.sleep(0.5)  # 짧은 대기
        self.open_gripper()

    def open_gripper(self):
        """Gripper 열기"""
        self.get_logger().info(f"Opening gripper to position: {self.gripper_open_pos}")

        goal = GripperCommand.Goal()
        goal.command = GripperCommandMsg()
        goal.command.position = self.gripper_open_pos
        goal.command.max_effort = 1.0

        send_goal_future = self.gripper_client.send_goal_async(goal)
        send_goal_future.add_done_callback(self.gripper_open_response_callback)

        self.state = 'opening_gripper'

    def gripper_open_response_callback(self, future):
        """Gripper 열기 응답 콜백"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Gripper open goal rejected")
            return

        self.get_logger().info("Gripper open goal accepted")
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.gripper_open_result_callback)

    def gripper_open_result_callback(self, future):
        """Gripper 열기 결과 콜백"""
        result = future.result().result
        self.get_logger().info("✅ Gripper opened. Sending camera image to API...")
        self.gripper_open_complete = True

        # API 요청 전송
        time.sleep(0.5)  # 짧은 대기
        self.send_camera_to_api()

    def image_callback(self, msg):
        """카메라 이미지 수신 콜백"""
        self.latest_image = msg

    def image_to_base64(self, cv_image):
        """OpenCV 이미지를 Base64 문자열로 변환"""
        # JPEG 압축
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 85]
        success, encoded_image = cv2.imencode('.jpg', cv_image, encode_param)

        if not success:
            self.get_logger().error("Failed to encode image")
            return None

        # Base64 인코딩
        image_bytes = encoded_image.tobytes()
        base64_string = base64.b64encode(image_bytes).decode('utf-8')
        return base64_string

    def send_camera_to_api(self):
        """카메라 이미지를 API 서버로 전송"""
        if self.latest_image is None:
            self.get_logger().warn("⚠️  No camera image received yet. Waiting...")
            # 이미지가 없으면 잠시 후 재시도
            self.create_timer(1.0, lambda: self.send_camera_to_api(), oneshot=True)
            return

        self.get_logger().info(f"Sending camera image to {self.api_url}")

        try:
            # ROS Image → OpenCV 변환
            cv_image = self.cv_bridge.imgmsg_to_cv2(self.latest_image, 'bgr8')

            # Base64 인코딩
            image_base64 = self.image_to_base64(cv_image)
            if image_base64 is None:
                self.get_logger().error("Failed to encode image")
                return

            # API 요청 데이터 준비
            request_data = {
                "image": image_base64,
                "timestamp": time.time(),
                "joint_positions": self.joint_positions
            }

            # HTTP POST 요청
            self.get_logger().info(f"POST {self.api_url}")
            response = requests.post(
                self.api_url,
                json=request_data,
                timeout=self.api_timeout
            )

            # 응답 처리
            if response.status_code == 200:
                self.get_logger().info(f"✅ API request successful: {response.status_code}")
                try:
                    result = response.json()
                    self.get_logger().info(f"API Response: {result}")
                except:
                    self.get_logger().info(f"API Response (text): {response.text}")
            else:
                self.get_logger().error(
                    f"❌ API request failed: HTTP {response.status_code} - {response.text}"
                )

        except requests.exceptions.Timeout:
            self.get_logger().error(f"⏱️  API request timeout after {self.api_timeout}s")
        except requests.exceptions.ConnectionError:
            self.get_logger().error(f"🔌 Connection error: Could not connect to {self.api_url}")
        except Exception as e:
            self.get_logger().error(f"💥 Unexpected error: {str(e)}")

        self.state = 'idle'
        self.get_logger().info("🎉 Sequence completed!")


def main(args=None):
    rclpy.init(args=args)
    node = SimpleControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Shutting down Simple Controller Node...")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

