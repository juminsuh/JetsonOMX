#!/usr/bin/env python3
"""
VLA Bridge Node - 원격 VLA API 서버와 통신하는 노드

이 노드는 Jetson Nano에서 실행되며:
1. 카메라 이미지를 구독
2. 이미지를 원격 VLA API 서버로 전송
3. 서버로부터 받은 Joint 위치를 ROS 2 메시지로 변환하여 publish

사용 예시:
    ros2 run omx_vla_controller vla_bridge
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import json
import base64
import requests
import threading
from datetime import datetime
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.qos import qos_profile_sensor_data

class VLABridgeNode(Node):
    """VLA Bridge Node - 원격 VLA API 서버와 통신"""

    def __init__(self):
        super().__init__('vla_bridge_node')

        # 파라미터 선언
        self.declare_parameter('api_url', 'http://localhost:8080/api/vla/infer')
        self.declare_parameter('api_timeout', 5.0)
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('output_topic', '/llm_command')
        self.declare_parameter('prompt', 'Move the robot arm to pick up the object')
        self.declare_parameter('request_rate', 2.0)  # 초당 요청 수 제한
        self.declare_parameter('enable_image_compression', True)
        self.declare_parameter('image_quality', 85)  # JPEG 품질 (1-100)

        # 파라미터 로드
        self.api_url = self.get_parameter('api_url').value
        self.api_timeout = self.get_parameter('api_timeout').value
        image_topic = self.get_parameter('image_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.prompt = self.get_parameter('prompt').value
        request_rate = self.get_parameter('request_rate').value
        self.enable_compression = self.get_parameter('enable_image_compression').value
        self.image_quality = self.get_parameter('image_quality').value

        # CV Bridge (ROS Image ↔ OpenCV 변환)
        self.cv_bridge = CvBridge()

        # ... __init__ 내부 ...
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, # 핵심: v4l2_camera와 맞춤
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.subscription = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            qos_profile_sensor_data
        )

        # 명령 Publisherm
        self.command_publisher = self.create_publisher(
            String,
            output_topic,
            10
        )

        # 요청 제한을 위한 타이머
        self.request_timer = self.create_timer(1.0 / request_rate, self.send_vla_request)
        self.latest_image = None
        self.image_lock = threading.Lock()
        self.request_pending = False

        # 통계
        self.request_count = 0
        self.success_count = 0
        self.error_count = 0

        self.get_logger().info("🌉 VLA Bridge Node Started!")
        self.get_logger().info(f"📡 API URL: {self.api_url}")
        self.get_logger().info(f"📸 Image topic: {image_topic}")
        self.get_logger().info(f"📤 Output topic: {output_topic}")

    def image_callback(self, msg):
        self.get_logger().info(
        f"📸 Image received: {msg.width}x{msg.height} encoding={msg.encoding}",
        throttle_duration_sec=2.0
        )
        with self.image_lock:
            self.latest_image = msg

    def image_to_base64(self, cv_image):
        """OpenCV 이미지를 Base64 문자열로 변환"""
        if self.enable_compression:
            # JPEG 압축
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self.image_quality]
            success, encoded_image = cv2.imencode('.jpg', cv_image, encode_param)
            if not success:
                self.get_logger().error("Failed to encode image")
                return None
            image_bytes = encoded_image.tobytes()
        else:
            # PNG (비압축)
            success, encoded_image = cv2.imencode('.png', cv_image)
            if not success:
                self.get_logger().error("Failed to encode image")
                return None
            image_bytes = encoded_image.tobytes()

        # Base64 인코딩
        base64_string = base64.b64encode(image_bytes).decode('utf-8')
        return base64_string

    def send_vla_request(self):
        """VLA API 서버로 요청 전송"""
        if self.request_pending:
            return

        # 최신 이미지 가져오기
        with self.image_lock:
            if self.latest_image is None:
                self.get_logger().warn("⚠️  No image received yet...", throttle_duration_sec=2.0)
                return
            image_msg = self.latest_image

        self.request_pending = True
        self.request_count += 1

        try:
            # ROS Image → OpenCV 변환
            cv_image = self.cv_bridge.imgmsg_to_cv2(image_msg, 'bgr8')

            # Base64 인코딩
            image_base64 = self.image_to_base64(cv_image)
            if image_base64 is None:
                self.request_pending = False
                return

            # API 요청 데이터 준비
            request_data = {
                "image": image_base64,
                "prompt": self.prompt,
                "unnorm_key": "libero_object"
            }

            # HTTP POST 요청
            self.get_logger().debug(f"Sending request to {self.api_url}")
            response = requests.post(
                self.api_url,
                json=request_data,
                timeout=self.api_timeout
            )

            # 응답 처리
            if response.status_code == 200:
                result = response.json()
                self.success_count += 1

                # ROS 2 메시지로 변환
                self.publish_command(result, image_msg.header.stamp)

                self.get_logger().info(
                    f"✅ Request #{self.request_count} successful "
                    f"(Success: {self.success_count}, Error: {self.error_count})"
                )
            else:
                self.error_count += 1
                self.get_logger().error(
                    f"❌ API request failed: HTTP {response.status_code} - {response.text}"
                )

        except requests.exceptions.Timeout:
            self.error_count += 1
            self.get_logger().error(
                f"⏱️  API request timeout after {self.api_timeout}s"
            )
        except requests.exceptions.ConnectionError:
            self.error_count += 1
            self.get_logger().error(
                f"🔌 Connection error: Could not connect to {self.api_url}"
            )
        except Exception as e:
            self.error_count += 1
            self.get_logger().error(f"💥 Unexpected error: {str(e)}")
        finally:
            self.request_pending = False

    def publish_command(self, api_result, timestamp):
        """API 응답을 ROS 2 메시지로 변환하여 publish"""
        try:
            # API 응답 형식에 맞게 파싱
            # 예상 형식: {"joint_positions": [0.1, 0.2, ...], "gripper": "open"}
            command_data = {
                "action": "vla_stream",
                "joint_positions": api_result.get("joint_positions", []),
                "gripper": api_result.get("gripper", "open"),
                "timestamp": timestamp.sec + timestamp.nanosec * 1e-9
            }

            # JSON 문자열로 변환
            msg = String()
            msg.data = json.dumps(command_data)

            # Publish
            self.command_publisher.publish(msg)

            self.get_logger().debug(
                f"Published command: {len(command_data['joint_positions'])} joints"
            )

        except Exception as e:
            self.get_logger().error(f"Failed to publish command: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    node = VLABridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Shutting down VLA Bridge Node...")
        node.get_logger().info(
            f"Statistics: Total={node.request_count}, "
            f"Success={node.success_count}, Error={node.error_count}"
        )
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
