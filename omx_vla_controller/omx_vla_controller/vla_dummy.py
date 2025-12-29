#!/usr/bin/env python3
"""
VLA Dummy Node - 가짜 두뇌 (Dummy Brain) 전략

이 노드는 실제 OpenVLA 모델 대신 사용되는 테스트용 더미 노드입니다.
카메라 이미지를 구독하여 연결을 확인하고, 주기적으로 가짜 명령을 생성하여
로봇 제어 파이프라인의 나머지 부분이 정상 작동하는지 검증합니다.

나중에 실제 OpenVLA 추론 코드로 교체될 예정입니다.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import json
import random


class VLADummyNode(Node):
    """VLA Dummy Node - 테스트용 더미 추론 노드"""

    def __init__(self):
        super().__init__('vla_dummy_node')

        # 1. 카메라 이미지 구독 (연결 테스트용)
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',  # usb_cam이 쏘는 토픽
            self.image_callback,
            10
        )
        self.cv_bridge = CvBridge()
        self.image_received = False

        # 2. 로봇에게 명령을 내리는 Publisher
        # /llm_command 토픽에 JSON 형태의 명령을 publish
        self.command_publisher = self.create_publisher(
            String,
            '/llm_command',
            10
        )

        # 3. 주기적으로 가짜 명령 생성 (2초마다)
        self.timer = self.create_timer(2.0, self.dummy_inference_loop)

        # 카운터 (명령에 ID 추가)
        self.command_counter = 0

        self.get_logger().info("🤖 VLA Dummy Node Started! (Waiting for Camera...)")
        self.get_logger().info("📡 Publishing dummy commands to /llm_command topic")

    def image_callback(self, msg):
        """카메라 이미지가 들어오면 로그만 찍음 (연결 확인용)"""
        if not self.image_received:
            self.image_received = True
            self.get_logger().info(
                f"✅ Camera connected! Frame size: {msg.width}x{msg.height}"
            )

        # 주기적으로 로그 출력 (2초마다)
        self.get_logger().info(
            f"📸 Frame Received: {msg.width}x{msg.height}",
            throttle_duration_sec=2.0
        )

    def dummy_inference_loop(self):
        """가짜 추론 로직 - 주기적으로 더미 명령 생성 및 전송"""
        # 카메라가 연결되지 않았으면 명령 전송하지 않음
        if not self.image_received:
            self.get_logger().warn("⚠️  Waiting for camera images...")
            return

        # --- [가짜 추론 로직] ---
        # 실제 모델 대신 랜덤하거나 고정된 값을 내보냄
        # 예시: X축으로 조금씩 왔다갔다 하게 만들기

        # 랜덤 delta 값 생성 (-0.02 ~ 0.02m 사이)
        delta_x = random.uniform(-0.02, 0.02)
        delta_y = random.uniform(-0.01, 0.01)
        delta_z = random.uniform(-0.01, 0.01)

        # 가짜 VLA action 생성
        # vla_stream action은 delta 값으로 로봇을 미세하게 이동시킴
        dummy_action = {
            "action": "vla_stream",
            "delta": {
                "x": delta_x,  # -0.02 ~ 0.02m 사이 랜덤 이동
                "y": delta_y,
                "z": delta_z
            },
            "gripper": "open"  # 또는 "close"
        }

        # JSON 문자열로 변환
        msg = String()
        msg.data = json.dumps(dummy_action)

        # 명령 전송
        self.command_publisher.publish(msg)
        self.command_counter += 1

        self.get_logger().info(
            f"📤 [{self.command_counter}] Sent Dummy Command: "
            f"dx={delta_x:.4f}, dy={delta_y:.4f}, dz={delta_z:.4f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = VLADummyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

