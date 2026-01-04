#!/usr/bin/env python3
"""
Dummy Camera Node - 시뮬레이션용 더미 카메라
가제보나 실제 카메라가 없을 때 테스트용으로 사용

더미 이미지를 /camera/image_raw 토픽에 publish합니다.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np
from cv_bridge import CvBridge
import cv2


class DummyCameraNode(Node):
    """시뮬레이션용 더미 카메라 노드"""

    def __init__(self):
        super().__init__('dummy_camera_node')

        # 파라미터 선언
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('frame_rate', 30.0)
        self.declare_parameter('image_topic', '/camera/image_raw')

        # 파라미터 로드
        image_width = self.get_parameter('image_width').value
        image_height = self.get_parameter('image_height').value
        frame_rate = self.get_parameter('frame_rate').value
        image_topic = self.get_parameter('image_topic').value

        # Publisher
        self.publisher = self.create_publisher(
            Image,
            image_topic,
            10
        )

        # CV Bridge
        self.cv_bridge = CvBridge()
        self.image_width = image_width
        self.image_height = image_height

        # 타이머 (프레임 레이트에 맞춰 이미지 publish)
        timer_period = 1.0 / frame_rate
        self.timer = self.create_timer(timer_period, self.publish_image)
        self.frame_count = 0

        self.get_logger().info("📷 Dummy Camera Node Started!")
        self.get_logger().info(f"📸 Image topic: {image_topic}")
        self.get_logger().info(f"📐 Resolution: {image_width}x{image_height}")
        self.get_logger().info(f"🎬 Frame rate: {frame_rate} FPS")

    def publish_image(self):
        """더미 이미지 생성 및 publish"""
        self.frame_count += 1

        # 더미 이미지 생성 (검은 배경에 텍스트와 패턴)
        img = np.zeros((self.image_height, self.image_width, 3), dtype=np.uint8)

        # 그라데이션 배경 (회색)
        for y in range(self.image_height):
            gray_value = int(50 + (y / self.image_height) * 30)
            img[y, :] = [gray_value, gray_value, gray_value]

        # 텍스트 추가
        text = f'Dummy Camera Frame: {self.frame_count}'
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.7
        color = (255, 255, 255)
        thickness = 2

        # 텍스트 크기 계산
        (text_width, text_height), baseline = cv2.getTextSize(
            text, font, font_scale, thickness
        )

        # 중앙에 텍스트 배치
        x = (self.image_width - text_width) // 2
        y = (self.image_height + text_height) // 2
        cv2.putText(img, text, (x, y), font, font_scale, color, thickness)

        # 추가 정보
        info_text = f'{self.image_width}x{self.image_height} @ 30 FPS'
        (info_width, info_height), _ = cv2.getTextSize(
            info_text, font, 0.5, 1
        )
        info_x = (self.image_width - info_width) // 2
        info_y = y + text_height + 20
        cv2.putText(img, info_text, (info_x, info_y), font, 0.5, (200, 200, 200), 1)

        # 패턴 추가 (대각선 라인)
        for i in range(0, max(self.image_width, self.image_height), 50):
            cv2.line(img, (i, 0), (0, i), (100, 100, 100), 1)
            if i < self.image_width:
                cv2.line(img, (i, self.image_height), (self.image_width, self.image_height - i), 
                        (100, 100, 100), 1)

        # ROS Image 메시지로 변환
        try:
            msg = self.cv_bridge.cv2_to_imgmsg(img, 'bgr8')
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'camera_frame'

            # Publish
            self.publisher.publish(msg)

            # 주기적으로 로그 출력 (5초마다)
            if self.frame_count % (int(30 * 5)) == 0:
                self.get_logger().info(
                    f"📸 Published frame #{self.frame_count} "
                    f"({self.image_width}x{self.image_height})"
                )

        except Exception as e:
            self.get_logger().error(f"Error publishing image: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = DummyCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Shutting down Dummy Camera Node...")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

