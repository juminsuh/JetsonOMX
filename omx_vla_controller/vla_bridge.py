#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from audio_common_msgs.msg import AudioStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from cv_bridge import CvBridge
import cv2
import json
import base64
import requests
import threading
import time
import os
import numpy as np
import pydub
from openai import OpenAI
from dotenv import load_dotenv
from deep_translator import GoogleTranslator

# .env 파일 로드
load_dotenv()

class VLABridgeNode(Node):
    def __init__(self):
        super().__init__('vla_bridge_node')

        # 1. 파라미터 및 설정
        self.declare_parameter('api_url', 'http://100.82.52.106:8080/api/vla/infer')
        self.api_url = self.get_parameter('api_url').value
        self.api_timeout = 5.0
        
        # Whisper 및 번역기 설정
        self.whisper_key = os.getenv("WHISPER_KEY")
        self.openai_client = OpenAI(api_key=self.whisper_key)
        self.translator = GoogleTranslator(source='ko', target='en')
        self.cv_bridge = CvBridge()

        # 2. 상태 제어 변수
        self.latest_image = None
        self.image_lock = threading.Lock()
        self.is_recording = False
        self.ready_to_send = False  # 음성 인식 후 True, 그리퍼 close 시 False
        self.prompt = ""
        self.recorded_data = []
        self.request_pending = False

        # 3. ROS 구독 및 발행 설정
        self.image_sub = self.create_subscription(Image, '/image_raw', self.image_callback, 10)
        
        audio_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)
        self.audio_sub = self.create_subscription(AudioStamped, '/audio', self.audio_callback, audio_qos)

        self.command_publisher = self.create_publisher(String, '/llm_command', 10)

        # 4. 타이머 및 스레드
        # 서버 요청 주기 (VLA 모델 처리 속도에 맞춰 조절 가능)
        self.request_timer = self.create_timer(0.5, self.send_vla_request)
        
        self.input_thread = threading.Thread(target=self.wait_for_keyboard, daemon=True)
        self.input_thread.start()

        self.get_logger().info("🚀 VLA Bridge Node Online!")
        self.get_logger().info('🎹 "q" + Enter를 눌러 명령을 시작하세요.')

    def wait_for_keyboard(self):
        while rclpy.ok():
            user_input = input()
            if user_input.lower() == 'q':
                if not self.is_recording:
                    self.get_logger().info('🔴 [음성 인식 시작] 말씀해 주세요...')
                    self.ready_to_send = False 
                    self.recorded_data = []
                    self.is_recording = True
                    time.sleep(5.0)
                    self.process_voice_to_prompt()

    def audio_callback(self, msg):
        if self.is_recording:
            try:
                data = msg.audio.audio_data.int16_data
                if data: self.recorded_data.extend(data)
            except: pass

    def process_voice_to_prompt(self):
        self.is_recording = False
        self.get_logger().info('⬛ 분석 중...')
        
        if not self.recorded_data:
            self.get_logger().warn("⚠️ 데이터 없음.")
            return

        temp_file = "voice_cmd.mp3"
        try:
            audio_array = np.array(self.recorded_data, dtype=np.int16)
            pydub.AudioSegment(
                audio_array.tobytes(), frame_rate=16000, sample_width=2, channels=1
            ).export(temp_file, format="mp3")

            with open(temp_file, "rb") as f:
                transcript = self.openai_client.audio.transcriptions.create(
                    model="whisper-1", file=f, language="ko"
                )
            
            self.prompt = self.translator.translate(transcript.text)
            self.get_logger().info(f'📝 명령 확정: "{self.prompt}"')
            self.ready_to_send = True

        except Exception as e:
            self.get_logger().error(f"❌ 오류: {e}")
        finally:
            if os.path.exists(temp_file): os.remove(temp_file)

    def image_callback(self, msg):
        with self.image_lock:
            self.latest_image = msg

    def send_vla_request(self):
        """실제 서버 통신 및 응답 처리"""
        if not self.ready_to_send or self.is_recording or self.request_pending:
            return

        with self.image_lock:
            if self.latest_image is None: return
            image_msg = self.latest_image

        self.request_pending = True
        try:
            # 이미지 인코딩
            cv_img = self.cv_bridge.imgmsg_to_cv2(image_msg, 'bgr8')
            _, buffer = cv2.imencode('.jpg', cv_img, [cv2.IMWRITE_JPEG_QUALITY, 85])
            img_base64 = base64.b64encode(buffer).decode('utf-8')

            # API 전송
            payload = {"image": img_base64, "prompt": self.prompt, "unnorm_key": "bridge_orig"}
            response = requests.post(self.api_url, json=payload, timeout=self.api_timeout)

            if response.status_code == 200:
                result = response.json()
                self.get_logger().info(f"✅ VLA Result: {result}")
                gripper_state = result.get("gripper", "open")
                
                # ROS 메시지 발행
                self.publish_command(result, image_msg.header.stamp)

                # 그리퍼가 닫히면(close) 요청 중단
                if gripper_state == "close":
                    self.get_logger().info("🔒 그리퍼가 닫혔습니다. 작업을 종료하고 대기합니다.")
                    self.ready_to_send = False
                    self.get_logger().info('🎹 다시 명령하려면 "q"를 누르세요.')
            else:
                self.get_logger().error(f"서버 응답 에러: {response.status_code}")

        except Exception as e:
            self.get_logger().error(f"통신 에러: {e}")
        finally:
            self.request_pending = False

    def publish_command(self, result, timestamp):
        command_data = {
            "action": "vla_stream",
            "joint_positions": result.get("joint_positions", []),
            "gripper": result.get("gripper", "open"),
            "timestamp": timestamp.sec + timestamp.nanosec * 1e-9
        }
        msg = String()
        msg.data = json.dumps(command_data)
        self.command_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = VLABridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()