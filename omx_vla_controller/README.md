# OMX VLA Controller

VLA (Vision-Language-Action) 컨트롤러 패키지 for OMX 로봇

## 개요

이 패키지는 **원격 VLA API 서버**와 통신하여 OMX 로봇을 제어하기 위한 ROS 2 패키지입니다.

Jetson Nano에서 실행되며, 카메라 이미지를 원격 서버로 전송하고 받은 Joint 위치 명령으로 로봇을 제어합니다.

**상세한 아키텍처 설명은 [ARCHITECTURE.md](ARCHITECTURE.md)를 참고하세요.**

## 구조

```
omx_vla_controller/
├── omx_vla_controller/
│   ├── __init__.py
│   ├── vla_dummy.py          # Dummy VLA 노드 (테스트용)
│   └── vla_bridge.py         # VLA Bridge 노드 (원격 API 연동)
├── launch/
│   ├── vla_dummy_test.launch.py  # Dummy 테스트 Launch 파일
│   └── vla_full_system.launch.py # 전체 시스템 Launch 파일
├── package.xml
├── setup.py
├── README.md
└── ARCHITECTURE.md           # 상세 아키텍처 문서
```

## Dummy Node 전략

실제 무거운 OpenVLA 모델을 올리기 전에, 가벼운 Dummy Node로 먼저 시스템의 뼈대를 검증합니다:

1. **카메라 연결 검증**: USB 카메라가 정상적으로 이미지를 전송하는지 확인
2. **통신 검증**: VLA 노드가 보낸 JSON 명령을 로봇 컨트롤러가 잘 받는지 확인
3. **제어 검증**: 로봇 팔이 delta 값에 따라 실제로 움직이는지 확인

## 사용 방법

### 1. 패키지 빌드

```bash
cd ~/ros2_ws
colcon build --packages-select omx_vla_controller
source install/setup.bash
```

### 2. Dummy 테스트 (로컬)

실제 VLA 서버 없이 파이프라인 테스트:

```bash
ros2 launch omx_vla_controller vla_dummy_test.launch.py
```

### 3. 전체 시스템 실행 (원격 VLA 서버 연동)

원격 VLA API 서버와 연동:

```bash
ros2 launch omx_vla_controller vla_full_system.launch.py \
    api_url:=http://your-server:8000/api/vla/infer \
    prompt:="Pick up the red block"
```

Launch 인자:
- `camera_name`: 카메라 노드 이름 (기본값: `camera`)
- `video_device`: 비디오 디바이스 경로 (기본값: `/dev/video0`)
- `image_width`: 이미지 너비 (기본값: `640`)
- `image_height`: 이미지 높이 (기본값: `480`)

예시:
```bash
ros2 launch omx_vla_controller vla_dummy_test.launch.py \
    video_device:=/dev/video2 \
    image_width:=320 \
    image_height:=240
```

### 4. 개별 노드 실행

카메라 노드만 실행:
```bash
ros2 launch open_manipulator_bringup camera_usb_cam.launch.py name:=camera
```

VLA Dummy 노드만 실행:
```bash
ros2 run omx_vla_controller vla_dummy
```

VLA Bridge 노드만 실행 (원격 API 연동):
```bash
ros2 run omx_vla_controller vla_bridge \
    --ros-args \
    -p api_url:=http://your-server:8000/api/vla/infer \
    -p prompt:="Pick up the object"
```

## 토픽 구조

### Subscribed Topics
- `/camera/image_raw` (sensor_msgs/Image): 카메라 이미지 입력

### Published Topics
- `/llm_command` (std_msgs/String): VLA에서 생성한 JSON 형태의 로봇 명령

### 예시 명령 형식

```json
{
  "action": "vla_stream",
  "delta": {
    "x": 0.01,
    "y": 0.0,
    "z": 0.0
  },
  "gripper": "open"
}
```

## VLA API 서버 구축

VLA Bridge 노드는 HTTP API를 통해 원격 서버와 통신합니다.

### API 엔드포인트

**POST** `/api/vla/infer`

**요청:**
```json
{
  "image": "base64_encoded_image_string",
  "prompt": "Move the robot arm to pick up the object",
  "format": "joint_positions"
}
```

**응답:**
```json
{
  "status": "success",
  "joint_positions": [0.1, 0.2, 0.3, 0.4, 0.5],
  "gripper": "open",
  "confidence": 0.95
}
```

### 서버 예시 (Python Flask)

```python
from flask import Flask, request, jsonify
import base64
import cv2
import numpy as np

app = Flask(__name__)

@app.route('/api/vla/infer', methods=['POST'])
def vla_infer():
    data = request.json
    image_b64 = data['image']
    prompt = data.get('prompt', '')
    
    # Base64 디코딩
    image_bytes = base64.b64decode(image_b64)
    nparr = np.frombuffer(image_bytes, np.uint8)
    image = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
    
    # VLA 모델 추론 (여기에 실제 모델 코드)
    # joint_positions = vla_model.predict(image, prompt)
    
    # 임시 더미 응답
    joint_positions = [0.1, 0.2, 0.3, 0.4, 0.5]
    
    return jsonify({
        "status": "success",
        "joint_positions": joint_positions,
        "gripper": "open",
        "confidence": 0.95
    })

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=8000)
```

## 의존성

- `rclpy`: ROS 2 Python 클라이언트
- `std_msgs`: 표준 메시지 타입
- `sensor_msgs`: 센서 메시지 타입 (이미지)
- `cv_bridge`: OpenCV와 ROS 메시지 간 변환

## 참고

- 기존 카메라 Launch 파일: `open_manipulator_bringup/launch/camera_usb_cam.launch.py`
- 표준 ROS 2 컨트롤러: `/arm_controller/joint_trajectory` 토픽을 구독하여 로봇 제어

