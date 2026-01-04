# FastAPI 서버 사용 가이드

Docker 컨테이너에 포함된 FastAPI 서버 예시입니다.

## 빠른 시작

### 1. Docker 컨테이너 접속

```bash
docker exec -it open_manipulator bash
```

### 2. API 서버 실행

```bash
# 방법 1: 스크립트 사용
/root/start_api_server.sh

# 방법 2: 직접 실행
python3 /root/api_server.py

# 방법 3: uvicorn 사용
uvicorn api_server:app --host 0.0.0.0 --port 8080 --app-dir /root
```

서버가 실행되면 `http://localhost:8080`에서 접근할 수 있습니다.

## API 엔드포인트

### GET `/`
API 정보 및 엔드포인트 목록

### GET `/health`
서버 상태 확인

### GET `/stats`
통계 정보 (요청 수, 성공률 등)

### POST `/api/camera`
카메라 이미지 처리

**요청 예시:**
```json
{
  "image": "base64_encoded_image_string",
  "timestamp": 1234567890.123,
  "joint_positions": [0.0, -1.57, 1.57, 1.57, 0.0]
}
```

**응답 예시:**
```json
{
  "status": "success",
  "message": "Image processed successfully",
  "image_info": {
    "width": 640,
    "height": 480,
    "channels": 3,
    "mean_bgr": [100.5, 120.3, 110.2],
    "mean_intensity": 110.3,
    "shape": [480, 640, 3]
  },
  "processed_at": "2024-01-01T12:00:00.123456"
}
```

### POST `/api/vla/infer`
VLA 추론 API (호환성을 위한 엔드포인트)

**요청 예시:**
```json
{
  "image": "base64_encoded_image_string",
  "prompt": "Move the robot arm to pick up the object",
  "format": "joint_positions"
}
```

**응답 예시:**
```json
{
  "status": "success",
  "joint_positions": [0.1, 0.2, 0.3, 0.4, 0.5],
  "gripper": "open",
  "confidence": 0.95,
  "image_info": {...},
  "processed_at": "2024-01-01T12:00:00.123456"
}
```

## Simple Controller와 함께 사용

1. **API 서버 실행** (터미널 1)
```bash
docker exec -it open_manipulator bash
python3 /root/api_server.py
```

2. **Simple Controller 실행** (터미널 2)
```bash
docker exec -it open_manipulator bash
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash

ros2 run omx_vla_controller simple_controller \
    --ros-args \
    -p joint_positions:="[0.0, -1.57, 1.57, 1.57, 0.0]" \
    -p api_url:=http://localhost:8080/api/camera
```

## 테스트

### curl로 테스트

```bash
# 헬스 체크
curl http://localhost:8080/health

# 통계 확인
curl http://localhost:8080/stats

# 이미지 전송 (Python 스크립트 필요)
python3 -c "
import requests
import base64
import cv2
import numpy as np

# 테스트 이미지 생성
img = np.zeros((480, 640, 3), dtype=np.uint8)
cv2.rectangle(img, (100, 100), (540, 380), (0, 255, 0), 2)
cv2.putText(img, 'Test Image', (200, 250), cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 255, 255), 3)

# Base64 인코딩
_, buffer = cv2.imencode('.jpg', img)
img_base64 = base64.b64encode(buffer).decode('utf-8')

# API 요청
response = requests.post(
    'http://localhost:8080/api/camera',
    json={
        'image': img_base64,
        'joint_positions': [0.0, -1.57, 1.57, 1.57, 0.0]
    }
)

print(response.json())
"
```

## Swagger UI

서버 실행 후 브라우저에서 다음 주소로 접근하면 Swagger UI를 사용할 수 있습니다:

```
http://localhost:8080/docs
```

## 주의사항

- 서버는 컨테이너 내부에서 `localhost:8080`으로 실행됩니다
- `network_mode: host`를 사용하므로 호스트에서도 `localhost:8080`으로 접근 가능합니다
- 실제 VLA 추론을 사용하려면 `/api/vla/infer` 엔드포인트를 수정하여 실제 모델을 로드해야 합니다

