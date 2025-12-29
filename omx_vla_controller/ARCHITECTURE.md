# OMX VLA Controller 아키텍처 문서

## 전체 시스템 구조

이 프로젝트는 **분산 아키텍처**를 사용하여 무거운 VLA 추론을 원격 서버에서 수행하고, Jetson Nano에서 실시간 로봇 제어를 담당합니다.

```
┌─────────────────────────────────────────────────────────────────────┐
│                        원격 서버 (VLA API)                           │
│  ┌──────────────────────────────────────────────────────────────┐  │
│  │  VLA 모델 서버                                                │  │
│  │  - 입력: 이미지 + 텍스트 (프롬프트)                            │  │
│  │  - 출력: Joint 위치 값 (JSON)                                 │  │
│  │  - 예시: HTTP/gRPC API 서버                                   │  │
│  └──────────────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────────┘
                              ↕ HTTP/API 통신
┌─────────────────────────────────────────────────────────────────────┐
│                    Jetson Nano (로봇 제어 노드)                      │
│  ┌──────────────┐    ┌──────────────┐    ┌──────────────┐         │
│  │   카메라     │    │  VLA Bridge  │    │  로봇 제어   │         │
│  │   노드       │───▶│    노드      │───▶│    노드      │         │
│  │ (usb_cam)    │    │              │    │              │         │
│  └──────────────┘    └──────────────┘    └──────────────┘         │
│        │                    │                    │                  │
│        │                    │                    │                  │
│        └────────────────────┴────────────────────┘                  │
│                           ROS 2 DDS                                │
└─────────────────────────────────────────────────────────────────────┘
                              ↕ 하드웨어 인터페이스
┌─────────────────────────────────────────────────────────────────────┐
│                      OMX 로봇 하드웨어                               │
│  - 카메라 (USB 웹캠)                                                │
│  - 로봇 팔 (Dynamixel 모터)                                         │
│  - 그리퍼                                                           │
└─────────────────────────────────────────────────────────────────────┘
```

## ROS 2 구조 설명

### ROS 2의 기본 개념

ROS 2는 **분산 로봇 프로그래밍 프레임워크**로, 다음과 같은 핵심 개념을 가집니다:

#### 1. 노드 (Node)
- **정의**: 실행 가능한 최소 단위 프로그램
- **역할**: 특정 작업을 수행 (예: 카메라에서 이미지 읽기, 모터 제어)
- **예시**: `camera_node`, `vla_bridge_node`, `robot_controller_node`

#### 2. 토픽 (Topic)
- **정의**: 노드 간 비동기 메시지 통신 채널
- **특징**: Publisher-Subscriber 패턴, Many-to-Many 통신
- **예시**:
  - `/camera/image_raw`: 카메라 이미지 (sensor_msgs/Image)
  - `/llm_command`: VLA 명령 (std_msgs/String)
  - `/arm_controller/joint_trajectory`: 관절 궤적 (trajectory_msgs/JointTrajectory)

#### 3. 메시지 (Message)
- **정의**: 토픽을 통해 전송되는 데이터 구조
- **타입**: 표준 메시지 타입 (sensor_msgs, std_msgs, trajectory_msgs 등)
- **예시**: `Image`, `String`, `JointTrajectory`

#### 4. 서비스 (Service)
- **정의**: Request-Response 패턴의 동기 통신
- **사용**: 한 번의 요청에 대한 응답이 필요한 작업
- **예시**: Inverse Kinematics 계산 서비스

#### 5. 액션 (Action)
- **정의**: 장기 실행 작업을 위한 통신 방식 (목표 설정 → 피드백 → 결과)
- **사용**: 로봇 팔 이동, 그리퍼 제어 등 시간이 걸리는 작업
- **예시**: `/gripper_controller/gripper_cmd` (GripperCommand action)

### ROS 2 패키지 구조

ROS 2 패키지는 관련된 노드, Launch 파일, 설정 파일들을 그룹화하는 단위입니다.

```
omx_vla_controller/                    # 패키지 루트
├── package.xml                        # 패키지 메타데이터 (의존성, 설명 등)
├── setup.py                           # Python 패키지 설정 (entry_points 포함)
├── setup.cfg                          # 추가 설정 파일
├── omx_vla_controller/                # Python 패키지 디렉토리
│   ├── __init__.py                    # Python 패키지 초기화
│   ├── vla_dummy.py                   # VLA Dummy Node (테스트용)
│   └── vla_bridge.py                  # VLA Bridge Node (원격 API 연동)
├── launch/                            # Launch 파일 디렉토리
│   ├── vla_dummy_test.launch.py       # Dummy 테스트 Launch 파일
│   └── vla_full_system.launch.py      # 전체 시스템 Launch 파일
├── config/                            # 설정 파일 디렉토리 (선택적)
│   └── vla_config.yaml                # VLA 관련 설정
└── README.md                          # 패키지 설명서
```

### 패키지 주요 파일 설명

#### `package.xml`
- 패키지 이름, 버전, 의존성 정의
- 빌드 시스템이 패키지를 인식하고 의존성을 해결하는 데 사용

#### `setup.py`
- Python 패키지 빌드 설정
- `entry_points`: 실행 가능한 노드를 등록
  ```python
  entry_points={
      'console_scripts': [
          'vla_dummy = omx_vla_controller.vla_dummy:main',
      ],
  }
  ```

#### Launch 파일 (`.launch.py`)
- 여러 노드를 한 번에 실행하는 스크립트
- 파라미터 설정, 토픽 리맵핑 등을 포함
- 실행: `ros2 launch <패키지명> <launch파일명>`

## 전체 파이프라인 상세

### 1단계: 이미지 캡처 (Jetson Nano)

```
┌─────────────┐
│ USB 카메라  │───▶ /camera/image_raw (sensor_msgs/Image)
└─────────────┘
       │
       ▼
┌─────────────────────────┐
│ usb_cam 노드            │
│ - 하드웨어에서 이미지   │
│   읽기                  │
│ - ROS 2 메시지로 변환   │
└─────────────────────────┘
```

**노드**: `usb_cam_node_exe` (usb_cam 패키지)
**토픽**: `/camera/image_raw`
**메시지 타입**: `sensor_msgs/Image`

### 2단계: VLA Bridge (Jetson Nano)

```
┌─────────────────────────┐
│ VLA Bridge Node         │
│                         │
│ 1. 이미지 구독          │
│    (/camera/image_raw)  │
│                         │
│ 2. 이미지 인코딩        │
│    (Base64 또는 바이너리)│
│                         │
│ 3. HTTP 요청 생성       │
│    POST /api/vla/infer  │
│    {                    │
│      "image": "...",    │
│      "prompt": "..."    │
│    }                    │
│                         │
│ 4. 응답 수신            │
│    {                    │
│      "joint_positions": │
│        [0.1, 0.2, ...]  │
│    }                    │
│                         │
│ 5. ROS 2 메시지로 변환  │
│    (/llm_command)       │
└─────────────────────────┘
```

**노드**: `vla_bridge` (omx_vla_controller 패키지)
**구독 토픽**: `/camera/image_raw`
**발행 토픽**: `/llm_command` (std_msgs/String, JSON 형식)
**외부 통신**: HTTP/HTTPS API (원격 VLA 서버)

### 3단계: 원격 VLA 추론 서버

```
┌─────────────────────────┐
│ VLA API 서버            │
│                         │
│ 엔드포인트:             │
│ POST /api/vla/infer     │
│                         │
│ 입력:                   │
│ {                       │
│   "image": "base64...", │
│   "prompt": "..."       │
│ }                       │
│                         │
│ 처리:                   │
│ - 이미지 디코딩         │
│ - VLA 모델 추론         │
│ - Joint 위치 계산       │
│                         │
│ 출력:                   │
│ {                       │
│   "joint_positions": [  │
│     0.1, 0.2, 0.3, ...  │
│   ],                    │
│   "gripper": "open"     │
│ }                       │
└─────────────────────────┘
```

**서버**: Python Flask/FastAPI 또는 gRPC 서버
**모델**: OpenVLA 또는 유사한 Vision-Language-Action 모델
**응답 시간**: 일반적으로 0.5~2초 (네트워크 + 추론 시간)

### 4단계: 로봇 제어 노드 (Jetson Nano)

```
┌─────────────────────────┐
│ Robot Controller Node   │
│                         │
│ 1. VLA 명령 구독        │
│    (/llm_command)       │
│                         │
│ 2. JSON 파싱            │
│    {                    │
│      "joint_positions": │
│        [0.1, 0.2, ...]  │
│    }                    │
│                         │
│ 3. Joint Trajectory 생성│
│    (trajectory_msgs)    │
│                         │
│ 4. 로봇 컨트롤러에 전송 │
│    (/arm_controller/    │
│     joint_trajectory)   │
└─────────────────────────┘
```

**노드**: `robot_controller` (omx_vla_controller 또는 별도 패키지)
**구독 토픽**: `/llm_command`
**발행 토픽**: `/arm_controller/joint_trajectory`
**메시지 타입**: `trajectory_msgs/JointTrajectory`

### 5단계: ROS 2 Control (Jetson Nano)

```
┌─────────────────────────┐
│ Joint Trajectory        │
│ Controller              │
│                         │
│ - /arm_controller/      │
│   joint_trajectory 구독 │
│                         │
│ - 궤적 계획 및 실행     │
│                         │
│ - 하드웨어 인터페이스   │
│   호출                  │
└─────────────────────────┘
       │
       ▼
┌─────────────────────────┐
│ Dynamixel Hardware      │
│ Interface               │
│                         │
│ - Dynamixel 모터 제어   │
│ - 시리얼 통신           │
│   (/dev/ttyACM0)        │
└─────────────────────────┘
       │
       ▼
┌─────────────────────────┐
│ OMX 로봇 팔             │
│ - Joint 1~5             │
│ - Gripper               │
└─────────────────────────┘
```

**컨트롤러**: `joint_trajectory_controller/JointTrajectoryController` (표준 ROS 2 컨트롤러)
**하드웨어**: `dynamixel_hardware_interface/DynamixelHardware`
**통신**: 시리얼 포트 (`/dev/ttyACM0`)

## 데이터 흐름 다이어그램

```
시간축 →
┌─────────────────────────────────────────────────────────────────────┐
│ 1. 이미지 캡처 (30 FPS)                                             │
│    카메라 → usb_cam → /camera/image_raw                             │
│    ⏱️ 33ms                                                           │
├─────────────────────────────────────────────────────────────────────┤
│ 2. VLA Bridge: 이미지 → API 요청                                    │
│    /camera/image_raw → HTTP POST → 원격 서버                        │
│    ⏱️ 네트워크 지연 (~10-100ms)                                     │
├─────────────────────────────────────────────────────────────────────┤
│ 3. VLA 추론 (원격 서버)                                             │
│    이미지 + 프롬프트 → VLA 모델 → Joint 위치                        │
│    ⏱️ 추론 시간 (~500-2000ms)                                       │
├─────────────────────────────────────────────────────────────────────┤
│ 4. VLA Bridge: API 응답 → ROS 메시지                                │
│    HTTP Response → JSON 파싱 → /llm_command publish                 │
│    ⏱️ 네트워크 지연 (~10-100ms)                                     │
├─────────────────────────────────────────────────────────────────────┤
│ 5. 로봇 제어: Joint Trajectory 생성                                 │
│    /llm_command → 파싱 → /arm_controller/joint_trajectory           │
│    ⏱️ 처리 시간 (~1-5ms)                                            │
├─────────────────────────────────────────────────────────────────────┤
│ 6. 로봇 실행: 하드웨어 제어                                         │
│    /arm_controller/joint_trajectory → Dynamixel → 로봇 팔 이동      │
│    ⏱️ 실행 시간 (명령에 따라 다름, ~1-5초)                          │
└─────────────────────────────────────────────────────────────────────┘

총 지연 시간: ~600-2200ms (추론 시간에 따라 크게 좌우)
```

## 통신 프로토콜 상세

### VLA API 통신 프로토콜 (HTTP)

#### 요청 (Request)
```http
POST /api/vla/infer HTTP/1.1
Host: vla-server.example.com
Content-Type: application/json

{
  "image": "base64_encoded_image_string",
  "prompt": "Pick up the red block",
  "format": "joint_positions"  // 또는 "delta", "waypoint" 등
}
```

#### 응답 (Response)
```http
HTTP/1.1 200 OK
Content-Type: application/json

{
  "status": "success",
  "joint_positions": [0.1, 0.2, 0.3, 0.4, 0.5],
  "gripper": "open",
  "confidence": 0.95,
  "timestamp": "2024-01-01T12:00:00Z"
}
```

### ROS 2 메시지 형식

#### `/llm_command` 토픽 (std_msgs/String)
```json
{
  "action": "vla_stream",
  "joint_positions": [0.1, 0.2, 0.3, 0.4, 0.5],
  "gripper": "open",
  "timestamp": "1234567890.123456789"
}
```

#### `/arm_controller/joint_trajectory` 토픽 (trajectory_msgs/JointTrajectory)
```yaml
header:
  stamp:
    sec: 1234567890
    nanosec: 123456789
  frame_id: "world"
joint_names:
  - "joint1"
  - "joint2"
  - "joint3"
  - "joint4"
  - "joint5"
points:
  - positions: [0.1, 0.2, 0.3, 0.4, 0.5]
    velocities: []
    accelerations: []
    time_from_start:
      sec: 2
      nanosec: 0
```

## 시스템 구성 요소 요약

| 구성 요소 | 위치 | 역할 | 통신 방식 |
|---------|------|------|----------|
| USB 카메라 | Jetson Nano | 이미지 캡처 | USB |
| usb_cam 노드 | Jetson Nano | 카메라 드라이버 | ROS 2 토픽 |
| VLA Bridge 노드 | Jetson Nano | API 클라이언트 | HTTP + ROS 2 |
| VLA API 서버 | 원격 서버 | 모델 추론 | HTTP |
| 로봇 제어 노드 | Jetson Nano | 명령 변환 | ROS 2 토픽 |
| Joint Trajectory Controller | Jetson Nano | 궤적 실행 | ROS 2 토픽 |
| Dynamixel Hardware | Jetson Nano | 모터 제어 | 시리얼 |
| OMX 로봇 팔 | 물리적 하드웨어 | 실제 동작 | 전기 신호 |

## 장점과 고려사항

### 장점
1. **계산 리소스 분산**: 무거운 VLA 추론을 원격 서버에서 처리하여 Jetson Nano 부담 감소
2. **확장성**: 여러 로봇이 동일한 VLA 서버를 공유 가능
3. **모델 업데이트 용이**: 서버에서만 모델을 업데이트하면 모든 클라이언트에 반영
4. **디버깅 편의**: 서버 로그를 중앙 집중 관리

### 고려사항
1. **네트워크 지연**: 인터넷 연결이 불안정하면 로봇 응답 지연
2. **오프라인 동작 불가**: 네트워크가 끊기면 로봇 제어 불가
3. **보안**: 이미지 데이터 전송 시 프라이버시 고려 필요
4. **서버 부하**: 여러 로봇이 동시에 요청 시 서버 부하 증가

## 향후 개선 방향

1. **로컬 폴백**: 네트워크 오류 시 로컬 경량 모델 사용
2. **스트리밍 최적화**: 이미지 압축, 프레임 스킵 등으로 대역폭 절약
3. **캐싱**: 유사한 이미지에 대한 추론 결과 캐싱
4. **gRPC 사용**: HTTP 대신 gRPC로 지연 시간 단축
5. **에지 추론**: Jetson Nano에서 직접 경량 모델 실행 (모델 최적화 후)

