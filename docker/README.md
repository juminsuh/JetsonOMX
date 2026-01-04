# Docker 설정 - ROS 2 환경

ROS 2 환경을 위한 Docker 컨테이너입니다.

## 구조

```
docker/
├── Dockerfile              # ROS 2 전용 Dockerfile
├── docker-compose.yml      # ROS 2 컨테이너 docker-compose 설정
├── container.sh            # 컨테이너 관리 스크립트
├── README.md               # 이 파일
└── fastapi/                # FastAPI 서버 (별도 디렉토리)
    ├── Dockerfile
    ├── docker-compose.yml
    └── ...
```

## 빠른 시작

### 1. 이미지 빌드

```bash
cd docker
docker compose build
```

### 2. 컨테이너 시작

```bash
docker compose up -d
```

또는 스크립트 사용:
```bash
./container.sh start
```

### 3. 컨테이너 접속

```bash
docker exec -it open_manipulator bash
```

또는 스크립트 사용:
```bash
./container.sh enter
```

### 4. 컨테이너 중지

```bash
docker compose down
```

또는 스크립트 사용:
```bash
./container.sh stop
```

## FastAPI 서버 실행

FastAPI 서버는 별도 디렉토리에 있습니다. 자세한 내용은 `fastapi/README.md`를 참고하세요.

```bash
cd docker/fastapi
docker compose up -d
```

## 주요 설정

- **네트워크**: `host` 모드 (호스트 네트워크 공유)
- **볼륨**: ROS 2 워크스페이스, 디바이스, X11
- **권한**: `privileged` 모드 (하드웨어 접근용)

## 컨테이너 스크립트 사용법

```bash
./container.sh start    # 컨테이너 시작
./container.sh enter    # 컨테이너 접속
./container.sh stop     # 컨테이너 중지
./container.sh help     # 도움말
```

