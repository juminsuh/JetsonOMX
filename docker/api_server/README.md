# OMX API Server

OMX 로봇 제어를 위한 FastAPI 서버

## 빌드 및 실행

### Docker Compose 사용 (권장)

```bash
cd docker
docker compose up -d api_server
```

### 직접 Docker 빌드

```bash
cd docker/api_server
docker build -t omx-api-server:latest .
docker run -d --name api_server --network host omx-api-server:latest
```

## API 엔드포인트

- `GET /` - API 정보
- `GET /health` - 헬스 체크
- `GET /stats` - 통계 정보
- `POST /api/camera` - 카메라 이미지 처리
- `POST /api/vla/infer` - VLA 추론 API

## 개발 모드

```bash
cd docker/api_server
pip install -r requirements.txt
python api_server.py
```

## 포트

기본 포트: `8080`

## 환경 변수

- `PYTHONUNBUFFERED=1` - Python 출력 버퍼링 비활성화

