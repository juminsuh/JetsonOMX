# FastAPI 서버 Docker 컨테이너

FastAPI 서버를 독립적으로 실행하는 Docker 설정입니다.

## 구조

```
fastapi/
├── Dockerfile              # FastAPI 서버 전용 Dockerfile
├── docker-compose.yml      # FastAPI 서버 docker-compose 설정
├── api_server.py          # FastAPI 서버 메인 파일
├── requirements.txt       # Python 의존성
├── start_api_server.sh    # 서버 시작 스크립트 (선택적)
└── README.md              # 이 파일
```

## 빠른 시작

### 1. 빌드 및 실행

```bash
cd docker/fastapi
docker compose up -d --build
```

### 2. 서버 상태 확인

```bash
# 로그 확인
docker compose logs -f

# 헬스 체크
curl http://localhost:8080/health

# API 정보
curl http://localhost:8080/
```

### 3. 서버 중지

```bash
docker compose down
```

## API 엔드포인트

- `GET /` - API 정보
- `GET /health` - 헬스 체크
- `GET /stats` - 통계 정보
- `POST /api/camera` - 카메라 이미지 처리
- `POST /api/vla/infer` - VLA 추론 API (호환성)

## Swagger UI

브라우저에서 접근:
```
http://localhost:8080/docs
```

## ROS 2 컨테이너와 통신

FastAPI 서버는 별도 컨테이너로 실행되지만, `network_mode: host`를 사용하지 않는 경우 네트워크 설정이 필요합니다.

ROS 2 컨테이너에서 FastAPI 서버에 접근하려면:
- 같은 Docker 네트워크에 연결하거나
- 호스트 네트워크를 통해 `localhost:8080`으로 접근

## 포트 변경

`docker-compose.yml`에서 포트 매핑을 수정:

```yaml
ports:
  - "8080:8080"  # 호스트:컨테이너
```

예: `9000:8080`으로 변경하면 `http://localhost:9000`으로 접근

