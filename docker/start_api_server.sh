#!/bin/bash
# FastAPI 서버 시작 스크립트

cd /root

# Python 경로 확인
echo "Starting FastAPI server on port 8080..."
echo "API will be available at: http://localhost:8080"
echo "Endpoints:"
echo "  - GET  /              : API 정보"
echo "  - GET  /health        : 헬스 체크"
echo "  - GET  /stats         : 통계 정보"
echo "  - POST /api/camera    : 카메라 이미지 처리"
echo "  - POST /api/vla/infer : VLA 추론 (호환성)"
echo ""
echo "Press Ctrl+C to stop the server"
echo ""

# 서버 실행
python3 api_server.py

