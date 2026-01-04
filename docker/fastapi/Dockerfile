FROM python:3.12-slim

# 작업 디렉토리 설정
WORKDIR /app

# 시스템 패키지 설치 (OpenCV 의존성 및 빌드 도구)
RUN apt-get update && apt-get install -y \
    libglib2.0-0 \
    libsm6 \
    libxext6 \
    libxrender-dev \
    libgomp1 \
    curl \
    build-essential \
    && rm -rf /var/lib/apt/lists/*

# pip, setuptools, wheel 업그레이드 (최신 버전 사용)
RUN pip3 install --no-cache-dir --upgrade pip setuptools wheel

# Python 의존성 파일 복사 및 설치
COPY requirements.txt /app/requirements.txt
RUN pip3 install --no-cache-dir -r requirements.txt

# API 서버 파일 복사
COPY api_server.py /app/api_server.py
RUN chmod +x /app/api_server.py

# 포트 노출
EXPOSE 8080

# 서버 실행
CMD ["python3", "api_server.py"]

