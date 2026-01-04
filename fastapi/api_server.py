#!/usr/bin/env python3
"""
FastAPI 서버 - 카메라 이미지 API 예시
카메라 이미지를 받아서 처리하는 간단한 API 서버

실행 방법:
    python3 api_server.py

또는 uvicorn으로:
    uvicorn api_server:app --host 0.0.0.0 --port 8080
"""

from fastapi import FastAPI, HTTPException
from fastapi.responses import JSONResponse
from pydantic import BaseModel
import base64
import cv2
import numpy as np
from datetime import datetime
import uvicorn
from typing import List, Optional
import logging

# 로깅 설정
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

app = FastAPI(
    title="OMX Camera API Server",
    description="카메라 이미지를 받아서 처리하는 API 서버",
    version="1.0.0"
)


class ImageRequest(BaseModel):
    """이미지 요청 모델"""
    image: str  # Base64 인코딩된 이미지
    timestamp: Optional[float] = None
    joint_positions: Optional[List[float]] = None


class ImageResponse(BaseModel):
    """이미지 응답 모델"""
    status: str
    message: str
    image_info: dict
    processed_at: str


# 통계
request_count = 0
success_count = 0


def decode_base64_image(image_base64: str) -> np.ndarray:
    """Base64 문자열을 OpenCV 이미지로 변환"""
    try:
        # Base64 디코딩
        image_bytes = base64.b64decode(image_base64)
        
        # NumPy 배열로 변환
        nparr = np.frombuffer(image_bytes, np.uint8)
        
        # OpenCV 이미지로 디코딩
        image = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
        
        if image is None:
            raise ValueError("Failed to decode image")
        
        return image
    except Exception as e:
        logger.error(f"Error decoding image: {e}")
        raise


def process_image(image: np.ndarray) -> dict:
    """이미지 처리 (예시: 간단한 정보 추출)"""
    height, width = image.shape[:2]
    
    # 이미지 통계
    mean_bgr = image.mean(axis=(0, 1))
    
    # 그레이스케일 변환
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    mean_intensity = gray.mean()
    
    return {
        "width": int(width),
        "height": int(height),
        "channels": int(image.shape[2]) if len(image.shape) > 2 else 1,
        "mean_bgr": [float(x) for x in mean_bgr],
        "mean_intensity": float(mean_intensity),
        "shape": list(image.shape)
    }


@app.get("/")
async def root():
    """루트 엔드포인트"""
    return {
        "message": "OMX Camera API Server",
        "version": "1.0.0",
        "endpoints": {
            "/api/camera": "POST - 카메라 이미지 처리",
            "/health": "GET - 서버 상태 확인",
            "/stats": "GET - 통계 정보"
        }
    }


@app.get("/health")
async def health_check():
    """헬스 체크"""
    return {
        "status": "healthy",
        "timestamp": datetime.now().isoformat()
    }


@app.get("/stats")
async def get_stats():
    """통계 정보"""
    global request_count, success_count
    return {
        "total_requests": request_count,
        "successful_requests": success_count,
        "failed_requests": request_count - success_count,
        "success_rate": success_count / request_count if request_count > 0 else 0.0
    }


@app.post("/api/camera", response_model=ImageResponse)
async def process_camera_image(request: ImageRequest):
    """
    카메라 이미지 처리 API
    
    요청:
    {
        "image": "base64_encoded_image_string",
        "timestamp": 1234567890.123,
        "joint_positions": [0.0, -1.57, 1.57, 1.57, 0.0]
    }
    
    응답:
    {
        "status": "success",
        "message": "Image processed successfully",
        "image_info": {
            "width": 640,
            "height": 480,
            ...
        },
        "processed_at": "2024-01-01T12:00:00"
    }
    """
    global request_count, success_count
    
    request_count += 1
    
    try:
        logger.info(f"Received image request #{request_count}")
        
        # Base64 디코딩
        image = decode_base64_image(request.image)
        
        # 이미지 처리
        image_info = process_image(image)
        
        # Joint 위치 정보 로깅 (있는 경우)
        if request.joint_positions:
            logger.info(f"Joint positions: {request.joint_positions}")
        
        # 타임스탬프 로깅 (있는 경우)
        if request.timestamp:
            logger.info(f"Request timestamp: {request.timestamp}")
        
        success_count += 1
        
        response = ImageResponse(
            status="success",
            message="Image processed successfully",
            image_info=image_info,
            processed_at=datetime.now().isoformat()
        )
        
        logger.info(f"✅ Request #{request_count} processed successfully")
        logger.info(f"Image info: {image_info['width']}x{image_info['height']}")
        
        return response
        
    except ValueError as e:
        logger.error(f"❌ Validation error: {e}")
        raise HTTPException(status_code=400, detail=f"Invalid image data: {str(e)}")
    except Exception as e:
        logger.error(f"❌ Processing error: {e}")
        raise HTTPException(status_code=500, detail=f"Image processing failed: {str(e)}")


@app.post("/api/vla/infer")
async def vla_infer(request: ImageRequest):
    """
    VLA 추론 API (호환성을 위한 엔드포인트)
    
    요청:
    {
        "image": "base64_encoded_image_string",
        "prompt": "Move the robot arm to pick up the object",
        "format": "joint_positions"
    }
    
    응답:
    {
        "status": "success",
        "joint_positions": [0.1, 0.2, 0.3, 0.4, 0.5],
        "gripper": "open",
        "confidence": 0.95
    }
    """
    global request_count, success_count
    
    request_count += 1
    
    try:
        logger.info(f"Received VLA inference request #{request_count}")
        
        # Base64 디코딩
        image = decode_base64_image(request.image)
        
        # 이미지 처리
        image_info = process_image(image)
        
        # 임시 더미 응답 (실제 VLA 모델이 있으면 여기서 추론)
        # 실제로는 VLA 모델을 사용하여 joint_positions를 계산해야 합니다
        dummy_joint_positions = [0.1, 0.2, 0.3, 0.4, 0.5]
        
        success_count += 1
        
        response = {
            "status": "success",
            "joint_positions": dummy_joint_positions,
            "gripper": "open",
            "confidence": 0.95,
            "image_info": image_info,
            "processed_at": datetime.now().isoformat()
        }
        
        logger.info(f"✅ VLA inference #{request_count} completed")
        logger.info(f"Predicted joint positions: {dummy_joint_positions}")
        
        return response
        
    except ValueError as e:
        logger.error(f"❌ Validation error: {e}")
        raise HTTPException(status_code=400, detail=f"Invalid image data: {str(e)}")
    except Exception as e:
        logger.error(f"❌ Processing error: {e}")
        raise HTTPException(status_code=500, detail=f"VLA inference failed: {str(e)}")


if __name__ == "__main__":
    # 서버 실행
    uvicorn.run(
        app,
        host="0.0.0.0",
        port=8080,
        log_level="info"
    )

