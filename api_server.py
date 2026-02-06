#!/usr/bin/env python3
"""
FastAPI 서버 - OpenVLA로부터 action을 출력하는 API 서버

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
import tensorflow as tf
from PIL import Image
from transformers import AutoModelForVision2Seq, AutoProcessor
import torch
import io

# tf와 pytorch의 충돌을 막음
import os
os.environ["CUDA_VISIBLE_DEVICES"] = "0" # 사용할 GPU 번호
tf.config.set_visible_devices([], 'GPU') # TF는 GPU를 잡지 않도록 설정

# 로깅 설정
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# TF32 가속 (Ampere 이상에서 matmul 속도 향상)
torch.backends.cuda.matmul.allow_tf32 = True
torch.backends.cudnn.allow_tf32 = True
try:
    torch.set_float32_matmul_precision("high")
except Exception:
    pass

# openvla 설정 및 로드
MODEL_ID = "openvla/openvla-7b-finetuned-libero-object"
DEVICE = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")

logger.info(f"Loading OpenVLA model to {DEVICE}...")
processor = AutoProcessor.from_pretrained(MODEL_ID, trust_remote_code=True)
def _load_vla_model(attn_impl: str):
    return AutoModelForVision2Seq.from_pretrained(
        MODEL_ID,
        attn_implementation=attn_impl,
        torch_dtype=torch.float16,
        low_cpu_mem_usage=True,
        trust_remote_code=True
    )

try:
    vla = _load_vla_model("flash_attention_2").to(DEVICE)
    logger.info("Using attention implementation: flash_attention_2")
except Exception as e:
    logger.warning(f"flash_attention_2 load failed ({e}); falling back to sdpa")
    vla = _load_vla_model("sdpa").to(DEVICE)
    logger.info("Using attention implementation: sdpa")
logger.info("🥳 OpenVLA model loaded successfully!")

    
class VLARequest(BaseModel):
    """OpenVLA 요청 모델"""
    image: str # Base64 인코딩된 이미지
    prompt: str
    unnorm_key: str = "libero_object"

# fastapi
app = FastAPI(
    title="OMX OpenVLA API Server",
    description="OpenVLA로부터 action을 출력하는 API 서버",
    version="1.0.0"
)

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
        image = cv2.imdecode(nparr, cv2.IMREAD_COLOR) # .jpeg -> numpy array
        
        if image is None:
            raise ValueError("Failed to decode image")
        
        return image
    except Exception as e:
        logger.error(f"Error decoding image: {e}")
        raise
    
def process_vla_image(image: np.ndarray, resize_size=(224, 224)) -> dict:
    """이미지 처리"""
    if isinstance(image, np.ndarray):
        img = tf.convert_to_tensor(image, dtype=tf.uint8)
    
    # RLDS 데이터셋 시뮬레이션: 훈련 때 봤던 이미지를 모사
    img = tf.image.encode_jpeg(img)
    img = tf.io.decode_image(img, expand_animations=False, dtype=tf.uint8)
    
    # Lanczos3 리사이징
    img = tf.image.resize(img, resize_size, method="lanczos3", antialias=True)
    
    # 후처리 및 numpy 반환
    img = tf.cast(tf.clip_by_value(tf.round(img), 0, 255), tf.uint8)
    np_img = img.numpy()
    
    # PIL 이미지로 변환
    pil_image = Image.fromarray(np_img)
    pil_image = pil_image.convert("RGB")
    
    return pil_image
    

@app.get("/")
async def root():
    """루트 엔드포인트"""
    return {
        "message": "OMX OpenVLA API Server",
        "version": "1.0.0",
        "endpoints": {
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

@app.post("/api/vla/infer")
async def vla_infer(request: VLARequest):
    """
    VLA 추론 API (호환성을 위한 엔드포인트)
    
    요청:
    {
        "image": "base64_encoded_image_string",
        "prompt": "Move the robot arm to pick up the object",
        "unnorm_key": "bridge_orig"
    }
    
    응답:
    {
        "status": "success",
        "action": [0.1, 0.2, 0.3, 0.4, 0.5, 0.2]
        "joint_positions": [0.1, 0.2, 0.3, 0.4, 0.5],
        "gripper": "open",
        "processed_at": 2024-05-22T14:30:05.123456
    }
    """
    global request_count, success_count
    
    request_count += 1
    
    try:
        logger.info(f"Received VLA inference request #{request_count}")
        
        # Base64 디코딩
        image = decode_base64_image(request.image) # base64 -> .jpeg -> numpy array
        
        # bgr -> rgb 변환
        rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        
        # openvla 이미지로 전처리
        pil_image = process_vla_image(rgb_image, resize_size=(224, 224)) # PIL Image
        
        prompt = f"In: {request.prompt}\nOut:"
        inputs = processor(prompt, pil_image).to(DEVICE, dtype=torch.float16)
        
        with torch.inference_mode():
            action = vla.predict_action(**inputs, unnorm_key=request.unnorm_key, do_sample=False)
        logger.info(f"➡️ Predicted action: {action.tolist()}")
        logger.info(f"✅ VLA inference #{request_count} completed")

        success_count += 1
        
        return {
            "status": "success",
            "action": action.tolist(),
            "joint_positions": action[:6].tolist(), # [x, y, z, roll, pitch, yaw]
            "gripper": "close" if action[6] < 0.5 else "open", # gripper: [0, 1]
            "processed_at": datetime.now().isoformat()
        }
        
    except ValueError as e:
        logger.error(f"❌ Validation error: {e}")
        raise HTTPException(status_code=400, detail=f"Invalid image data: {str(e)}")
    except Exception as e:
        logger.error(f"❌ Processing error: {e}")
        raise HTTPException(status_code=500, detail=f"VLA inference failed: {str(e)}")

# @app.get("/stats") 다음에 추가

@app.post("/api/vla/clear_cache")
async def clear_cache():
    """
    CUDA 메모리 캐시 정리 엔드포인트
    명령어가 완전히 끝났을 때 클라이언트가 호출
    """
    try:
        torch.cuda.empty_cache()
        logger.info("🧹 CUDA cache cleared by client request")
        return {
            "status": "success",
            "message": "CUDA cache cleared",
            "timestamp": datetime.now().isoformat()
        }
    except Exception as e:
        logger.error(f"❌ Cache clear error: {e}")
        raise HTTPException(status_code=500, detail=f"Failed to clear cache: {str(e)}")

if __name__ == "__main__":
    # 서버 실행
    uvicorn.run(
        app,
        host="0.0.0.0",
        port=8080,
        log_level="info"
    )
