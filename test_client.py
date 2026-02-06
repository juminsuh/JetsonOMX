import requests
import base64
import numpy as np
import cv2

# 1. 서버 주소 (본인 서버 IP로 수정)
url = "http://localhost:8080/api/vla/infer"

# 2. 가짜 이미지 생성 (Dummy 이미지)
dummy_img = np.zeros((224, 224, 3), dtype=np.uint8)
_, buffer = cv2.imencode('.jpg', dummy_img)
img_base64 = base64.b64encode(buffer).decode('utf-8')

# 3. 서버에 보낼 데이터
payload = {
    "image": img_base64,
    "prompt": "Pick up the red block",
    "unnorm_key": "libero_object"
}

print(f"서버({url})로 요청을 보냅니다...")

try:
    # 4. POST 요청 보내기
    response = requests.post(url, json=payload)
    
    if response.status_code == 200:
        print("✅ 서버 응답 성공!")
        print("결과값(Action):", response.json())
    else:
        print(f"❌ 서버 응답 실패 (상태 코드: {response.status_code})")
        print(f"에러 내용: {response.text}")
except Exception as e:
    print(f"🔥 접속 에러: {e}")
