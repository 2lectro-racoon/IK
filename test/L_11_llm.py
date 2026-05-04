import afb2
import cv2
import base64
import time
import sys
import select
from openai import OpenAI

# ==============================
# GPT API 설정
# ==============================
client = OpenAI(api_key="YOUR_API_KEY")

# ==============================
# 카메라 초기화
# ==============================
afb2.camera.init(640, 480, 30)

# ==============================
# 상태 변수
# ==============================
gpt_result = "No result yet"

# ==============================
# GPT 분석 함수
# ==============================
def analyze_image(frame):
    _, buffer = cv2.imencode('.jpg', frame)
    img_base64 = base64.b64encode(buffer).decode('utf-8')

    response = client.chat.completions.create(
        model="gpt-4.1-mini",
        messages=[
            {
                "role": "user",
                "content": [
                    {"type": "text", "text": "이 이미지에 있는 물체를 한 문장으로 설명해줘."},
                    {
                        "type": "image_url",
                        "image_url": {
                            "url": f"data:image/jpeg;base64,{img_base64}"
                        }
                    }
                ]
            }
        ]
    )

    return response.choices[0].message.content


# ==============================
# 입력 감지 함수 (논블로킹)
# ==============================
def input_available():
    return select.select([sys.stdin], [], [], 0)[0]


print("엔터를 누르면 GPT 분석 실행")

# ==============================
# 메인 루프
# ==============================
while True:
    frame = afb2.camera.get_image()

    # -----------------------------
    # 터미널 입력 감지
    # -----------------------------
    if input_available():
        sys.stdin.readline()  # 입력 소비

        print("분석 중...")

        result = analyze_image(frame)

        gpt_result = result

        print("GPT 결과:")
        print(result)

    # -----------------------------
    # 화면 표시 (웹)
    # -----------------------------
    display = frame.copy()

    cv2.putText(display, gpt_result, (20, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    frame_rgb = cv2.cvtColor(display, cv2.COLOR_BGR2RGB)
    afb2.flask.imshow("Camera", frame_rgb, 0)