import afb2
import time
import cv2

# 카메라 초기화
afb2.camera.init(640, 480, 30)

while True:
    frame = afb2.camera.get_image()

    # BGR → RGB 변환
    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    # 웹 스트리밍 출력
    afb2.flask.imshow("AFB Camera", frame_rgb, 0)