import afb2
import cv2
import numpy as np

# 카메라 초기화
afb2.camera.init(640, 480, 30)

# ArUco 딕셔너리 설정
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
parameters = cv2.aruco.DetectorParameters()

detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

while True:
    frame = afb2.camera.get_image()

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # 마커 검출
    corners, ids, rejected = detector.detectMarkers(gray)

    if ids is not None:
        for i in range(len(ids)):
            marker_id = ids[i][0]
            corner = corners[i][0]

            # 4개 좌표
            pts = corner.astype(int)

            # 중심 좌표 계산
            cx = int(np.mean(pts[:, 0]))
            cy = int(np.mean(pts[:, 1]))

            # 면적 계산 (다각형 면적)
            area = cv2.contourArea(pts)

            # bounding box 그리기
            cv2.polylines(frame, [pts], True, (0, 255, 0), 2)

            # 중심점 표시
            cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)

            # 텍스트 출력
            text = f"ID:{marker_id} X:{cx} Y:{cy} A:{int(area)}"
            cv2.putText(frame, text, (cx, cy - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

            # 터미널 로그
            print(f"[MARKER] ID={marker_id} | X={cx} Y={cy} | AREA={int(area)}")

    # 웹 출력
    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    afb2.flask.imshow("AR Marker", frame_rgb, 0)