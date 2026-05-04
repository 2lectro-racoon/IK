# contest.py
# ------------------------------------------------------------
# AR 마커 기반 주행 미션 코드
#
# 기본:
#   - 계속 직진
#   - 정면 가까운 AR 마커 인식 시
#       ID 1 → 좌회전 90도
#       ID 2 → 우회전 90도
# ------------------------------------------------------------

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Optional, Tuple

import afb2
import cv2
import numpy as np

from A_crawl_drive import CrawlDriver, Cmd


# ==============================
# 설정값
# ==============================
CAM_W = 640
CAM_H = 480
CAM_FPS = 30

CENTER_X_RATIO = 0.35

TRIGGER_AREA = 8000   # 🔥 AR마커 "화면 면적" 기준 (클수록 가까움)

MARKER_COOLDOWN_SEC = 2.0

TURN_90_STEPS = 4     # 🔥 90도 회전용 step (반드시 튜닝 필요)
FORWARD_AFTER_TURN_STEPS = 2

SHOW_CAMERA = True


# ==============================
# 마커 정보 구조체
# ==============================
@dataclass
class MarkerInfo:
    marker_id: int
    cx: int
    cy: int
    area: float


# ==============================
# AR 마커 검출기
# ==============================
class ArMarkerDetector:
    def __init__(self):
        afb2.camera.init(CAM_W, CAM_H, CAM_FPS)

        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

    def detect_front_marker(self) -> Tuple[Optional[MarkerInfo], np.ndarray]:
        frame = afb2.camera.get_image()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = self.detector.detectMarkers(gray)

        if ids is None:
            return None, frame

        image_cx = CAM_W / 2.0
        center_limit = CAM_W * CENTER_X_RATIO

        best = None

        for i in range(len(ids)):
            marker_id = int(ids[i][0])
            pts = corners[i][0].astype(int)

            cx = int(np.mean(pts[:, 0]))
            cy = int(np.mean(pts[:, 1]))
            area = float(cv2.contourArea(pts))

            # 화면 표시
            cv2.polylines(frame, [pts], True, (0, 255, 0), 2)
            cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
            cv2.putText(
                frame,
                f"ID:{marker_id} A:{int(area)}",
                (cx, cy - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 0, 0),
                2,
            )

            # 중앙 근처만 사용
            if abs(cx - image_cx) > center_limit:
                continue

            if best is None or area > best.area:
                best = MarkerInfo(marker_id, cx, cy, area)

        return best, frame


# ==============================
# 미션 제어
# ==============================
class ContestMission:
    def __init__(self):
        self.driver = CrawlDriver()
        self.detector = ArMarkerDetector()
        self.last_trigger_t = 0.0

    def start(self):
        self.driver.reset()
        self.driver.go_stand(duration=0.6)

        print("[INFO] 시작: 기본 직진")
        print("[INFO] ID 1=좌회전 / ID 2=우회전")
        print(f"[INFO] TRIGGER_AREA={TRIGGER_AREA} 이상이면 반응")

    def stop(self):
        self.driver.shutdown()

    def forward(self):
        self.driver.crawl_step(Cmd(vx=+1, vy=0, wz=0))

    def turn_left(self):
        print("[TURN] LEFT 90deg")
        for _ in range(TURN_90_STEPS):
            self.driver.crawl_step(Cmd(vx=0, vy=0, wz=+1))

    def turn_right(self):
        print("[TURN] RIGHT 90deg")
        for _ in range(TURN_90_STEPS):
            self.driver.crawl_step(Cmd(vx=0, vy=0, wz=-1))

    def after_turn(self):
        for _ in range(FORWARD_AFTER_TURN_STEPS):
            self.forward()

    def handle_marker(self, m: MarkerInfo):
        print(f"[MARKER] ID={m.marker_id} AREA={int(m.area)}")

        self.driver.go_stand(duration=0.25)

        if m.marker_id == 1:
            self.turn_left()
        elif m.marker_id == 2:
            self.turn_right()
        else:
            print("[MARKER] 무시된 ID")
            return

        self.driver.go_stand(duration=0.25)
        self.after_turn()

    def loop(self):
        while True:
            marker, frame = self.detector.detect_front_marker()

            if SHOW_CAMERA:
                afb2.flask.imshow("Contest", cv2.cvtColor(frame, cv2.COLOR_BGR2RGB), 0)

            now = time.time()

            if (
                marker is not None
                and marker.area > TRIGGER_AREA
                and (now - self.last_trigger_t) > MARKER_COOLDOWN_SEC
            ):
                self.last_trigger_t = now
                self.handle_marker(marker)
            else:
                self.forward()


# ==============================
# 실행
# ==============================
def main():
    mission = ContestMission()

    try:
        mission.start()
        mission.loop()

    except KeyboardInterrupt:
        print("\n[EXIT]")

    finally:
        mission.stop()


if __name__ == "__main__":
    main()