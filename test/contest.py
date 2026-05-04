# contest.py
# ------------------------------------------------------------
# AR 마커 기반 4족 로봇 주행 미션 코드
#
# 구조:
#   - 보행 자체는 A_crawl_drive.py의 CrawlDriver/Cmd를 그대로 사용한다.
#   - contest.py는 키보드 입력 대신 AR 마커 인식 결과로 주행 명령을 만든다.
#
# 이유:
#   - A_crawl_drive.py에는 이미 안정 보행에 필요한
#     shift / counter / swing / bodymove / recenter 로직이 들어있다.
#   - contest.py에서 별도 보행 드라이버를 단순 구현하면
#     center pose와 stand pose 기준 불일치, bodymove 누락 등으로 자세가 깨질 수 있다.
#
# 동작:
#   - 기본 동작: 직진
#   - 정면 가까운 AR 마커 감지:
#       ID 1 -> 좌회전 90도 제자리 회전 후 직진
#       ID 2 -> 우회전 90도 제자리 회전 후 직진
# ------------------------------------------------------------

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Optional, Tuple

import afb2
import cv2
import numpy as np

from A_crawl_drive import CrawlDriver, Cmd, STAND_XYZ


# ============================================================
# 1. 카메라 / AR 마커 설정
# ============================================================
CAM_W = 640
CAM_H = 480
CAM_FPS = 30

# 화면 중앙 근처에 있는 마커만 정면 마커로 판단
CENTER_X_RATIO = 0.35

# AR마커 화면 면적 기준.
# 마커가 가까울수록 화면에서 크게 보이므로 area 값이 커진다.
TRIGGER_AREA = 1500

# 같은 마커에 너무 자주 반응하지 않도록 하는 시간
MARKER_COOLDOWN_SEC = 2.0

SHOW_CAMERA = True


# ============================================================
# 2. 미션 주행 설정
# ============================================================
# 90도 회전에 필요한 crawl_step 반복 횟수.
# 실제 로봇에서 반드시 튜닝 필요.
TURN_90_STEPS = 10

# 회전 후 다시 직진을 안정적으로 시작하기 위한 직진 step 수
FORWARD_AFTER_TURN_STEPS = 2

# 기본 직진 명령
CMD_FORWARD = Cmd(vx=+1, vy=0, wz=0)

# 제자리 회전 명령
# A_crawl_drive.py 기준 q = wz +1, e = wz -1
CMD_TURN_LEFT = Cmd(vx=0, vy=0, wz=+1)
CMD_TURN_RIGHT = Cmd(vx=0, vy=0, wz=-1)


# ============================================================
# 3. AR 마커 검출
# ============================================================
@dataclass
class MarkerInfo:
    marker_id: int
    cx: int
    cy: int
    area: float


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
        best: Optional[MarkerInfo] = None

        for i in range(len(ids)):
            marker_id = int(ids[i][0])
            pts = corners[i][0].astype(int)

            cx = int(np.mean(pts[:, 0]))
            cy = int(np.mean(pts[:, 1]))
            area = float(cv2.contourArea(pts))

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
            print(f"marker id={marker_id} area={int(area)}", flush=True)

            # 화면 중앙 근처에 있는 마커만 정면 마커로 사용
            if abs(cx - image_cx) > center_limit:
                continue

            info = MarkerInfo(marker_id=marker_id, cx=cx, cy=cy, area=area)

            # 여러 개가 보이면 가장 가까운 것으로 추정되는 큰 마커 선택
            if best is None or info.area > best.area:
                best = info

        return best, frame


# ============================================================
# 4. contest용 미션 제어기
# ============================================================
class ContestMission:
    def __init__(self):
        self.driver = CrawlDriver()
        self.detector = ArMarkerDetector()
        self.last_trigger_t = 0.0

    def start(self):
        # A_crawl_drive.py와 같은 초기화 흐름을 사용한다.
        self.driver.reset()

        print("ready", flush=True)

    def stop(self):
        self.driver.shutdown()

    def forward_step(self):
        print("go", flush=True)
        self.driver.crawl_step(CMD_FORWARD)

    def turn_left_90(self):
        print("left", flush=True)
        self.driver.go_stand(duration=0.25)

        for _ in range(TURN_90_STEPS):
            self.driver.crawl_step(CMD_TURN_LEFT)

        self.driver.go_stand(duration=0.25)

    def turn_right_90(self):
        print("right", flush=True)
        self.driver.go_stand(duration=0.25)

        for _ in range(TURN_90_STEPS):
            self.driver.crawl_step(CMD_TURN_RIGHT)

        self.driver.go_stand(duration=0.25)

    def handle_marker(self, marker: MarkerInfo):
        print(f"marker id={marker.marker_id} area={int(marker.area)}", flush=True)
        if marker.marker_id == 1:
            time.sleep(2)
            self.turn_left_90()
        elif marker.marker_id == 2:
            time.sleep(2)
            self.turn_right_90()
        else:
            print(f"[MARKER] ID={marker.marker_id}는 미션 대상이 아니므로 무시", flush=True)
            return

        # 회전 후 직진을 다시 시작
        for _ in range(FORWARD_AFTER_TURN_STEPS):
            self.forward_step()

    def loop(self):
        while True:
            marker, frame = self.detector.detect_front_marker()

            if SHOW_CAMERA:
                afb2.flask.imshow("Contest", cv2.cvtColor(frame, cv2.COLOR_BGR2RGB), 0)

            now = time.time()
            can_trigger = (now - self.last_trigger_t) > MARKER_COOLDOWN_SEC

            if marker is not None and marker.area > TRIGGER_AREA and can_trigger:
                self.last_trigger_t = now
                self.handle_marker(marker)
            else:
                self.forward_step()


# ============================================================
# 5. 실행부
# ============================================================
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