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
#       ID 3 -> 제자리 정지
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

TRIGGER_AREA = 8000

# ID 3 정지 마커는 별도 거리 기준을 사용한다.
ID3_TRIGGER_AREA = 10000

# 같은 마커에 너무 자주 반응하지 않도록 하는 시간
MARKER_COOLDOWN_SEC = 2.0

SHOW_CAMERA = True


# ============================================================
# 2. 미션 주행 설정
# ============================================================
# 90도 회전에 필요한 crawl_step 반복 횟수.
# 실제 로봇에서 반드시 튜닝 필요.
TURN_90_STEPS = 20

# 회전 후 다시 직진을 안정적으로 시작하기 위한 직진 step 수
FORWARD_AFTER_TURN_STEPS = 2


# SPI 전송 안정화용 인터벌
STEP_INTERVAL = 0.12

# 회전 동작은 너무 길어지면 서보 부하 시간이 늘어나 전원 문제가 커질 수 있다.
# 따라서 직진보다 짧게 둔다.
TURN_STEP_INTERVAL = 0.03

# 마커 로그가 카메라 프레임마다 과도하게 출력되지 않도록 제한한다.
MARKER_PRINT_INTERVAL = 0.25

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
        self.last_marker_print_t = 0.0

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
            now = time.time()
            if now - self.last_marker_print_t >= MARKER_PRINT_INTERVAL:
                print(f"marker id={marker_id} area={int(area)}", flush=True)
                self.last_marker_print_t = now

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
        self.last_state = None
        self.is_stopped = False

    def start(self):
        # A_crawl_drive.py와 같은 초기화 흐름을 사용한다.
        self.driver.reset()

        print("ready", flush=True)

    def stop(self):
        self.driver.shutdown()

    def forward_step(self):
        if self.last_state != "go":
            print("go", flush=True)
            self.last_state = "go"
        self.driver.crawl_step(CMD_FORWARD)

    def stop_in_place(self):
        if self.last_state != "stop":
            print("stop", flush=True)
            self.last_state = "stop"
        self.is_stopped = True

    def turn_left_90(self):
        if self.last_state != "left":
            print("left", flush=True)
            self.last_state = "left"
        self.driver.go_stand(duration=0.25)

        for _ in range(TURN_90_STEPS):
            self.driver.crawl_step(CMD_TURN_LEFT)
            time.sleep(TURN_STEP_INTERVAL)

        self.driver.go_stand(duration=0.25)

    def turn_right_90(self):
        if self.last_state != "right":
            print("right", flush=True)
            self.last_state = "right"
        self.driver.go_stand(duration=0.25)

        for _ in range(TURN_90_STEPS):
            self.driver.crawl_step(CMD_TURN_RIGHT)
            time.sleep(TURN_STEP_INTERVAL)

        self.driver.go_stand(duration=0.25)

    def is_marker_triggered(self, marker: MarkerInfo) -> bool:
        if marker.marker_id == 3:
            return marker.area > ID3_TRIGGER_AREA
        return marker.area > TRIGGER_AREA

    def handle_marker(self, marker: MarkerInfo):
        print(f"marker id={marker.marker_id} area={int(marker.area)}", flush=True)

        # ID 3은 최종 정지 마커이므로 회전 처리보다 먼저 판단한다.
        if marker.marker_id == 3:
            self.stop_in_place()
            self.driver.bodyup(duration=0.8)
            return

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
            time.sleep(STEP_INTERVAL)

    def loop(self):
        last_step_t = 0.0

        while True:
            marker, frame = self.detector.detect_front_marker()

            if SHOW_CAMERA:
                afb2.flask.imshow(
                    "Contest",
                    cv2.cvtColor(frame, cv2.COLOR_BGR2RGB),
                    0,
                )

            now = time.time()
            can_trigger = (now - self.last_trigger_t) > MARKER_COOLDOWN_SEC

            if marker is not None and self.is_marker_triggered(marker) and can_trigger:
                self.last_trigger_t = now
                self.handle_marker(marker)

                # 회전 직후 바로 step 연타 방지
                last_step_t = time.time()

            else:
                # ID 3 마커로 정지한 뒤에는 추가 보행 명령을 보내지 않는다.
                if self.is_stopped:
                    continue

                # 일정 주기로만 보행 step 실행
                if now - last_step_t >= STEP_INTERVAL:
                    self.forward_step()
                    last_step_t = now
    # def loop(self):
    #     while True:
    #         marker, frame = self.detector.detect_front_marker()

    #         if SHOW_CAMERA:
    #             afb2.flask.imshow("Contest", cv2.cvtColor(frame, cv2.COLOR_BGR2RGB), 0)

    #         now = time.time()
    #         can_trigger = (now - self.last_trigger_t) > MARKER_COOLDOWN_SEC

    #         if marker is not None and marker.area > TRIGGER_AREA and can_trigger:
    #             self.last_trigger_t = now
    #             self.handle_marker(marker)
    #         else:
    #             self.forward_step()


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