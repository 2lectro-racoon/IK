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
#       ID 3 -> 정지 후 바디업, 카메라 이미지로 LEFT/RIGHT 판단 후 해당 방향 회전 후 직진
# ------------------------------------------------------------

from __future__ import annotations

import base64
import os
import threading
import time
from dataclasses import dataclass
from typing import Optional, Tuple

import afb2
import cv2
import numpy as np
from openai import OpenAI

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

# ID 3 방향 판단 마커는 별도 거리 기준을 사용한다.
ID3_TRIGGER_AREA = 10000

# 같은 마커에 너무 자주 반응하지 않도록 하는 시간
MARKER_COOLDOWN_SEC = 2.0

SHOW_CAMERA = True

# ID 3 정지 후 카메라 이미지를 OpenAI API로 분석할지 여부
USE_OPENAI_ANALYSIS = True

# OpenAI API 모델. 필요하면 더 가벼운 모델로 바꿔도 된다.
OPENAI_MODEL = "gpt-4.1-mini"


# ============================================================
# 2. 미션 주행 설정
# ============================================================
# 90도 회전에 필요한 crawl_step 반복 횟수.
# 실제 로봇에서 반드시 튜닝 필요.
TURN_90_STEPS = 22

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

        self.lock = threading.Lock()
        self.latest_frame: Optional[np.ndarray] = None
        self.latest_display_frame: Optional[np.ndarray] = None
        self.latest_marker: Optional[MarkerInfo] = None
        self.openai_display_frame: Optional[np.ndarray] = None
        self.running = False
        self.thread: Optional[threading.Thread] = None

    def start(self):
        if self.running:
            return

        self.running = True
        self.thread = threading.Thread(target=self._camera_loop, daemon=True)
        self.thread.start()

    def stop(self):
        self.running = False
        if self.thread is not None:
            self.thread.join(timeout=1.0)
            self.thread = None

    def get_latest(self) -> Tuple[Optional[MarkerInfo], Optional[np.ndarray]]:
        with self.lock:
            marker = self.latest_marker
            frame = None if self.latest_display_frame is None else self.latest_display_frame.copy()
        return marker, frame

    def get_raw_frame(self) -> Optional[np.ndarray]:
        with self.lock:
            if self.latest_frame is None:
                return None
            return self.latest_frame.copy()

    def set_openai_display_frame(self, frame: np.ndarray):
        """OpenAI 분석에 사용한 정지 이미지를 Flask 1번 슬롯에 유지 표시한다."""
        with self.lock:
            self.openai_display_frame = frame.copy()

    def _camera_loop(self):
        while self.running:
            try:
                marker, frame = self.detect_front_marker()

                with self.lock:
                    self.latest_marker = marker
                    self.latest_display_frame = frame.copy()

                if SHOW_CAMERA:
                    afb2.flask.imshow(
                        "Contest",
                        cv2.cvtColor(frame, cv2.COLOR_BGR2RGB),
                        0,
                    )

                    with self.lock:
                        openai_frame = None if self.openai_display_frame is None else self.openai_display_frame.copy()

                    if openai_frame is not None:
                        afb2.flask.imshow(
                            "openai",
                            cv2.cvtColor(openai_frame, cv2.COLOR_BGR2RGB),
                            1,
                        )

            except Exception as e:
                print(f"[CAMERA] 카메라 스레드 오류: {e}", flush=True)
                time.sleep(0.1)

            time.sleep(0.001)

    def detect_front_marker(self) -> Tuple[Optional[MarkerInfo], np.ndarray]:
        frame = afb2.camera.get_image()

        with self.lock:
            self.latest_frame = frame.copy()

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
        self.should_exit = False
        self.openai_client = None

        if USE_OPENAI_ANALYSIS:
            if os.getenv("OPENAI_API_KEY"):
                self.openai_client = OpenAI()
            else:
                print("[OPENAI] OPENAI_API_KEY 환경변수가 없어 이미지 분석을 건너뜁니다.", flush=True)

    def start(self):
        # 카메라/Flask 갱신은 별도 스레드에서 계속 돌린다.
        self.detector.start()

        # A_crawl_drive.py와 같은 초기화 흐름을 사용한다.
        self.driver.reset()

        print("ready", flush=True)

    def stop(self):
        self.detector.stop()
        self.driver.bodyup(60, 120, -10, duration=0.8)
        time.sleep(1)
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

    def analyze_arrow_once(self) -> Optional[str]:
        """
        ID 3 정지/자세 상승 후 카메라 이미지를 한 장 캡처해서
        OpenAI API에 좌/우 방향 판단을 요청한다.
        반환값은 LEFT, RIGHT, NONE 중 하나로 정규화한다.
        """
        if self.openai_client is None:
            return None

        frame = self.detector.get_raw_frame()
        if frame is None:
            print("[OPENAI] 사용할 수 있는 카메라 프레임이 없습니다.", flush=True)
            return None

        if SHOW_CAMERA:
            # OpenAI로 보낼 정지 이미지를 카메라 스레드가 1번 슬롯에 계속 표시하게 한다.
            self.detector.set_openai_display_frame(frame)
            print("DEBUG_OPENAI_IMG", flush=True)

        ok, buffer = cv2.imencode(".jpg", frame)
        if not ok:
            print("[OPENAI] 카메라 프레임 JPEG 인코딩 실패", flush=True)
            return None

        img_base64 = base64.b64encode(buffer).decode("utf-8")

        try:
            response = self.openai_client.chat.completions.create(
                model=OPENAI_MODEL,
                messages=[
                    {
                        "role": "user",
                        "content": [
                            {
                                "type": "text",
                                "text": (
                                    "이미지 안에 방향을 나타내는 화살표, 표식, 글자가 있는지 확인해줘. "
                                    "방향은 왼쪽 또는 오른쪽만 존재한다고 가정해. "
                                    "결과는 반드시 다음 중 하나의 단어만 출력해: LEFT, RIGHT, NONE. "
                                    "왼쪽 방향이면 LEFT, 오른쪽 방향이면 RIGHT, 방향 판단이 어렵거나 없으면 NONE."
                                ),
                            },
                            {
                                "type": "image_url",
                                "image_url": {
                                    "url": f"data:image/jpeg;base64,{img_base64}",
                                },
                            },
                        ],
                    }
                ],
            )
        except Exception as e:
            print(f"[OPENAI] 이미지 분석 실패: {e}", flush=True)
            return None

        result = response.choices[0].message.content.strip().upper()

        if "LEFT" in result:
            result = "LEFT"
        elif "RIGHT" in result:
            result = "RIGHT"
        else:
            result = "NONE"

        print(f"[OPENAI] arrow={result}", flush=True)
        return result

    def handle_marker(self, marker: MarkerInfo):
        print(f"marker id={marker.marker_id} area={int(marker.area)}", flush=True)

        # ID 4는 미션 종료 마커이다.
        # loop를 빠져나가면 main()의 finally에서 stop()이 호출된다.
        if marker.marker_id == 4:
            print("exit", flush=True)
            self.should_exit = True
            return

        # ID 3은 방향 판단 마커이다.
        # 정지 후 바디업 상태에서 카메라 이미지를 분석하고,
        # LEFT/RIGHT 결과에 따라 기존 ID 1/2 회전 루틴을 재사용한다.
        if marker.marker_id == 3:
            self.stop_in_place()
            self.driver.bodyup(100, 100, -100, duration=0.8)

            direction = self.analyze_arrow_once()

            # 바디업 상태에서 바로 보행하면 자세 기준이 꼬일 수 있으므로
            # 기존 보행 기준 자세로 돌아온 뒤 회전한다.
            self.is_stopped = False
            # self.driver.go_stand(duration=0.4)
            # time.sleep(3)
            self.driver.bodyup(60, 120, -50, duration=0.8)
            time.sleep(3)

            if direction == "LEFT":
                self.turn_left_90()
            elif direction == "RIGHT":
                self.turn_right_90()
            else:
                print("[OPENAI] 방향 판단 실패: 회전하지 않고 직진 재개", flush=True)

            for _ in range(FORWARD_AFTER_TURN_STEPS):
                self.forward_step()
                time.sleep(STEP_INTERVAL)

            return

        if marker.marker_id == 1:
            time.sleep(0.5)
            self.turn_left_90()
        elif marker.marker_id == 2:
            time.sleep(0.5)
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

        while not self.should_exit:
            marker, _ = self.detector.get_latest()

            now = time.time()
            can_trigger = (now - self.last_trigger_t) > MARKER_COOLDOWN_SEC

            if marker is not None and self.is_marker_triggered(marker) and can_trigger:
                self.last_trigger_t = now
                self.handle_marker(marker)

                if self.should_exit:
                    break

                # 회전 직후 바로 step 연타 방지
                last_step_t = time.time()

            else:
                # 정지 상태에서는 추가 보행 명령을 보내지 않는다.
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