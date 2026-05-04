# contest.py
# ------------------------------------------------------------
# AR 마커 기반 4족 로봇 주행 미션 코드
#
# 구조:
#   - A_crawl_drive.CrawlDriver는 사용하지 않는다.
#   - contest.py 내부의 ContestQuadDriver가 A_quad_api만 사용한다.
#   - mission layer는 AR 마커 인식 결과에 따라 driver 명령만 호출한다.
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
from typing import Dict, Optional, Tuple

import afb2
import cv2
import numpy as np

from A_quad_api import make_default_api
from A_ik_3dof_a0 import IKError


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

# 같은 마커에 너무 자주 반응하지 않도록 하는 시간
MARKER_COOLDOWN_SEC = 2.0

SHOW_CAMERA = True


# ============================================================
# 2. 보행 설정
# ============================================================
# 기본 서있는 자세
STAND_XYZ = (60.0, 120.0, -50.0)

# 다리를 들어올리는 높이
LIFT_DZ = 60.0

# 직진 시 swing 다리를 보내는 거리
STEP_FWD = 40.0

# 제자리 회전 시 swing 다리 이동량
STEP_YAW = 60.0

# 90도 회전에 필요한 step 수. 실제 로봇에서 튜닝 필요
TURN_90_STEPS = 4

# 회전 후 다시 직진을 안정적으로 시작하기 위한 직진 step 수
FORWARD_AFTER_TURN_STEPS = 2

# 보간 주기 / 각 phase 시간
MOVE_DT = 0.02
PHASE_T = 0.2
STAND_T = 0.25

# 다리 번호
# 0=FR, 1=BR, 2=BL, 3=FL
FORWARD_SEQ = [1, 0, 2, 3]
TURN_SEQ = [0, 1, 2, 3]

RIGHT_LEGS = {0, 1}
FRONT_LEGS = {0, 3}


# ============================================================
# 3. 공통 유틸
# ============================================================
def clampf(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


def smoothstep(t: float) -> float:
    t = clampf(t, 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)


def lerp(a: float, b: float, t: float) -> float:
    return a + (b - a) * t


def body_x_to_local_x(leg_id: int, body_x: float) -> float:
    """
    BODY x를 각 다리 LOCAL x로 변환한다.
    앞다리: +x가 전방
    뒷다리: +x가 후방
    """
    return body_x if leg_id in FRONT_LEGS else -body_x


def side_sign(leg_id: int) -> int:
    """
    오른쪽 다리 +1, 왼쪽 다리 -1.
    제자리 회전에서 좌우 다리를 반대 방향으로 보내기 위해 사용한다.
    """
    return +1 if leg_id in RIGHT_LEGS else -1


# ============================================================
# 4. AR 마커 검출
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

            if abs(cx - image_cx) > center_limit:
                continue

            info = MarkerInfo(marker_id=marker_id, cx=cx, cy=cy, area=area)
            if best is None or info.area > best.area:
                best = info

        return best, frame


# ============================================================
# 5. contest.py 전용 4족 보행 드라이버
# ============================================================
class ContestQuadDriver:
    """
    A_quad_api만 사용하는 contest 전용 드라이버.

    A_crawl_drive.CrawlDriver를 사용하지 않기 때문에
    키보드 입력 처리, 방향 전환 reset, fb_step 같은 상위 로직과 충돌하지 않는다.
    """

    def __init__(self):
        self.api = make_default_api()
        sx, sy, sz = STAND_XYZ
        self.stand = (sx, sy, sz)
        self.foot: Dict[int, Tuple[float, float, float]] = {
            leg_id: (sx, sy, sz) for leg_id in (0, 1, 2, 3)
        }
        self.forward_idx = 0
        self.turn_idx = 0

    def reset(self):
        # 하드웨어 초기화
        self.api.leg_reset()
        time.sleep(8)

        # 안전하게 센터 포즈로 먼저 이동 (기구적 안정화)
        self.api.go_center_pose(debug=False)
        time.sleep(1.0)

        # 내부 foot 상태를 실제 자세와 동기화
        sx, sy, sz = self.stand
        for leg_id in (0, 1, 2, 3):
            self.foot[leg_id] = (sx, sy, sz)

        print("[RESET] synced to stand pose", flush=True)

    def shutdown(self):
        self.api.go_center_pose(debug=True)

    def _try_set_leg_xyz(self, leg_id: int, x: float, y: float, z: float) -> bool:
        try:
            self.api.set_leg_xyz(leg_id, x, y, z, debug=False)
            return True
        except IKError as e:
            print(
                f"[IKError] leg={leg_id} target=({x:.1f},{y:.1f},{z:.1f}) -> {e}",
                flush=True,
            )
            return False

    def set_leg_pose(self, leg_id: int, x: float, y: float, z: float, duration: float) -> bool:
        x0, y0, z0 = self.foot[leg_id]
        steps = max(1, int(duration / MOVE_DT))

        for i in range(steps + 1):
            u = i / steps
            ue = smoothstep(u)
            xi = lerp(x0, x, ue)
            yi = lerp(y0, y, ue)
            zi = lerp(z0, z, ue)

            if not self._try_set_leg_xyz(leg_id, xi, yi, zi):
                return False

            time.sleep(MOVE_DT)

        self.foot[leg_id] = (x, y, z)
        return True

    def set_all_legs(self, x: float, y: float, z: float, duration: float) -> bool:
        start = {leg_id: self.foot[leg_id] for leg_id in (0, 1, 2, 3)}
        steps = max(1, int(duration / MOVE_DT))

        for s in range(steps + 1):
            u = s / steps
            ue = smoothstep(u)

            for leg_id in (0, 1, 2, 3):
                x0, y0, z0 = start[leg_id]
                xi = lerp(x0, x, ue)
                yi = lerp(y0, y, ue)
                zi = lerp(z0, z, ue)

                if not self._try_set_leg_xyz(leg_id, xi, yi, zi):
                    return False

            time.sleep(MOVE_DT)

        for leg_id in (0, 1, 2, 3):
            self.foot[leg_id] = (x, y, z)

        return True

    def go_stand(self, duration: float = STAND_T):
        sx, sy, sz = self.stand
        self.set_all_legs(sx, sy, sz, duration)

    def forward_step(self):
        """
        직진 1 step.
        한 다리를 들어 전방으로 보낸 뒤 기준 자세로 복귀한다.
        """
        leg_id = FORWARD_SEQ[self.forward_idx % len(FORWARD_SEQ)]
        self.forward_idx += 1

        sx, sy, sz = self.stand
        z_lift = sz + LIFT_DZ
        x0, y0, _ = self.foot[leg_id]
        dx = body_x_to_local_x(leg_id, STEP_FWD)

        print(f"[FWD] leg={leg_id} dx={dx:+.1f}", flush=True)

        if not self.set_leg_pose(leg_id, x0, y0, z_lift, PHASE_T):
            self.go_stand()
            return

        if not self.set_leg_pose(leg_id, x0 + dx, y0, z_lift, PHASE_T):
            self.go_stand()
            return

        if not self.set_leg_pose(leg_id, x0 + dx, y0, sz, PHASE_T):
            self.go_stand()
            return

        # 누적 오차를 줄이기 위해 매 step 후 기준 자세로 복귀한다.
        self.go_stand(duration=0.15)

    def turn_step(self, direction: int):
        """
        제자리 회전 1 step.

        direction:
          +1 = 좌회전
          -1 = 우회전
        """
        leg_id = TURN_SEQ[self.turn_idx % len(TURN_SEQ)]
        self.turn_idx += 1

        sx, sy, sz = self.stand
        z_lift = sz + LIFT_DZ
        x0, y0, _ = self.foot[leg_id]

        body_dx = direction * STEP_YAW * side_sign(leg_id)
        dx = body_x_to_local_x(leg_id, body_dx)

        print(f"[TURN_STEP] leg={leg_id} dir={direction:+d} dx={dx:+.1f}", flush=True)

        if not self.set_leg_pose(leg_id, x0, y0, z_lift, PHASE_T):
            self.go_stand()
            return

        if not self.set_leg_pose(leg_id, x0 + dx, y0, z_lift, PHASE_T):
            self.go_stand()
            return

        if not self.set_leg_pose(leg_id, x0 + dx, y0, sz, PHASE_T):
            self.go_stand()
            return

        # 누적 오차를 줄이기 위해 매 step 후 기준 자세로 복귀한다.
        self.go_stand(duration=0.15)

    def turn_left_90(self):
        print(f"[TURN] LEFT 90deg steps={TURN_90_STEPS}", flush=True)
        self.go_stand(duration=STAND_T)
        self.turn_idx = 0

        for _ in range(TURN_90_STEPS):
            self.turn_step(+1)

        self.go_stand(duration=STAND_T)

    def turn_right_90(self):
        print(f"[TURN] RIGHT 90deg steps={TURN_90_STEPS}", flush=True)
        self.go_stand(duration=STAND_T)
        self.turn_idx = 0

        for _ in range(TURN_90_STEPS):
            self.turn_step(-1)

        self.go_stand(duration=STAND_T)


# ============================================================
# 6. 미션 제어기
# ============================================================
class ContestMission:
    def __init__(self):
        self.driver = ContestQuadDriver()
        self.detector = ArMarkerDetector()
        self.last_trigger_t = 0.0

    def start(self):
        self.driver.reset()
        self.driver.go_stand(duration=0.6)

        print("[INFO] 시작: 기본 직진")
        print("[INFO] ID 1=좌회전 90도 / ID 2=우회전 90도")
        print(f"[INFO] TRIGGER_AREA={TRIGGER_AREA} 이상이면 반응")

    def stop(self):
        self.driver.shutdown()

    def handle_marker(self, marker: MarkerInfo):
        print(f"[MARKER] ID={marker.marker_id} AREA={int(marker.area)}", flush=True)

        if marker.marker_id == 1:
            self.driver.turn_left_90()
        elif marker.marker_id == 2:
            self.driver.turn_right_90()
        else:
            print(f"[MARKER] ID={marker.marker_id}는 미션 대상이 아니므로 무시", flush=True)
            return

        for _ in range(FORWARD_AFTER_TURN_STEPS):
            self.driver.forward_step()

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
                self.driver.forward_step()


# ============================================================
# 7. 실행부
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