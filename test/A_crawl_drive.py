# crawl_drive.py
# ------------------------------------------------------------
# 4족보행 로봇의 Crawl gait(한 발씩 이동하는 보행) 제어 코드
#
# 이 코드는 키보드 입력(W/A/S/D/Q/E)을 받아서
# 로봇이 전진 / 후진 / 좌이동 / 우이동 / 제자리 회전을 수행하도록 만든다.
#
# 핵심 흐름:
#   1) 사용자가 키를 누른다.
#   2) 그 키를 이동 명령(Cmd)으로 바꾼다.
#   3) Crawl gait 순서에 맞는 다리 하나를 선택한다.
#   4) 몸의 무게중심을 약간 옮긴다(shift).
#   5) 선택된 다리를 든다(lift).
#   6) 앞으로/옆으로/회전 방향에 맞게 그 다리를 옮긴다(swing).
#   7) 다시 바닥에 내린다(touchdown).
#   8) 임시로 주었던 shift/counter를 원래대로 되돌린다(unshift).
#
# 특징:
# - quad_api.make_default_api()를 사용해서 실제 IK 및 서보 명령을 보낸다.
# - z값이 "클수록 다리가 위로 올라간다"는 현재 로봇 기준을 따른다.
# - 교육용 이해를 위해 각 단계에 상세 주석을 달았다.
#
# 실행:
#   python3 crawl_drive.py
# ------------------------------------------------------------

from __future__ import annotations

import sys
import time
import math
import select
import termios
import tty
from dataclasses import dataclass
from typing import Dict, Tuple

from A_quad_api import make_default_api
from A_ik_3dof_a0 import IKError
import A_ik_3dof_a0 as ikmod


# ============================================================
# 1. 조절 가능한 파라미터 영역
# ============================================================
# 이 값들을 바꾸면 보행 크기, 속도, 안정성 등이 달라진다.
# 수업에서는 이 부분을 학생들이 직접 바꿔보게 하면 매우 좋다.

# 기본 서있는 자세 (각 다리의 로컬 좌표계 기준 x, y, z)
# x : 앞/뒤 방향
# y : 좌/우 벌어진 정도
# z : 높이 (현재 시스템은 z가 클수록 다리 상승)
STAND_XYZ = (60.0, 120.0, -50.0)

# 다리를 얼마나 들어올릴지
# 예: stand z=-50, lift_dz=60이면 들어올릴 때 z=10이 됨
LIFT_DZ = 60.0

# 몸을 한쪽으로 얼마나 기울일지(무게중심 이동)
# 한 다리를 들기 전에 반대 방향 지지력을 높이기 위해 사용
SHIFT_MAG = 20.0

# ------------------------------------------------------------
# Counterweight(카운터 밸런스) 관련
# ------------------------------------------------------------
# 다리를 들 때 대각선 다리나 다른 지지 다리를 임시로 더 바깥으로 뻗어
# 중심이 무너지지 않게 보조하는 용도

COUNTER_DX = 20.0
COUNTER_DY = 20.0
COUNTER_DZ = 20.0

# 대각선 다리 외에 반대편 다른 지지 다리에도 카운터를 줄지 여부
COUNTER2_ENABLE = True

# 두 번째 보조 다리는 얼마나 약하게 적용할지
COUNTER2_SCALE = 0.6

# ------------------------------------------------------------
# 보폭 관련
# ------------------------------------------------------------
STEP_FWD = 40.0     # 전진/후진 한 번에 이동하는 크기
STEP_LAT = 30.0     # 좌/우 이동 한 번에 이동하는 크기
STEP_YAW = 60.0     # 회전 시 양쪽 다리 차등 이동 크기

# 전/후진 시 swing 다리를 더 크게 보내기 위한 배수
FB_SWING_SCALE = 2.0

# swing 중간에 약간 더 위로 들어올려서 발 궤적을 자연스럽게 만드는 값
SWING_ARC_DZ = 15.0

# ------------------------------------------------------------
# BODY MOVE (몸이 전진하는 느낌을 만드는 전체 발 이동)
# ------------------------------------------------------------
# 다리만 앞으로 놓고 끝나는 것이 아니라,
# 바닥에 닿아 있는 모든 다리를 반대로 밀어
# 몸체가 전진하는 느낌을 만드는 단계
BODYMOVE_ENABLE = True
BODYMOVE_T = 0.35

BODYMOVE_FWD = 1.25
BODYMOVE_LAT = 0.9
BODYMOVE_YAW = 0.7

# ------------------------------------------------------------
# Drift(조금씩 자세가 틀어지는 현상) 방지용
# ------------------------------------------------------------
RECENTER_ENABLE = True
RECENTER_K = 0.2

# 과거 push용 변수 (현재는 사용 안 함)
PUSH_ENABLE = False
PUSH_T = 0.25
PUSH_FWD = 20.0
PUSH_LAT = 12.0
PUSH_YAW = 15.0

# ------------------------------------------------------------
# 시간 관련
# ------------------------------------------------------------
# 보간 주기(작을수록 부드럽지만 연산 많아짐)
MOVE_DT = 0.02

# shift/lift/swing/down/unshift 각 단계 시간
PHASE_T = 0.2

# 일정 시간 입력이 없으면 정지 상태로 바꿈
IDLE_HOLD = 0.35

# ------------------------------------------------------------
# Crawl 순서
# ------------------------------------------------------------
# 다리를 어떤 순서로 들 것인지 정의
# 0=FR, 1=BR, 2=BL, 3=FL
CRAWL_ORDER_FWD = [0, 1, 3, 2]   # 전진 시
CRAWL_ORDER_BACK = [1, 0, 2, 3]  # 후진 시

# 오른쪽 다리 / 왼쪽 다리 구분
RIGHT_LEGS = {0, 1}   # FR, BR
LEFT_LEGS = {2, 3}    # BL, FL

# 앞다리 / 뒷다리 구분
FRONT_LEGS = {0, 3}   # FR, FL
BACK_LEGS = {1, 2}    # BR, BL

# 대각선 다리 매핑
# 예: FR(0)의 대각은 BL(2)
DIAG_LEG = {0: 2, 2: 0, 1: 3, 3: 1}

# 각 다리 기준 "반대편 측면 다리들"
OPPOSITE_SIDE_LEGS = {
    0: LEFT_LEGS,
    1: LEFT_LEGS,
    2: RIGHT_LEGS,
    3: RIGHT_LEGS,
}


# ============================================================
# 2. 작은 수학/좌표 변환 보조 함수
# ============================================================

def smoothstep(t: float) -> float:
    """
    부드러운 보간 함수.
    0~1 사이의 t를 받아 시작/끝에서 급격하지 않게 움직이도록 만든다.

    일반 선형 보간보다 시작/끝이 부드럽기 때문에
    서보가 더 자연스럽게 움직인다.
    """
    t = clampf(t, 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)


def lerp(a: float, b: float, t: float) -> float:
    """
    선형 보간(linear interpolation).
    t=0이면 a, t=1이면 b.
    """
    return a + (b - a) * t


def clampf(x: float, lo: float, hi: float) -> float:
    """
    x를 [lo, hi] 범위 안으로 제한한다.
    """
    return lo if x < lo else hi if x > hi else x


def body_y_to_local_y(leg_id: int, body_y: float) -> float:
    """
    BODY 좌표계의 y 이동량을 각 다리의 LOCAL y로 변환한다.

    BODY 좌표계:
      +Y = 왼쪽

    그런데 각 다리는 자기 기준(local) y방향이 서로 다를 수 있다.
    현재 로봇 기준:
      - 오른쪽 다리(RIGHT_LEGS)는 local +y가 body +Y와 같은 방향
      - 왼쪽 다리(LEFT_LEGS)는 local +y가 body -Y와 같은 방향

    따라서:
      오른쪽 다리: local_y = +body_y
      왼쪽 다리 : local_y = -body_y
    """
    return body_y if leg_id in RIGHT_LEGS else -body_y


def body_x_to_local_x(leg_id: int, body_x: float) -> float:
    """
    BODY 좌표계의 x 이동량을 각 다리의 LOCAL x로 변환한다.

    측정 결과:
      - 앞다리(FR, FL)는 local +x가 몸 기준 앞으로 감
      - 뒷다리(BR, BL)는 local +x가 몸 기준 뒤로 감

    따라서:
      앞다리: local_x = +body_x
      뒷다리: local_x = -body_x
    """
    return body_x if leg_id in FRONT_LEGS else -body_x


def side_sign(leg_id: int) -> int:
    """
    다리가 오른쪽이면 +1, 왼쪽이면 -1 반환.
    yaw 회전 시 좌/우 다리에 반대 부호를 주기 위해 사용.
    """
    return +1 if leg_id in RIGHT_LEGS else -1


# ============================================================
# 3. 디버그용 IK 확인 함수
# ============================================================

def _try_ik_angles_deg(x: float, y: float, z: float) -> Tuple[float, float, float] | None:
    """
    현재 목표 xyz에 대해 IK 각도를 '디버그용'으로 추정해서 출력하기 위한 함수.

    중요한 점:
    - 이 함수는 "출력용"이다.
    - 실제 로봇 제어 로직은 이 함수 결과에 의존하면 안 된다.
    - A_ik_3dof_a0 내부 함수 이름이 여러 버전일 수 있어서
      후보 이름들을 순서대로 찾아본다.
    """
    cand_names = [
        "ik_deg",
        "ik_solve_deg",
        "solve_ik_deg",
        "ik",
        "ik_solve",
        "solve_ik",
        "solve",
    ]

    fn = None
    for nm in cand_names:
        fn = getattr(ikmod, nm, None)
        if callable(fn):
            break

    if fn is None:
        return None

    try:
        out = fn(x, y, z)
    except Exception:
        return None

    try:
        if isinstance(out, (list, tuple)) and len(out) >= 3:
            a0, a1, a2 = float(out[0]), float(out[1]), float(out[2])
        elif isinstance(out, dict):
            a0 = float(out.get("a0"))
            a1 = float(out.get("a1"))
            a2 = float(out.get("a2"))
        else:
            return None
    except Exception:
        return None

    # 값이 radian처럼 보이면 degree로 변환
    if max(abs(a0), abs(a1), abs(a2)) <= 6.3:
        a0 = math.degrees(a0)
        a1 = math.degrees(a1)
        a2 = math.degrees(a2)

    return (a0, a1, a2)


# ============================================================
# 4. 키보드 입력 처리
# ============================================================

class KeyReader:
    """
    터미널에서 키를 실시간으로 읽기 위한 클래스.

    일반 input()은 Enter를 눌러야 입력이 들어오므로
    로봇 실시간 제어에는 적합하지 않다.
    그래서 터미널을 cbreak 모드로 바꿔
    한 글자씩 즉시 읽을 수 있도록 만든다.
    """

    def __init__(self):
        self.fd = sys.stdin.fileno()
        self.old = termios.tcgetattr(self.fd)

    def __enter__(self):
        tty.setcbreak(self.fd)
        return self

    def __exit__(self, exc_type, exc, tb):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)

    def read_key(self) -> str | None:
        """
        읽을 키가 있으면 1글자를 반환하고,
        없으면 None 반환.
        """
        r, _, _ = select.select([sys.stdin], [], [], 0.0)
        if not r:
            return None
        ch = sys.stdin.read(1)
        return ch

    def drain_last_key(self) -> str | None:
        """
        입력 버퍼에 여러 키가 쌓여 있으면
        마지막 키만 남기고 다 버린다.

        이유:
        - 키보드 자동 반복 때문에 입력이 너무 많이 밀리면
          로봇 반응이 늦어진다.
        - 가장 최신 입력만 반영하는 것이 실시간 제어에 더 적합하다.
        """
        last = None
        while True:
            k = self.read_key()
            if k is None:
                return last
            last = k


@dataclass
class Cmd:
    """
    이동 명령 구조체.
    vx : 전진(+1), 후진(-1)
    vy : 좌/우 이동
    wz : 회전(+1 CCW, -1 CW)
    """
    vx: int = 0
    vy: int = 0
    wz: int = 0


def key_to_cmd(k: str) -> Cmd:
    """
    키보드 입력을 이동 명령으로 바꾼다.

    여기서 부호 체계를 수업 중 꼭 짚고 넘어가면 좋다.
    """
    k = k.lower()
    if k == "w":
        return Cmd(vx=+1, vy=0, wz=0)
    if k == "s":
        return Cmd(vx=-1, vy=0, wz=0)
    if k == "a":
        return Cmd(vx=0, vy=-1, wz=0)  # 현재 코드 기준 left
    if k == "d":
        return Cmd(vx=0, vy=+1, wz=0)  # 현재 코드 기준 right
    if k == "q":
        return Cmd(vx=0, vy=0, wz=+1)  # 반시계 회전
    if k == "e":
        return Cmd(vx=0, vy=0, wz=-1)  # 시계 회전
    return Cmd(0, 0, 0)


# ============================================================
# 5. Crawl gait 드라이버 핵심 클래스
# ============================================================

class CrawlDriver:
    """
    실제 crawl gait를 수행하는 핵심 클래스.

    역할:
    - 현재 각 다리 끝점 위치 저장
    - 키 입력에 따라 다음에 움직일 다리 결정
    - shift / lift / swing / touchdown / unshift 실행
    - IK 에러 발생 시 안전하게 복귀
    """

    def __init__(self):
        self.api = make_default_api()

        sx, sy, sz = STAND_XYZ

        # 각 다리 발끝의 현재 목표 위치
        # leg_id : (x, y, z)
        self.foot: Dict[int, Tuple[float, float, float]] = {
            i: (sx, sy, sz) for i in (0, 1, 2, 3)
        }

        self.stand = (sx, sy, sz)

        # HOME 위치:
        # drift 보정 시 기준이 되는 "원래 자리"
        self.home: Dict[int, Tuple[float, float, float]] = {
            i: self.foot[i] for i in (0, 1, 2, 3)
        }

        self.order_idx = 0
        self._crawl_order_key = "fwd"

        # 전/후진 전용 순서 인덱스
        self.fb_idx = 0
        self._fb_dir = 0

        # 직전 이동 명령
        self._last_move_cmd = Cmd(0, 0, 0)

    def _try_set_leg_xyz(self, leg_id: int, x: float, y: float, z: float) -> bool:
        """
        한 다리의 목표 xyz를 실제 API에 전달한다.

        성공하면 True
        IK 도달 불가면 False
        """
        try:
            self.api.set_leg_xyz(leg_id, x, y, z, debug=False)
            return True
        except IKError as e:
            print(f"[IKError] leg={leg_id} target=({x:.1f},{y:.1f},{z:.1f}) -> {e}", flush=True)
            return False

    def set_pose(self, leg_id: int, x: float, y: float, z: float, duration: float) -> bool:
        """
        한 다리를 현재 위치에서 목표 위치까지 부드럽게 이동시킨다.

        포인트:
        - 바로 점프하지 않고 여러 step으로 나누어 이동
        - smoothstep으로 자연스럽게 가속/감속
        """
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

    def set_all(self, x: float, y: float, z: float, duration: float) -> bool:
        """
        모든 다리를 같은 xyz로 동시에 이동시킨다.
        stand 자세로 갈 때 주로 사용.
        """
        steps = max(1, int(duration / MOVE_DT))

        x0 = {i: self.foot[i][0] for i in (0, 1, 2, 3)}
        y0 = {i: self.foot[i][1] for i in (0, 1, 2, 3)}
        z0 = {i: self.foot[i][2] for i in (0, 1, 2, 3)}

        for s in range(steps + 1):
            u = s / steps
            ue = smoothstep(u)

            for leg_id in (0, 1, 2, 3):
                xi = lerp(x0[leg_id], x, ue)
                yi = lerp(y0[leg_id], y, ue)
                zi = lerp(z0[leg_id], z, ue)

                if not self._try_set_leg_xyz(leg_id, xi, yi, zi):
                    return False

            time.sleep(MOVE_DT)

        for leg_id in (0, 1, 2, 3):
            self.foot[leg_id] = (x, y, z)

        return True

    def go_stand(self, duration: float = 0.4):
        """
        모든 다리를 기본 stand 자세로 이동.

        이 동작은 매우 중요하다.
        - 초기 자세 세팅
        - 방향 전환 시 재정렬
        - IK 에러 이후 복구
        """
        sx, sy, sz = self.stand
        _ = self.set_all(sx, sy, sz, duration)

        # home도 현재 stand 기준으로 갱신
        self.home = {i: self.foot[i] for i in (0, 1, 2, 3)}

    def shift_body(self, swing_leg: int, body_shift_y: float, duration: float):
        """
        한 다리를 들기 전에 몸의 무게중심을 반대쪽으로 옮기는 단계.

        실제로 몸체를 이동시키는 것이 아니라,
        발 위치를 반대로 움직여서 '몸이 기울어진 효과'를 만든다.

        body_shift_y:
          +면 몸을 왼쪽으로 옮기는 효과
          -면 몸을 오른쪽으로 옮기는 효과
        """
        targets = {}

        for leg_id in (0, 1, 2, 3):
            x, y, z = self.foot[leg_id]
            dy_local = body_y_to_local_y(leg_id, body_shift_y)
            targets[leg_id] = (x, y + dy_local, z)

        steps = max(1, int(duration / MOVE_DT))
        start = {i: self.foot[i] for i in (0, 1, 2, 3)}

        for s in range(steps + 1):
            u = s / steps
            ue = smoothstep(u)

            for leg_id in (0, 1, 2, 3):
                x0, y0, z0 = start[leg_id]
                xt, yt, zt = targets[leg_id]

                xi = lerp(x0, xt, ue)
                yi = lerp(y0, yt, ue)
                zi = lerp(z0, zt, ue)

                if not self._try_set_leg_xyz(leg_id, xi, yi, zi):
                    return

            time.sleep(MOVE_DT)

        for leg_id in (0, 1, 2, 3):
            self.foot[leg_id] = targets[leg_id]

    def _push_all(self, cmd: Cmd) -> bool:
        """
        BODY MOVE 단계.

        swing 다리를 놓은 뒤,
        모든 발을 바닥에 붙인 상태에서
        전체 발 위치를 명령 반대 방향으로 밀어
        몸체가 실제로 전진/이동/회전하는 느낌을 만든다.
        """
        if not BODYMOVE_ENABLE:
            return True

        if cmd.vx == 0 and cmd.vy == 0 and cmd.wz == 0:
            return True

        sx, sy, sz = self.stand

        # 몸은 앞으로 가야 하므로, 발은 상대적으로 뒤로 미는 방향
        body_dx = -(cmd.vx * STEP_FWD * BODYMOVE_FWD)
        body_dy = -(cmd.vy * STEP_LAT * BODYMOVE_LAT)
        body_dx_yaw = -(cmd.wz * STEP_YAW * BODYMOVE_YAW)

        print(
            f"[BODYMOVE] cmd(vx,vy,wz)=({cmd.vx:+d},{cmd.vy:+d},{cmd.wz:+d}) "
            f"body_dx={body_dx:+.1f} body_dy={body_dy:+.1f} body_dx_yaw={body_dx_yaw:+.1f} T={BODYMOVE_T:.2f}",
            flush=True,
        )

        targets: Dict[int, Tuple[float, float, float]] = {}

        for leg_id in (0, 1, 2, 3):
            x_cur, y_cur, _ = self.foot[leg_id]

            # yaw는 좌우 다리에 반대 부호를 준다.
            body_dx_leg = body_dx + body_dx_yaw * side_sign(leg_id)

            dx_local = body_x_to_local_x(leg_id, body_dx_leg)
            dy_local = body_y_to_local_y(leg_id, body_dy)

            xt = x_cur + dx_local
            yt = y_cur + dy_local

            # drift 방지: home 쪽으로 약간 당겨줌
            if RECENTER_ENABLE:
                hx, hy, _ = self.home.get(leg_id, (sx, sy, sz))
                xt = xt + RECENTER_K * (hx - xt)
                yt = yt + RECENTER_K * (hy - yt)

            targets[leg_id] = (xt, yt, sz)

        if RECENTER_ENABLE:
            print(f"[RECENTER] enabled k={RECENTER_K:.3f} (targets pulled toward HOME)", flush=True)

        steps = max(1, int(BODYMOVE_T / MOVE_DT))
        start = {i: self.foot[i] for i in (0, 1, 2, 3)}

        for s in range(steps + 1):
            u = s / steps
            ue = smoothstep(u)

            for leg_id in (0, 1, 2, 3):
                x0p, y0p, z0p = start[leg_id]
                xtp, ytp, ztp = targets[leg_id]

                xi = lerp(x0p, xtp, ue)
                yi = lerp(y0p, ytp, ue)
                zi = lerp(z0p, ztp, ue)

                if not self._try_set_leg_xyz(leg_id, xi, yi, zi):
                    return False

            time.sleep(MOVE_DT)

        for leg_id in (0, 1, 2, 3):
            self.foot[leg_id] = targets[leg_id]

        return True

    def _single_leg_step_no_push(self, swing_leg: int, cmd: Cmd, *, swing_scale: float = 1.0, support_move: bool = True) -> bool:
        """
        한 다리만 움직이는 crawl step의 핵심 동작.
        단, BODYMOVE(push) 단계는 포함하지 않는다.

        순서:
          1) SHIFT
          2) COUNTER
          3) LIFT
          4) SWING
          5) TOUCHDOWN
          6) UNSHIFT
        """
        diag_leg = DIAG_LEG[swing_leg]

        # 대각선 다리 외의 추가 보조 다리 선정
        opp_side = OPPOSITE_SIDE_LEGS[swing_leg]
        ctr2_leg = None
        if COUNTER2_ENABLE:
            cand = [lid for lid in opp_side if lid != diag_leg]
            ctr2_leg = cand[0] if cand else None

        # 카운터용 임시 이동량
        dx_ctr_local = +COUNTER_DX
        dy_ctr_local = +COUNTER_DY
        dz_ctr_local = +COUNTER_DZ

        dx_ctr2_local = dx_ctr_local * COUNTER2_SCALE if ctr2_leg is not None else 0.0
        dy_ctr2_local = dy_ctr_local * COUNTER2_SCALE if ctr2_leg is not None else 0.0
        dz_ctr2_local = dz_ctr_local * COUNTER2_SCALE if ctr2_leg is not None else 0.0

        sx, sy, sz = self.stand
        z_lift = sz + LIFT_DZ

        # BODY 기준 명령량
        body_dx = cmd.vx * STEP_FWD * swing_scale
        body_dy = cmd.vy * STEP_LAT
        body_dx_yaw = cmd.wz * STEP_YAW

        # swing 다리에 대해 BODY -> LOCAL 변환
        body_dx_swing = body_dx + body_dx_yaw * side_sign(swing_leg)
        dx_leg = body_x_to_local_x(swing_leg, body_dx_swing)
        dy_leg = body_y_to_local_y(swing_leg, body_dy)

        support = [i for i in (0, 1, 2, 3) if i != swing_leg]
        body_dy_support = (-body_dy / len(support)) if (support and support_move) else 0.0

        # ----------------------------------------------------
        # 1) SHIFT : 들 다리 반대쪽으로 몸의 중심 이동
        # ----------------------------------------------------
        desired_body_shift = +SHIFT_MAG if swing_leg in RIGHT_LEGS else -SHIFT_MAG
        self.shift_body(swing_leg, desired_body_shift, PHASE_T)

        # ----------------------------------------------------
        # 1b) COUNTER : 대각선/보조 다리를 임시로 더 뻗어서 안정성 확보
        # ----------------------------------------------------
        xd, yd, zd = self.foot[diag_leg]
        if not self.set_pose(diag_leg, xd + dx_ctr_local, yd + dy_ctr_local, zd + dz_ctr_local, PHASE_T):
            return False

        if ctr2_leg is not None:
            x2, y2, z2 = self.foot[ctr2_leg]
            if not self.set_pose(ctr2_leg, x2 + dx_ctr2_local, y2 + dy_ctr2_local, z2 + dz_ctr2_local, PHASE_T):
                return False

        # ----------------------------------------------------
        # 2) LIFT : 선택된 다리를 위로 든다
        # ----------------------------------------------------
        x0, y0, _ = self.foot[swing_leg]
        if not self.set_pose(swing_leg, x0, y0, z_lift, PHASE_T):
            return False

        # ----------------------------------------------------
        # 3) SWING : 다리를 목표 방향으로 이동
        # ----------------------------------------------------
        swing_target = (x0 + dx_leg, y0 + dy_leg, z_lift)

        support_targets = {}
        if support_move:
            for leg_id in support:
                xs, ys, zs = self.foot[leg_id]

                body_dx_support = -(body_dx + body_dx_yaw * side_sign(leg_id)) / len(support)
                sup_dx_local = body_x_to_local_x(leg_id, body_dx_support)
                sup_dy_local = body_y_to_local_y(leg_id, body_dy_support)

                support_targets[leg_id] = (xs + sup_dx_local, ys + sup_dy_local, zs)

        steps = max(1, int(PHASE_T / MOVE_DT))
        swing_start = self.foot[swing_leg]
        support_start = {i: self.foot[i] for i in support}

        for s in range(steps + 1):
            u = s / steps
            ue = smoothstep(u)

            # swing 다리 보간
            xS0, yS0, zS0 = swing_start
            xSt, ySt, zSt = swing_target

            xi = lerp(xS0, xSt, ue)
            yi = lerp(yS0, ySt, ue)
            zi = lerp(zS0, zSt, ue)

            # 중간에 살짝 더 들어올려서 자연스러운 곡선 궤적 형성
            if SWING_ARC_DZ and SWING_ARC_DZ > 0:
                zi = zi + SWING_ARC_DZ * math.sin(math.pi * ue)

            if not self._try_set_leg_xyz(swing_leg, xi, yi, zi):
                return False

            # 지지 다리도 약간 반대방향으로 이동
            if support_move:
                for leg_id in support:
                    xA0, yA0, zA0 = support_start[leg_id]
                    xAt, yAt, zAt = support_targets[leg_id]

                    xj = lerp(xA0, xAt, ue)
                    yj = lerp(yA0, yAt, ue)
                    zj = lerp(zA0, zAt, ue)

                    if not self._try_set_leg_xyz(leg_id, xj, yj, zj):
                        return False

            time.sleep(MOVE_DT)

        self.foot[swing_leg] = swing_target
        if support_move:
            for leg_id in support:
                self.foot[leg_id] = support_targets[leg_id]

        # ----------------------------------------------------
        # 4) TOUCHDOWN : 다리를 다시 바닥에 내린다
        # ----------------------------------------------------
        x1, y1, _ = self.foot[swing_leg]
        if not self.set_pose(swing_leg, x1, y1, sz, PHASE_T):
            return False

        # ----------------------------------------------------
        # 5) UNSHIFT : shift / counter를 되돌린다
        # ----------------------------------------------------
        steps = max(1, int(PHASE_T / MOVE_DT))
        start = {i: self.foot[i] for i in (0, 1, 2, 3)}

        x_target = {}
        y_target = {}

        for leg_id in (0, 1, 2, 3):
            dy_local_shift = body_y_to_local_y(leg_id, desired_body_shift)
            x_cur, y_cur, _ = self.foot[leg_id]

            y_t = y_cur - dy_local_shift
            x_t = x_cur

            if leg_id == diag_leg:
                x_t = x_t - dx_ctr_local
                y_t = y_t - dy_ctr_local

            if ctr2_leg is not None and leg_id == ctr2_leg:
                x_t = x_t - dx_ctr2_local
                y_t = y_t - dy_ctr2_local

            x_target[leg_id] = x_t
            y_target[leg_id] = y_t

        for s in range(steps + 1):
            u = s / steps
            ue = smoothstep(u)

            for leg_id in (0, 1, 2, 3):
                x0u, y0u, z0u = start[leg_id]
                xi = lerp(x0u, x_target[leg_id], ue)
                yi = lerp(y0u, y_target[leg_id], ue)
                zi = lerp(z0u, sz, ue)

                if not self._try_set_leg_xyz(leg_id, xi, yi, zi):
                    return False

            time.sleep(MOVE_DT)

        for leg_id in (0, 1, 2, 3):
            self.foot[leg_id] = (x_target[leg_id], y_target[leg_id], sz)

        return True

    def fb_step(self, cmd: Cmd):
        """
        전진/후진 전용 보행.

        일반 crawl과 별도로,
        side pair 기반으로 더 직관적인 전/후진 시퀀스를 사용한다.

        전진:
          BR -> FR -> BODYMOVE -> BL -> FL -> BODYMOVE

        후진:
          FR -> BR -> BODYMOVE -> FL -> BL -> BODYMOVE
        """
        if cmd.vx > 0:
            seq = [1, 0, -1, 2, 3, -1]
            seq_name = "W/FWD"
        else:
            seq = [0, 1, -1, 3, 2, -1]
            seq_name = "S/BACK"

        print(f"[FB] select seq={seq_name} cmd.vx={cmd.vx:+d} -> {seq}", flush=True)

        item = seq[self.fb_idx % len(seq)]
        self.fb_idx += 1

        # -1은 BODYMOVE 단계 의미
        if item == -1:
            print(f"[FB] step={self.fb_idx:04d} phase=BODYMOVE", flush=True)
            ok = self._push_all(cmd)
            if not ok:
                self.go_stand(duration=0.3)
            return

        # 디버그용 swing 목표 출력
        x0, y0, z0 = self.foot[item]
        body_dx = cmd.vx * STEP_FWD * FB_SWING_SCALE
        body_dx_yaw = cmd.wz * STEP_YAW
        body_dy = cmd.vy * STEP_LAT

        body_dx_swing = body_dx + body_dx_yaw * side_sign(item)
        dx_leg = body_x_to_local_x(item, body_dx_swing)
        dy_leg = body_y_to_local_y(item, body_dy)

        sx, sy, sz = self.stand
        z_lift = sz + LIFT_DZ
        swing_tgt = (x0 + dx_leg, y0 + dy_leg, z_lift)

        ang = _try_ik_angles_deg(*swing_tgt)
        if ang is None:
            print(
                f"[FB] step={self.fb_idx:04d} leg={item} (BL/FL/BR/FR)="
                f"{['FR','BR','BL','FL'][item]} "
                f"swing_tgt=({swing_tgt[0]:+.1f},{swing_tgt[1]:+.1f},{swing_tgt[2]:+.1f}) a0/a1/a2=(n/a)",
                flush=True,
            )
        else:
            a0d, a1d, a2d = ang
            print(
                f"[FB] step={self.fb_idx:04d} leg={item} (BL/FL/BR/FR)="
                f"{['FR','BR','BL','FL'][item]} "
                f"swing_tgt=({swing_tgt[0]:+.1f},{swing_tgt[1]:+.1f},{swing_tgt[2]:+.1f}) "
                f"a0={a0d:+.2f} a1={a1d:+.2f} a2={a2d:+.2f}",
                flush=True,
            )

        ok = self._single_leg_step_no_push(item, cmd, swing_scale=FB_SWING_SCALE, support_move=False)
        if not ok:
            self.go_stand(duration=0.3)
            return

    def crawl_step(self, cmd: Cmd):
        """
        현재 명령(cmd)에 따라 crawl 한 스텝 수행.

        이 함수가 전체 제어의 중심이다.
        """
        self._maybe_reset_on_direction_change(cmd)

        # 정지 명령이면 XY는 유지하고 Z만 바닥 높이로 맞춘다.
        if cmd.vx == 0 and cmd.vy == 0 and cmd.wz == 0:
            sx, sy, sz = self.stand
            for leg_id in (0, 1, 2, 3):
                x, y, _ = self.foot[leg_id]
                _ = self.set_pose(leg_id, x, y, sz, duration=0.15)
            return

        self._last_move_cmd = cmd

        # 전진/후진 전용 gait
        if cmd.vx != 0 and cmd.vy == 0 and cmd.wz == 0:
            fb_dir = +1 if cmd.vx > 0 else -1
            if fb_dir != self._fb_dir:
                self._fb_dir = fb_dir
                self.fb_idx = 0
                self.order_idx = 0
                self._crawl_order_key = "fwd" if fb_dir > 0 else "back"

                print(f"[FB] direction change -> {'FWD' if fb_dir > 0 else 'BACK'} : reset idx + go_stand", flush=True)

            self.fb_step(cmd)
            return

        # 일반 crawl 순서 선택
        if cmd.vx < 0:
            order = CRAWL_ORDER_BACK
            key = "back"
        else:
            order = CRAWL_ORDER_FWD
            key = "fwd"

        if key != self._crawl_order_key:
            self._crawl_order_key = key
            self.order_idx = 0

        swing_leg = order[self.order_idx % len(order)]
        self.order_idx += 1

        diag_leg = DIAG_LEG[swing_leg]

        opp_side = OPPOSITE_SIDE_LEGS[swing_leg]
        ctr2_leg = None
        if COUNTER2_ENABLE:
            cand = [lid for lid in opp_side if lid != diag_leg]
            ctr2_leg = cand[0] if cand else None

        dx_ctr_local = +COUNTER_DX
        dy_ctr_local = +COUNTER_DY
        dz_ctr_local = +COUNTER_DZ

        dx_ctr2_local = dx_ctr_local * COUNTER2_SCALE if ctr2_leg is not None else 0.0
        dy_ctr2_local = dy_ctr_local * COUNTER2_SCALE if ctr2_leg is not None else 0.0
        dz_ctr2_local = dz_ctr_local * COUNTER2_SCALE if ctr2_leg is not None else 0.0

        if ctr2_leg is None:
            print(
                f"[COUNTER] swing_leg={swing_leg} diag_leg={diag_leg} "
                f"local_ctr=({dx_ctr_local:+.1f},{dy_ctr_local:+.1f},{dz_ctr_local:+.1f})"
            )
        else:
            print(
                f"[COUNTER] swing_leg={swing_leg} diag_leg={diag_leg} ctr2_leg={ctr2_leg} "
                f"diag=({dx_ctr_local:+.1f},{dy_ctr_local:+.1f},{dz_ctr_local:+.1f}) "
                f"ctr2=({dx_ctr2_local:+.1f},{dy_ctr2_local:+.1f},{dz_ctr2_local:+.1f})"
            )

        sx, sy, sz = self.stand
        z_lift = sz + LIFT_DZ

        # BODY 명령량 계산
        body_dx = cmd.vx * STEP_FWD
        body_dy = cmd.vy * STEP_LAT
        body_dx_yaw = cmd.wz * STEP_YAW

        # swing 다리 기준 local 이동량
        body_dx_swing = body_dx + body_dx_yaw * side_sign(swing_leg)
        dx_leg = body_x_to_local_x(swing_leg, body_dx_swing)
        dy_leg = body_y_to_local_y(swing_leg, body_dy)

        print(
            f"[SWING] leg={swing_leg} body_dx={body_dx_swing:+.1f} body_dy={body_dy:+.1f} "
            f"-> local_dx={dx_leg:+.1f} local_dy={dy_leg:+.1f}"
        )

        support = [i for i in (0, 1, 2, 3) if i != swing_leg]
        body_dy_support = -body_dy / len(support) if support else 0.0

        # 1) SHIFT
        desired_body_shift = +SHIFT_MAG if swing_leg in RIGHT_LEGS else -SHIFT_MAG
        self.shift_body(swing_leg, desired_body_shift, PHASE_T)

        # 1b) COUNTER
        xd, yd, zd = self.foot[diag_leg]
        if not self.set_pose(diag_leg, xd + dx_ctr_local, yd + dy_ctr_local, zd + dz_ctr_local, PHASE_T):
            self.go_stand(duration=0.3)
            return

        if ctr2_leg is not None:
            x2, y2, z2 = self.foot[ctr2_leg]
            if not self.set_pose(ctr2_leg, x2 + dx_ctr2_local, y2 + dy_ctr2_local, z2 + dz_ctr2_local, PHASE_T):
                self.go_stand(duration=0.3)
                return

        # 2) LIFT
        x0, y0, _ = self.foot[swing_leg]
        if not self.set_pose(swing_leg, x0, y0, z_lift, PHASE_T):
            self.go_stand(duration=0.3)
            return

        # 3) SWING
        swing_target = (x0 + dx_leg, y0 + dy_leg, z_lift)

        xt, yt, zt = swing_target
        ang = _try_ik_angles_deg(xt, yt, zt)
        if ang is None:
            print(f"[SWING_TGT] leg={swing_leg} xyz=({xt:+.1f},{yt:+.1f},{zt:+.1f}) a0/a1/a2=(n/a)")
        else:
            a0d, a1d, a2d = ang
            print(
                f"[SWING_TGT] leg={swing_leg} xyz=({xt:+.1f},{yt:+.1f},{zt:+.1f}) "
                f"a0={a0d:+.2f} a1={a1d:+.2f} a2={a2d:+.2f}"
            )

        support_targets = {}
        for leg_id in support:
            xs, ys, zs = self.foot[leg_id]

            body_dx_support = -(body_dx + body_dx_yaw * side_sign(leg_id)) / len(support)
            sup_dx_local = body_x_to_local_x(leg_id, body_dx_support)
            sup_dy_local = body_y_to_local_y(leg_id, body_dy_support)

            support_targets[leg_id] = (xs + sup_dx_local, ys + sup_dy_local, zs)

        steps = max(1, int(PHASE_T / MOVE_DT))
        swing_start = self.foot[swing_leg]
        support_start = {i: self.foot[i] for i in support}

        for s in range(steps + 1):
            u = s / steps
            ue = smoothstep(u)

            # swing 다리
            xS0, yS0, zS0 = swing_start
            xSt, ySt, zSt = swing_target

            xi = lerp(xS0, xSt, ue)
            yi = lerp(yS0, ySt, ue)
            zi = lerp(zS0, zSt, ue)

            if SWING_ARC_DZ and SWING_ARC_DZ > 0:
                zi = zi + SWING_ARC_DZ * math.sin(math.pi * ue)

            if not self._try_set_leg_xyz(swing_leg, xi, yi, zi):
                self.go_stand(duration=0.3)
                return

            # 지지 다리
            for leg_id in support:
                xA0, yA0, zA0 = support_start[leg_id]
                xAt, yAt, zAt = support_targets[leg_id]

                xj = lerp(xA0, xAt, ue)
                yj = lerp(yA0, yAt, ue)
                zj = lerp(zA0, zAt, ue)

                if not self._try_set_leg_xyz(leg_id, xj, yj, zj):
                    self.go_stand(duration=0.3)
                    return

            time.sleep(MOVE_DT)

        self.foot[swing_leg] = swing_target
        for leg_id in support:
            self.foot[leg_id] = support_targets[leg_id]

        # 4) TOUCHDOWN
        x1, y1, _ = self.foot[swing_leg]
        if not self.set_pose(swing_leg, x1, y1, sz, PHASE_T):
            self.go_stand(duration=0.3)
            return

        # 5) UNSHIFT
        steps = max(1, int(PHASE_T / MOVE_DT))
        start = {i: self.foot[i] for i in (0, 1, 2, 3)}

        x_target = {}
        y_target = {}

        for leg_id in (0, 1, 2, 3):
            dy_local_shift = body_y_to_local_y(leg_id, desired_body_shift)
            x_cur, y_cur, _ = self.foot[leg_id]

            y_t = y_cur - dy_local_shift
            x_t = x_cur

            if leg_id == diag_leg:
                x_t = x_t - dx_ctr_local
                y_t = y_t - dy_ctr_local

            if ctr2_leg is not None and leg_id == ctr2_leg:
                x_t = x_t - dx_ctr2_local
                y_t = y_t - dy_ctr2_local

            x_target[leg_id] = x_t
            y_target[leg_id] = y_t

        for s in range(steps + 1):
            u = s / steps
            ue = smoothstep(u)

            for leg_id in (0, 1, 2, 3):
                x0, y0, z0 = start[leg_id]
                xi = lerp(x0, x_target[leg_id], ue)
                yi = lerp(y0, y_target[leg_id], ue)
                zi = lerp(z0, sz, ue)

                if not self._try_set_leg_xyz(leg_id, xi, yi, zi):
                    self.go_stand(duration=0.3)
                    return

            time.sleep(MOVE_DT)

        for leg_id in (0, 1, 2, 3):
            self.foot[leg_id] = (x_target[leg_id], y_target[leg_id], sz)

    def reset(self):
        """
        GPIO/보드 리셋.
        현재는 리셋 후 15초 대기.
        하드웨어 초기화 시간 확보 목적.
        """
        self.api.leg_reset()
        time.sleep(15)

    def shutdown(self):
        """
        종료 시 안전하게 센터 포즈로 이동.
        """
        self.api.go_center_pose(debug=True)

    def _maybe_reset_on_direction_change(self, cmd: Cmd):
        """
        이동 방향이 바뀌었을 때
        순서 인덱스를 초기화하고 stand 자세로 정렬한다.

        이유:
        - crawl은 순서 기반 보행이라
          중간 단계에서 방향이 바뀌면 꼬일 수 있다.
        """
        prev = self._last_move_cmd

        if (prev.vx, prev.vy, prev.wz) == (0, 0, 0):
            return
        if (cmd.vx, cmd.vy, cmd.wz) == (0, 0, 0):
            return
        if (cmd.vx, cmd.vy, cmd.wz) == (prev.vx, prev.vy, prev.wz):
            return

        self.fb_idx = 0
        self.order_idx = 0
        self._fb_dir = 0
        self._crawl_order_key = "fwd"

        print(
            f"[RESET] cmd change {prev.vx:+d},{prev.vy:+d},{prev.wz:+d} -> "
            f"{cmd.vx:+d},{cmd.vy:+d},{cmd.wz:+d} : go_stand + reset idx",
            flush=True,
        )
        self.go_stand(duration=0.25)


# ============================================================
# 6. 메인 함수
# ============================================================

def main():
    """
    프로그램 시작점.

    동작 순서:
      1) CrawlDriver 생성
      2) 하드웨어 reset
      3) stand 자세로 이동
      4) 키 입력을 계속 읽으면서 crawl_step 실행
      5) Ctrl+C 시 종료 자세로 복귀
    """
    drv = CrawlDriver()
    drv.reset()

    print("[INFO] Stand pose:", STAND_XYZ)
    print("[INFO] Keys: W/S/A/D/Q/E, other = stop. Ctrl+C to exit.")

    drv.go_stand(duration=0.6)

    cmd = Cmd(0, 0, 0)
    last_key_t = 0.0

    with KeyReader() as kr:
        try:
            while True:
                # 현재 버퍼에 쌓인 키 중 마지막 키만 사용
                k = kr.drain_last_key()
                now = time.time()

                if k is not None:
                    cmd = key_to_cmd(k)
                    last_key_t = now

                # 일정 시간 입력이 없으면 자동 정지
                if (now - last_key_t) > IDLE_HOLD:
                    cmd = Cmd(0, 0, 0)

                # 현재 명령 기준으로 한 스텝 수행
                drv.crawl_step(cmd)

        except KeyboardInterrupt:
            print("\n[CTRL+C] Exit")

        finally:
            drv.shutdown()


if __name__ == "__main__":
    main()