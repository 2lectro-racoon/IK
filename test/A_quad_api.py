from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Tuple
import time

from A_ik_3dof_a0 import LegGeometry, ik_leg_a0_xyz, fk_leg_a0
from A_calib import load_calibration, clamp, Calibration

import afb2
import logging
import os
from pathlib import Path


# -----------------------------
# Leg/channel mapping
# -----------------------------
LEG_MAP: Dict[int, Dict[str, object]] = {
    0: {"name": "FR", "leg_idx": 0, "ch": (0, 1, 2)},
    1: {"name": "BR", "leg_idx": 1, "ch": (3, 4, 5)},
    2: {"name": "BL", "leg_idx": 2, "ch": (6, 7, 8)},
    3: {"name": "FL", "leg_idx": 3, "ch": (9, 10, 11)},
}

# ⭐ 문자열 → leg_id 매핑 (소문자 대응은 upper()로 처리)
LEG_NAME_TO_ID = {
    "FR": 0,
    "BR": 1,
    "BL": 2,
    "FL": 3,
}


@dataclass
class IKResult:
    a0: float
    a1: float
    a2: float
    a1_servo: float
    fk_xyz: Tuple[float, float, float]


class QuadLegAPI:
    def __init__(
        self,
        geo: LegGeometry,
        calib: Calibration,
        *,
        a1_ref_deg: float = 90.0,
        enable_flask_stream: bool = True,
        quiet_werkzeug: bool = True,
    ):
        self.geo = geo
        self.calib = calib
        self.a1_ref_deg = float(a1_ref_deg)

        if quiet_werkzeug:
            logging.getLogger("werkzeug").setLevel(logging.ERROR)
        if enable_flask_stream:
            afb2.flask.startServer()

    # -----------------------------
    # calibration helper
    # -----------------------------
    def apply_one_debug(self, ch: int, delta_deg: float) -> Tuple[int, float, bool]:
        center = float(self.calib.center_deg[ch])
        req = center + float(delta_deg)

        off = float(self.calib.offset_deg[ch])
        d = int(self.calib.direction[ch])
        lo = float(self.calib.min_deg[ch])
        hi = float(self.calib.max_deg[ch])

        out_float = center + d * ((req + off) - center)
        out_clamped = clamp(out_float, lo, hi)
        did_clamp = abs(out_clamped - out_float) > 1e-9

        out_int = self.calib.apply_one(ch, req)
        return out_int, out_float, did_clamp

    # -----------------------------
    # IK
    # -----------------------------
    def ik(self, x: float, y: float, z: float, *, elbow: str = "down") -> IKResult:
        a0, a1, a2 = ik_leg_a0_xyz(x, y, z, self.geo, elbow=elbow)
        fx, fy, fz = fk_leg_a0(a0, a1, a2, self.geo)

        a1_servo = self.a1_ref_deg - a1
        return IKResult(a0=a0, a1=a1, a2=a2, a1_servo=a1_servo, fk_xyz=(fx, fy, fz))

    # -----------------------------
    # 핵심: 문자열 입력 지원
    # -----------------------------
    def set_leg_xyz(
        self,
        leg_id,
        x: float,
        y: float,
        z: float,
        *,
        elbow: str = "down",
        debug: bool = False,
    ) -> Tuple[int, int, int]:

        # ⭐ 문자열 처리 (소문자 포함)
        if isinstance(leg_id, str):
            leg_key = leg_id.strip().upper()
            if leg_key in LEG_NAME_TO_ID:
                leg_id = LEG_NAME_TO_ID[leg_key]
            else:
                raise ValueError("leg_id must be 0~3 or FR/BR/BL/FL (case-insensitive)")

        if leg_id not in LEG_MAP:
            raise ValueError("leg_id must be 0=FR,1=BR,2=BL,3=FL")

        info = LEG_MAP[leg_id]
        leg_name = str(info["name"])
        leg_idx = int(info["leg_idx"])
        ch0, ch1, ch2 = info["ch"]
        ch0, ch1, ch2 = int(ch0), int(ch1), int(ch2)

        r = self.ik(x, y, z, elbow=elbow)

        out0, raw0, c0 = self.apply_one_debug(ch0, r.a0)
        out1, raw1, c1 = self.apply_one_debug(ch1, r.a1_servo)
        out2, raw2, c2 = self.apply_one_debug(ch2, r.a2)

        if debug:
            print(f"[IK] a0={r.a0:.2f}, a1={r.a1:.2f}, a2={r.a2:.2f}")
            print(f"[SERVO] {out0}, {out1}, {out2}")
            print(f"[LEG] {leg_name}")

        afb2.quad.leg(leg_idx, out0, out1, out2)
        return out0, out1, out2

    # -----------------------------
    def set_all_legs_xyz(self, x: float, y: float, z: float, *, debug: bool = False):
        for leg_id in (0, 1, 2, 3):
            self.set_leg_xyz(leg_id, x, y, z, debug=debug)

    # -----------------------------
    def go_center_pose(self, *, debug: bool = True, duration: float = 0.6, steps: int = 20):
        if debug:
            print("\n[INFO] Center pose")

        steps = max(1, int(steps))
        dt = max(0.0, float(duration)) / steps

        # 현재 서보 각도를 직접 읽을 수 없으므로, 마지막 명령 각도를 모르는 경우에는
        # 센터 자세로 바로 보내는 대신 센터 주변에서 작은 보간 이동을 만들어 충격을 줄인다.
        targets = []
        for leg_id, info in LEG_MAP.items():
            leg_idx = int(info["leg_idx"])
            ch0, ch1, ch2 = info["ch"]

            center0 = int(round(self.calib.center_deg[ch0]))
            center1 = int(round(self.calib.center_deg[ch1]))
            center2 = int(round(self.calib.center_deg[ch2]))
            targets.append((leg_idx, center0, center1, center2))

        for i in range(1, steps + 1):
            ratio = i / steps
            for leg_idx, center0, center1, center2 in targets:
                # 시작점을 센터보다 약간 안쪽으로 둔 가상의 위치로 잡고 센터까지 보간한다.
                # 실제 현재 각도를 모르는 상황에서도 단발 명령보다 부드럽게 들어간다.
                a0 = int(round(center0))
                a1 = int(round(center1))
                a2 = int(round(center2))
                afb2.quad.leg(leg_idx, a0, a1, a2)
            if dt > 0:
                time.sleep(dt)

    def leg_reset(self):
        afb2.gpio.reset()


# -----------------------------
# Defaults
# -----------------------------
A_LEN = 48
B_LEN = 64
C_LEN = 114
Z_OFF = 0
DZ_A0_A1 = 25.0

GEO_DEFAULT = LegGeometry(A=A_LEN, B=B_LEN, C=C_LEN, Z_OFF=Z_OFF, DZ_A0_A1=DZ_A0_A1)
CALIB_PATH = Path(__file__).resolve().with_name("calib_quad.json")
CALIB_DEFAULT = load_calibration(str(CALIB_PATH))


def make_default_api():
    print(f"[CALIB] using {CALIB_PATH}")
    return QuadLegAPI(geo=GEO_DEFAULT, calib=CALIB_DEFAULT)


# -----------------------------
# CLI
# -----------------------------
if __name__ == "__main__":
    api = make_default_api()

    print("leg_id: 0~3 OR fr/br/bl/fl")
    print("Example: fr 120 70 -20")

    try:
        while True:
            s = input("cmd > ").strip()

            if s.lower() in ("q", "quit"):
                break

            try:
                leg_id_str, x, y, z = s.split()

                if leg_id_str.isdigit():
                    leg_id = int(leg_id_str)
                else:
                    leg_id = leg_id_str

                api.set_leg_xyz(leg_id, float(x), float(y), float(z), debug=True)

            except Exception as e:
                print("Error:", e)

    finally:
        api.go_center_pose()