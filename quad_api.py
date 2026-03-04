# quad_api.py
# Core control pipeline (IK -> remap -> calibration -> STM32 servo command)
# Designed to be imported by gait scripts (crawl/trot/etc.)

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Tuple

from ik_3dof_a0 import LegGeometry, ik_leg_a0_xyz, fk_leg_a0
from calib import load_calibration, clamp, Calibration

import afb
import logging
import os


# -----------------------------
# Leg/channel mapping
# -----------------------------
LEG_MAP: Dict[int, Dict[str, object]] = {
    0: {"name": "FR", "leg_idx": 0, "ch": (0, 1, 2)},
    1: {"name": "BR", "leg_idx": 1, "ch": (3, 4, 5)},
    2: {"name": "BL", "leg_idx": 2, "ch": (6, 7, 8)},
    3: {"name": "FL", "leg_idx": 3, "ch": (9, 10, 11)},
}


@dataclass
class IKResult:
    a0: float
    a1: float
    a2: float
    a1_servo: float
    fk_xyz: Tuple[float, float, float]


class QuadLegAPI:
    """Low-level control API: (leg_id, x,y,z) -> IK -> calib -> send to STM32.

    Input coordinates are in each leg's A0 local frame.
    Calibration behavior matches the calibration tool (Calibration.apply_one):

      req = center + delta
      out = calib.apply_one(ch, req)

    Notes:
      - This does NOT implement gait (crawl/trot/etc).
      - Your current system: 'smaller z lifts the leg' (as you observed).
    """

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
            afb.flask.startServer()

    # ----- calibration compatible helper -----
    def apply_one_debug(self, ch: int, delta_deg: float) -> Tuple[int, float, bool]:
        """Return (out_int, out_float_before_clamp, did_clamp).

        delta_deg is interpreted as a delta around channel center.
        """
        center = float(self.calib.center_deg[ch])
        req = center + float(delta_deg)

        off = float(self.calib.offset_deg[ch])
        d = int(self.calib.direction[ch])
        lo = float(self.calib.min_deg[ch])
        hi = float(self.calib.max_deg[ch])

        # Match Calibration.apply_one(): (req + off) then mirror around center.
        out_float = center + d * ((req + off) - center)
        out_clamped = clamp(out_float, lo, hi)
        did_clamp = abs(out_clamped - out_float) > 1e-9

        out_int = self.calib.apply_one(ch, req)
        return out_int, out_float, did_clamp

    # ----- IK / FK -----
    def ik(self, x: float, y: float, z: float, *, elbow: str = "down") -> IKResult:
        a0, a1, a2 = ik_leg_a0_xyz(x, y, z, self.geo, elbow=elbow)
        fx, fy, fz = fk_leg_a0(a0, a1, a2, self.geo)

        # Servo-friendly remap for hip pitch
        a1_servo = self.a1_ref_deg - a1
        return IKResult(a0=a0, a1=a1, a2=a2, a1_servo=a1_servo, fk_xyz=(fx, fy, fz))

    # ----- command layer -----
    def set_leg_xyz(
        self,
        leg_id: int,
        x: float,
        y: float,
        z: float,
        *,
        elbow: str = "down",
        debug: bool = False,
    ) -> Tuple[int, int, int]:
        """Compute IK and send one leg command.

        Returns (out0,out1,out2) integer degrees actually sent.
        """
        if leg_id not in LEG_MAP:
            raise ValueError("leg_id must be 0=FR,1=BR,2=BL,3=FL")

        info = LEG_MAP[leg_id]
        leg_name = str(info["name"])
        leg_idx = int(info["leg_idx"])
        ch0, ch1, ch2 = info["ch"]  # type: ignore
        ch0, ch1, ch2 = int(ch0), int(ch1), int(ch2)

        r = self.ik(x, y, z, elbow=elbow)

        out0, raw0, c0 = self.apply_one_debug(ch0, r.a0)
        out1, raw1, c1 = self.apply_one_debug(ch1, r.a1_servo)
        out2, raw2, c2 = self.apply_one_debug(ch2, r.a2)

        if debug:
            print(f"IK deg: a0={r.a0:.2f}, a1={r.a1:.2f}, a2(bend)={r.a2:.2f}")
            fx, fy, fz = r.fk_xyz
            print(f"FK xyz: ({fx:.1f}, {fy:.1f}, {fz:.1f})  target=({x:.1f},{y:.1f},{z:.1f})")
            print(f"A1 remap: a1={r.a1:.2f} -> a1_servo={r.a1_servo:.2f} (A1_REF={self.a1_ref_deg:.1f})")

            clamp_note = []
            if c0: clamp_note.append("CH0 clamped")
            if c1: clamp_note.append("CH1 clamped")
            if c2: clamp_note.append("CH2 clamped")
            clamp_note_s = (" | " + ", ".join(clamp_note)) if clamp_note else ""

            print(
                f"SERVO(calib_out->int): "
                f"ch{ch0} {raw0:.2f}->{out0}  "
                f"ch{ch1} {raw1:.2f}->{out1}  "
                f"ch{ch2} {raw2:.2f}->{out2}"
                f"{clamp_note_s}\n"
            )
            print(f"[LEG] {leg_name} (leg_id={leg_id}, leg_idx={leg_idx}) -> channels {ch0},{ch1},{ch2}")

        afb.quad.leg(leg_idx, out0, out1, out2)
        return out0, out1, out2

    def set_all_legs_xyz(self, x: float, y: float, z: float, *, debug: bool = False) -> None:
        """Convenience: set same (x,y,z) to all legs (each leg's local frame)."""
        for leg_id in (0, 1, 2, 3):
            self.set_leg_xyz(leg_id, x, y, z, debug=debug)

    def go_center_pose(self, *, debug: bool = True) -> None:
        """Move all legs to calibration centers."""
        if debug:
            print("\n[INFO] Returning all legs to calibration center pose...")

        for leg_id, info in LEG_MAP.items():
            leg_name = str(info["name"])
            leg_idx = int(info["leg_idx"])
            ch0, ch1, ch2 = info["ch"]  # type: ignore
            ch0, ch1, ch2 = int(ch0), int(ch1), int(ch2)

            center0 = int(round(self.calib.center_deg[ch0]))
            center1 = int(round(self.calib.center_deg[ch1]))
            center2 = int(round(self.calib.center_deg[ch2]))

            if debug:
                print(f"[CENTER] {leg_name} -> ch{ch0}={center0}, ch{ch1}={center1}, ch{ch2}={center2}")
            afb.quad.leg(leg_idx, center0, center1, center2)

        if debug:
            print("[INFO] Done.\n")


# -----------------------------
# Defaults (edit if needed)
# -----------------------------
A_LEN = 48
B_LEN = 64
C_LEN = 114
Z_OFF = 0
DZ_A0_A1 = 25.0

GEO_DEFAULT = LegGeometry(A=A_LEN, B=B_LEN, C=C_LEN, Z_OFF=Z_OFF, DZ_A0_A1=DZ_A0_A1)
CALIB_DEFAULT = load_calibration("calib_quad.json")


def make_default_api() -> QuadLegAPI:
    return QuadLegAPI(geo=GEO_DEFAULT, calib=CALIB_DEFAULT, a1_ref_deg=90.0)


# -----------------------------
# Optional CLI test (you can delete this section if you want)
# -----------------------------
if __name__ == "__main__":
    print(f"[RUNNING] {__file__}")
    print(f"[CWD] {os.getcwd()}")
    print(f"[Geometry] A={A_LEN} B={B_LEN} C={C_LEN} Z_OFF={Z_OFF} DZ_A0_A1={DZ_A0_A1}")

    api = make_default_api()

    print("Enter: leg_id x y z (mm) relative to that leg's A0 axis")
    print("leg_id: 0=FR, 1=BR, 2=BL, 3=FL")
    print("Example: 0 120 70 -20")
    print("Type 'q' to quit.\n")

    try:
        while True:
            s = input("ch xyz > ").strip()
            if s.lower() in ("q", "quit", "exit"):
                api.go_center_pose(debug=True)
                break

            try:
                leg_id_str, x_str, y_str, z_str = s.split()
                api.set_leg_xyz(int(leg_id_str), float(x_str), float(y_str), float(z_str), debug=True)
            except ValueError:
                print("Invalid input. Example: 0 120 70 -20")
            except Exception as e:
                print("Error:", e)

    except KeyboardInterrupt:
        print("\n[CTRL+C] KeyboardInterrupt detected.")
    finally:
        api.go_center_pose(debug=True)