# test_fr_ik2.py
from ik_3dof_a0 import LegGeometry, ik_leg_a0_xyz, fk_leg_a0
from calib import load_calibration
import afb
import logging
import os

print(f"[RUNNING] {__file__}")
print(f"[CWD] {os.getcwd()}")

logging.getLogger("werkzeug").setLevel(logging.ERROR)
afb.flask.startServer()

# ---- geometry (EDIT HERE) ----
A_LEN = 48
B_LEN = 64
C_LEN = 114
Z_OFF = 0
DZ_A0_A1 = 25.0

geo = LegGeometry(A=A_LEN, B=B_LEN, C=C_LEN, Z_OFF=Z_OFF, DZ_A0_A1=DZ_A0_A1)
print(f"[Geometry] A={A_LEN} B={B_LEN} C={C_LEN} Z_OFF={Z_OFF} DZ_A0_A1={DZ_A0_A1}")

calib = load_calibration("calib_quad.json")

LEG_IDX = 0
CH0, CH1, CH2 = 0, 1, 2

def servo_from_logical(ch: int, logical_deg: float):
    """Return (servo_deg_int, raw_deg_float, did_clamp)."""
    center = float(calib.center_deg[ch])
    direction = int(calib.direction[ch])
    offset = float(calib.offset_deg[ch])
    lo = float(calib.min_deg[ch])
    hi = float(calib.max_deg[ch])

    raw = center + direction * float(logical_deg) + offset
    clamped = max(lo, min(hi, raw))
    did_clamp = abs(clamped - raw) > 1e-9
    return int(round(clamped)), raw, did_clamp

print("Enter: leg_id x y z (mm) relative to that leg's A0 axis")
print("leg_id: 0=FR, 1=BR, 2=BL, 3=FL")
print("Example: 0 120 60 -80")
print("Type 'q' to quit.\n")

LEG_MAP = {
    0: {"name": "FR", "leg_idx": 0, "ch": (0, 1, 2)},
    1: {"name": "BR", "leg_idx": 1, "ch": (3, 4, 5)},
    2: {"name": "BL", "leg_idx": 2, "ch": (6, 7, 8)},
    3: {"name": "FL", "leg_idx": 3, "ch": (9, 10, 11)},
}

while True:
    s = input("ch xyz > ").strip()
    if s.lower() in ("q", "quit", "exit"):
        break

    try:
        leg_id_str, x_str, y_str, z_str = s.split()
        leg_id = int(leg_id_str)
        x = float(x_str)
        y = float(y_str)
        z = float(z_str)
    except ValueError:
        print("Invalid input. Example: 0 120 60 -80")
        continue

    if leg_id not in LEG_MAP:
        print("Invalid leg_id. Use 0=FR, 1=BR, 2=BL, 3=FL")
        continue

    leg_name = LEG_MAP[leg_id]["name"]
    leg_idx = LEG_MAP[leg_id]["leg_idx"]
    ch0, ch1, ch2 = LEG_MAP[leg_id]["ch"]

    a0, a1, a2 = ik_leg_a0_xyz(x, y, z, geo, elbow="down")
    print(f"IK deg: a0={a0:.2f}, a1={a1:.2f}, a2(bend)={a2:.2f}")

    fx, fy, fz = fk_leg_a0(a0, a1, a2, geo)
    print(f"FK xyz: ({fx:.1f}, {fy:.1f}, {fz:.1f})  target=({x:.1f},{y:.1f},{z:.1f})")

    # 핵심: CH1이 0으로 클램프되는 문제를 막기 위한 리맵
    A1_REF = 90.0
    a1_servo = A1_REF - a1
    print(f"A1 remap: a1={a1:.2f} -> a1_servo={a1_servo:.2f}")

    out0, raw0, c0 = servo_from_logical(ch0, a0)
    out1, raw1, c1 = servo_from_logical(ch1, a1_servo)
    out2, raw2, c2 = servo_from_logical(ch2, a2)

    clamp_note = []
    if c0: clamp_note.append("CH0 clamped")
    if c1: clamp_note.append("CH1 clamped")
    if c2: clamp_note.append("CH2 clamped")
    clamp_note = (" | " + ", ".join(clamp_note)) if clamp_note else ""

    print(
        f"SERVO(raw->out): "
        f"ch0 {raw0:.2f}->{out0}  "
        f"ch1 {raw1:.2f}->{out1}  "
        f"ch2 {raw2:.2f}->{out2}"
        f"{clamp_note}\n"
    )

    print(f"[LEG] {leg_name} (leg_id={leg_id}, leg_idx={leg_idx}) -> channels {ch0},{ch1},{ch2}")
    afb.quad.leg(leg_idx, out0, out1, out2)