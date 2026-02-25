# test_fr_ik.py
from ik_3dof import LegGeometry, ik_leg_xyz
from calib import load_calibration
import afb  # 너 환경에 맞게 import (같은 폴더면 OK)
import logging

logging.getLogger('werkzeug').setLevel(logging.ERROR)
afb.flask.startServer()

# 1) 기구 파라미터
# geo = LegGeometry(A=64.8, B=112.0, C=46.5, Z_OFF=10.0)
# geo = LegGeometry(A=0, B=112.0, C=46.5, Z_OFF=10.0)
# geo = LegGeometry(A=48, B=64, C=112, Z_OFF=10.0)
# 1) 기구 파라미터 (mm)
# ※ 여기 값만 바꿔가며 IK를 맞추면 됩니다.
A_LEN = 64.8      # A0(yaw) 축 -> A1(pitch) 축의 XY 평면 거리
B_LEN = 112.0     # 허벅지(hip pitch -> knee) 길이
C_LEN = 46.5      # 종아리(knee -> foot) 길이
Z_OFF = 10.0      # 기계적 높이 보정 (mm)
DZ_A0_A1 = 25.0   # A1이 A0보다 위(+Z)로 25mm면 +25.0 (A0 기준 입력일 때만 의미 있음)

geo = LegGeometry(A=A_LEN, B=B_LEN, C=C_LEN, Z_OFF=Z_OFF, DZ_A0_A1=DZ_A0_A1)
print(f"[Geometry] A={A_LEN} B={B_LEN} C={C_LEN} Z_OFF={Z_OFF} DZ_A0_A1={DZ_A0_A1}")

# 2) 캘리브레이션 로드

calib = load_calibration("calib_quad.json")

# Convert logical IK angle to actual servo angle (with clamp debug)
def servo_from_logical(calib, ch, logical_deg):
    center = float(calib.center_deg[ch])
    direction = int(calib.direction[ch])
    offset = float(calib.offset_deg[ch])
    lo = float(calib.min_deg[ch])
    hi = float(calib.max_deg[ch])

    raw = center + direction * float(logical_deg) + offset
    clamped = max(lo, min(hi, raw))
    did_clamp = abs(clamped - raw) > 1e-9
    return int(round(clamped)), raw, did_clamp
# 3) FR 다리(leg_idx=0), 채널 0,1,2
LEG_IDX = 0
CH0, CH1, CH2 = 0, 1, 2

# --- Calibration debug helper and snapshot printout ---
def _print_ch_calib(ch: int) -> None:
    print(
        f"calib ch{ch}: center={calib.center_deg[ch]:.2f}, dir={calib.direction[ch]}, "
        f"offset={calib.offset_deg[ch]:.2f}, lim=[{calib.min_deg[ch]:.2f},{calib.max_deg[ch]:.2f}]"
    )

print("\n[Calibration snapshot]")
_print_ch_calib(CH0)
_print_ch_calib(CH1)
_print_ch_calib(CH2)
print("")

# 4) Interactive XYZ input loop
print("Enter x y z (mm). Example: 120 60 -80")
print("Type 'q' to quit.\n")

while True:
    user_input = input("xyz > ").strip()

    if user_input.lower() in ("q", "quit", "exit"):
        print("Exiting...")
        break

    try:
        x_str, y_str, z_str = user_input.split()
        x = float(x_str)
        y = float(y_str)
        z = float(z_str)
    except ValueError:
        print("Invalid input. Please enter three numbers separated by spaces.")
        continue

    try:
        # 5) IK 계산 (논리각)
        a0, a1, a2 = ik_leg_xyz(x, y, z, geo, elbow="down")
        # --- A1 remap (servo-friendly) ---
        # IK a1 is referenced to the mathematical triangle solution and can be negative.
        # Your servo channel CH1 is limited to 0..180 and is currently clamping at 0.
        # Remap so that typical standing poses produce positive angles.
        A1_REF = 90.0
        a1 = A1_REF - a1

        # --- A2 remap (knee bend) ---
        # ik_3dof returns the knee *internal angle* (0..180).
        # For most quadruped joints, we want a2 to increase when the knee bends (folds).
        # Bending corresponds to a smaller internal angle, so remap to a "bend" angle:
        #   a2_bend = 180 - a2_internal
        a2 = 180.0 - a2

        print(f"IK (logical deg): {a0:.2f}, {a1:.2f}, {a2:.2f}  (A1 remap: A1_REF=90, A2 remap: 180-a2)")

        # 6) 논리각 → 실제 서보각 변환 후 전송 (show raw/clamp)
        out0, raw0, c0 = servo_from_logical(calib, CH0, a0)
        out1, raw1, c1 = servo_from_logical(calib, CH1, a1)
        out2, raw2, c2 = servo_from_logical(calib, CH2, a2)

        clamp_note = []
        if c0: clamp_note.append("CH0 clamped")
        if c1: clamp_note.append("CH1 clamped")
        if c2: clamp_note.append("CH2 clamped")
        clamp_note = (" | " + ", ".join(clamp_note)) if clamp_note else ""

        print(
            f"SERVO(raw->clamp): "
            f"ch0 {raw0:.2f}->{out0}  "
            f"ch1 {raw1:.2f}->{out1}  "
            f"ch2 {raw2:.2f}->{out2}" + clamp_note
        )

        # leg()는 다리 단위로 3개를 한 번에 보냄
        afb.quad.leg(LEG_IDX, out0, out1, out2)

    except Exception as e:
        print(f"Error: {e}")
        continue