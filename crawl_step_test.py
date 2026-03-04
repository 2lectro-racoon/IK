# crawl_step_test.py
# One-step crawl test (no keyboard yet)
# Uses quad_api.py (do not modify quad_api.py)

from __future__ import annotations

import time

from quad_api import make_default_api

# -------------------------
# Tunable parameters
# -------------------------
STAND_XYZ = (120.0, 70.0, -50.0)   # (x, y, z) in each leg's local A0 frame
STEP_LEN = 30.0                   # mm (forward for the swinging leg in its local +x)
LIFT_DZ = 25.0                    # mm (since smaller z lifts => z_lift = z_stand - LIFT_DZ)
SHIFT_Y = -15.0                    # mm (shift body to reduce tipping when lifting one leg)
MOVE_DT = 0.04                    # seconds per interpolation step (25 Hz)
MOVE_T = 0.6                      # seconds for each phase (shift/lift/swing/down/unshift)

# Leg order for crawl (we start with FR only for this test)
SWING_LEG_ID = 0  # 0=FR,1=BR,2=BL,3=FL


# -------------------------
# Helpers
# -------------------------
def lerp(a: float, b: float, t: float) -> float:
    return a + (b - a) * t


def move_all(api, x: float, y: float, z: float, duration: float, *, debug: bool = False):
    """Smoothly move all legs to the same (x,y,z) in their local frames."""
    x0, y0, z0 = STAND_XYZ
    steps = max(1, int(duration / MOVE_DT))
    for i in range(steps + 1):
        u = i / steps
        xi = lerp(x0, x, u)
        yi = lerp(y0, y, u)
        zi = lerp(z0, z, u)
        api.set_all_legs_xyz(xi, yi, zi, debug=debug)
        time.sleep(MOVE_DT)


def move_leg(api, leg_id: int, x: float, y: float, z: float, duration: float, *, debug: bool = False):
    """Smoothly move one leg to (x,y,z). Other legs stay at stand."""
    sx, sy, sz = STAND_XYZ
    steps = max(1, int(duration / MOVE_DT))
    for i in range(steps + 1):
        u = i / steps
        xi = lerp(sx, x, u)
        yi = lerp(sy, y, u)
        zi = lerp(sz, z, u)
        api.set_leg_xyz(leg_id, xi, yi, zi, debug=debug)
        time.sleep(MOVE_DT)


def do_shift(api, swing_leg_id: int, shift_y: float, duration: float):
    """Shift body away from the swing leg to keep COM inside support triangle.

    Simple heuristic:
      - If swing leg is on the RIGHT side (FR=0, BR=1), shift LEFT => increase left legs y, decrease right legs y
      - If swing leg is on the LEFT side  (BL=2, FL=3), shift RIGHT => increase right legs y, decrease left legs y

    We keep x,z at stand.
    """
    sx, sy, sz = STAND_XYZ

    right_legs = {0, 1}
    left_legs = {2, 3}

    if swing_leg_id in right_legs:
        # shift to LEFT: left legs go more outward (+y), right legs go slightly inward (-y)
        targets = {
            0: (sx, sy - shift_y, sz),
            1: (sx, sy - shift_y, sz),
            2: (sx, sy + shift_y, sz),
            3: (sx, sy + shift_y, sz),
        }
    else:
        # shift to RIGHT
        targets = {
            0: (sx, sy + shift_y, sz),
            1: (sx, sy + shift_y, sz),
            2: (sx, sy - shift_y, sz),
            3: (sx, sy - shift_y, sz),
        }

    steps = max(1, int(duration / MOVE_DT))
    for i in range(steps + 1):
        u = i / steps
        for leg_id, (tx, ty, tz) in targets.items():
            xi = lerp(sx, tx, u)
            yi = lerp(sy, ty, u)
            zi = lerp(sz, tz, u)
            api.set_leg_xyz(leg_id, xi, yi, zi, debug=False)
        time.sleep(MOVE_DT)


def unshift(api, swing_leg_id: int, shift_y: float, duration: float):
    """Return y shift back to stand."""
    # Just call do_shift in reverse by swapping swing leg side (hack-free way is to interpolate to STAND_XYZ)
    sx, sy, sz = STAND_XYZ
    steps = max(1, int(duration / MOVE_DT))
    for i in range(steps + 1):
        u = i / steps
        # Interpolate each leg's y back to sy; x,z are already sx,sz
        for leg_id in (0, 1, 2, 3):
            # Read the intended shifted target same as in do_shift
            # We'll recompute once and interpolate from target->stand by flipping u.
            pass
        time.sleep(MOVE_DT)

    # Simpler + robust: directly move all legs back to stand smoothly
    move_all(api, sx, sy, sz, duration, debug=False)


def one_step_crawl(api, leg_id: int):
    """Do one crawl step for a single leg:
    shift -> lift -> swing forward -> touchdown -> unshift
    """
    sx, sy, sz = STAND_XYZ
    z_lift = sz + LIFT_DZ          # smaller z lifts (your system)
    x_fwd = sx + STEP_LEN

    print(f"[STEP] stand={STAND_XYZ}  lift_z={z_lift}  step_len={STEP_LEN}  shift_y={SHIFT_Y}")
    print(f"[STEP] swing_leg={leg_id}")

    # 1) Start from stand pose
    api.set_all_legs_xyz(sx, sy, sz, debug=False)
    time.sleep(0.3)

    # 2) SHIFT (reduce tipping)
    print("[PHASE] shift")
    do_shift(api, leg_id, SHIFT_Y, MOVE_T)
    time.sleep(2)
    # 3) LIFT swing leg
    print("[PHASE] lift")
    move_leg(api, leg_id, sx, sy, z_lift, MOVE_T, debug=False)

    # 4) SWING forward (while lifted)
    print("[PHASE] swing")
    # keep lifted z during swing
    steps = max(1, int(MOVE_T / MOVE_DT))
    for i in range(steps + 1):
        u = i / steps
        xi = lerp(sx, x_fwd, u)
        api.set_leg_xyz(leg_id, xi, sy, z_lift, debug=False)
        time.sleep(MOVE_DT)

    # 5) TOUCHDOWN
    print("[PHASE] touchdown")
    move_leg(api, leg_id, x_fwd, sy, sz, MOVE_T, debug=False)

    # 6) UNSHIFT back to stand
    print("[PHASE] unshift")
    move_all(api, sx, sy, sz, MOVE_T, debug=False)

    print("[DONE] one step complete")


def main():
    api = make_default_api()

    # Safety: go to stand first
    print("[INIT] stand pose")
    api.set_all_legs_xyz(*STAND_XYZ, debug=False)
    time.sleep(0.5)

    # One-step test (FR only)
    one_step_crawl(api, SWING_LEG_ID)

    # Leave in stand at end
    print("[END] stand pose")
    api.set_all_legs_xyz(*STAND_XYZ, debug=False)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n[CTRL+C] KeyboardInterrupt")
    finally:
        # Always return to calibration centers on exit
        try:
            api = make_default_api()
            api.go_center_pose(debug=True)
        except Exception as e:
            print("[WARN] failed to return to center pose:", e)