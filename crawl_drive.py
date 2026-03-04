# crawl_drive.py
# Crawl gait driver with WASD/QE keyboard control (single file)
#
# Key map:
#   w: forward, s: backward
#   a: left,    d: right
#   q: CCW yaw, e: CW yaw
#   anything else: stop
#
# Notes:
# - Uses quad_api.make_default_api() (do not modify quad_api.py)
# - Uses a simple crawl cycle with shift->lift->swing->touchdown->unshift
# - z direction: in your robot, "smaller z lifts" (you already confirmed)
#
# Run:
#   python3 crawl_drive.py

from __future__ import annotations

import sys
import time
import math
import select
import termios
import tty
from dataclasses import dataclass
from typing import Dict, Tuple

from quad_api import make_default_api


# -------------------------
# Tunable parameters
# -------------------------
STAND_XYZ = (120.0, 70.0, -50.0)   # your current stable stand
LIFT_DZ = 60.0                     # lift: z_lift = z_stand - LIFT_DZ (smaller z lifts)
SHIFT_MAG = 20.0                   # magnitude of lateral shift (mm) before lifting
STEP_FWD = 25.0                    # mm per step for forward/back command
STEP_LAT = 20.0                    # mm per step for left/right command
STEP_YAW = 20.0                    # mm per step for yaw command (as differential dx between sides)

# Timing
MOVE_DT = 0.04                     # 25 Hz interpolation
PHASE_T = 0.45                     # seconds for each phase (shift/lift/swing/down/unshift)
IDLE_HOLD = 0.35                   # if no key within this time -> cmd becomes 0 (stop)

# Crawl order (stable): FR -> BL -> FL -> BR
CRAWL_ORDER = [0, 2, 3, 1]         # 0=FR,1=BR,2=BL,3=FL

# Leg side sets (for mapping body-left to local y)
RIGHT_LEGS = {0, 1}                # FR, BR
LEFT_LEGS = {2, 3}                 # BL, FL


# -------------------------
# Small helpers
# -------------------------
def lerp(a: float, b: float, t: float) -> float:
    return a + (b - a) * t


def clampf(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


def body_y_to_local_y(leg_id: int, body_y: float) -> float:
    """Convert body-frame +Y(left) displacement to each leg's local y.

    Assumption (matches your calibration convention):
      - For RIGHT legs, local +y corresponds to body +Y (outward to the right side is body +Y)
      - For LEFT legs, local +y corresponds to body -Y (outward to the left side is body -Y)
    So local_y = +body_y for right legs, and local_y = -body_y for left legs.
    """
    return body_y if leg_id in RIGHT_LEGS else -body_y


def side_sign(leg_id: int) -> int:
    return +1 if leg_id in RIGHT_LEGS else -1


# -------------------------
# Non-blocking keyboard reader (WASD/QE)
# -------------------------
class KeyReader:
    def __init__(self):
        self.fd = sys.stdin.fileno()
        self.old = termios.tcgetattr(self.fd)

    def __enter__(self):
        tty.setcbreak(self.fd)  # non-canonical, no-echo
        return self

    def __exit__(self, exc_type, exc, tb):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)

    def read_key(self) -> str | None:
        """Return a single character if available, else None."""
        r, _, _ = select.select([sys.stdin], [], [], 0.0)
        if not r:
            return None
        ch = sys.stdin.read(1)
        return ch


@dataclass
class Cmd:
    vx: int = 0   # forward(+1) / back(-1)
    vy: int = 0   # left(+1) / right(-1) in BODY frame
    wz: int = 0   # yaw CCW(+1) / CW(-1)


def key_to_cmd(k: str) -> Cmd:
    k = k.lower()
    if k == "w":
        return Cmd(vx=+1, vy=0, wz=0)
    if k == "s":
        return Cmd(vx=-1, vy=0, wz=0)
    if k == "a":
        return Cmd(vx=0, vy=+1, wz=0)  # left
    if k == "d":
        return Cmd(vx=0, vy=-1, wz=0)  # right
    if k == "q":
        return Cmd(vx=0, vy=0, wz=+1)  # CCW
    if k == "e":
        return Cmd(vx=0, vy=0, wz=-1)  # CW
    return Cmd(0, 0, 0)


# -------------------------
# Crawl driver
# -------------------------
class CrawlDriver:
    def __init__(self):
        self.api = make_default_api()

        sx, sy, sz = STAND_XYZ
        # per-leg current target (x,y,z) in that leg's local frame
        self.foot: Dict[int, Tuple[float, float, float]] = {i: (sx, sy, sz) for i in (0, 1, 2, 3)}
        self.stand = (sx, sy, sz)

        self.order_idx = 0

    def set_pose(self, leg_id: int, x: float, y: float, z: float, duration: float):
        x0, y0, z0 = self.foot[leg_id]
        steps = max(1, int(duration / MOVE_DT))
        for i in range(steps + 1):
            u = i / steps
            xi = lerp(x0, x, u)
            yi = lerp(y0, y, u)
            zi = lerp(z0, z, u)
            self.api.set_leg_xyz(leg_id, xi, yi, zi, debug=False)
            time.sleep(MOVE_DT)
        self.foot[leg_id] = (x, y, z)

    def set_all(self, x: float, y: float, z: float, duration: float):
        # smooth all legs from current positions
        steps = max(1, int(duration / MOVE_DT))
        x0 = {i: self.foot[i][0] for i in (0, 1, 2, 3)}
        y0 = {i: self.foot[i][1] for i in (0, 1, 2, 3)}
        z0 = {i: self.foot[i][2] for i in (0, 1, 2, 3)}
        for s in range(steps + 1):
            u = s / steps
            for leg_id in (0, 1, 2, 3):
                xi = lerp(x0[leg_id], x, u)
                yi = lerp(y0[leg_id], y, u)
                zi = lerp(z0[leg_id], z, u)
                self.api.set_leg_xyz(leg_id, xi, yi, zi, debug=False)
            time.sleep(MOVE_DT)
        for leg_id in (0, 1, 2, 3):
            self.foot[leg_id] = (x, y, z)

    def go_stand(self, duration: float = 0.4):
        sx, sy, sz = self.stand
        self.set_all(sx, sy, sz, duration)

    def shift_body(self, swing_leg: int, body_shift_y: float, duration: float):
        """Shift 'body' laterally by moving all feet (in local y) accordingly.

        body_shift_y: + means shift body LEFT in body frame.
        We convert to each leg's local y shift.
        """
        # Apply to ALL legs (including swing leg) while still on ground.
        targets = {}
        for leg_id in (0, 1, 2, 3):
            x, y, z = self.foot[leg_id]
            dy_local = body_y_to_local_y(leg_id, body_shift_y)
            targets[leg_id] = (x, y + dy_local, z)

        # interpolate simultaneously
        steps = max(1, int(duration / MOVE_DT))
        start = {i: self.foot[i] for i in (0, 1, 2, 3)}
        for s in range(steps + 1):
            u = s / steps
            for leg_id in (0, 1, 2, 3):
                x0, y0, z0 = start[leg_id]
                xt, yt, zt = targets[leg_id]
                xi = lerp(x0, xt, u)
                yi = lerp(y0, yt, u)
                zi = lerp(z0, zt, u)
                self.api.set_leg_xyz(leg_id, xi, yi, zi, debug=False)
            time.sleep(MOVE_DT)

        for leg_id in (0, 1, 2, 3):
            self.foot[leg_id] = targets[leg_id]

    def crawl_step(self, cmd: Cmd):
        """Execute one crawl step using current cmd."""
        if cmd.vx == 0 and cmd.vy == 0 and cmd.wz == 0:
            # no movement requested
            self.go_stand(duration=0.25)
            return

        swing_leg = CRAWL_ORDER[self.order_idx % len(CRAWL_ORDER)]
        self.order_idx += 1

        sx, sy, sz = self.stand
        z_lift = sz - LIFT_DZ  # smaller z lifts

        # Convert cmd into per-leg swing delta (local frame)
        # Translation:
        dx = cmd.vx * STEP_FWD
        dy_body = cmd.vy * STEP_LAT

        # Yaw:
        # For CCW (+wz), push right legs forward, left legs backward (differential dx)
        dx_yaw = cmd.wz * STEP_YAW
        dx_leg = dx + dx_yaw * side_sign(swing_leg)

        # Lateral command uses body-y -> local-y mapping
        dy_leg = body_y_to_local_y(swing_leg, dy_body)

        # Support legs get a small opposite motion during swing to approximate body progress
        support = [i for i in (0, 1, 2, 3) if i != swing_leg]
        if support:
            sup_dx = -dx_leg / len(support)
            sup_dy = -dy_leg / len(support)
        else:
            sup_dx = 0.0
            sup_dy = 0.0

        # 1) SHIFT away from swing leg
        # If swing leg is RIGHT, shift body LEFT (+Y). If swing leg is LEFT, shift body RIGHT (-Y).
        desired_body_shift = +SHIFT_MAG if swing_leg in RIGHT_LEGS else -SHIFT_MAG
        self.shift_body(swing_leg, desired_body_shift, PHASE_T)

        # 2) LIFT swing leg (only z)
        x0, y0, _ = self.foot[swing_leg]
        self.set_pose(swing_leg, x0, y0, z_lift, PHASE_T)

        # 3) SWING: move swing leg to new x/y while keeping lifted z.
        # Simultaneously move support legs a little opposite (stance).
        swing_target = (x0 + dx_leg, y0 + dy_leg, z_lift)
        support_targets = {}
        for leg_id in support:
            xs, ys, zs = self.foot[leg_id]
            support_targets[leg_id] = (xs + sup_dx, ys + sup_dy, zs)

        steps = max(1, int(PHASE_T / MOVE_DT))
        swing_start = self.foot[swing_leg]
        support_start = {i: self.foot[i] for i in support}

        for s in range(steps + 1):
            u = s / steps

            # swing leg
            xS0, yS0, zS0 = swing_start
            xSt, ySt, zSt = swing_target
            xi = lerp(xS0, xSt, u)
            yi = lerp(yS0, ySt, u)
            zi = lerp(zS0, zSt, u)
            self.api.set_leg_xyz(swing_leg, xi, yi, zi, debug=False)

            # support legs
            for leg_id in support:
                xA0, yA0, zA0 = support_start[leg_id]
                xAt, yAt, zAt = support_targets[leg_id]
                xj = lerp(xA0, xAt, u)
                yj = lerp(yA0, yAt, u)
                zj = lerp(zA0, zAt, u)
                self.api.set_leg_xyz(leg_id, xj, yj, zj, debug=False)

            time.sleep(MOVE_DT)

        self.foot[swing_leg] = swing_target
        for leg_id in support:
            self.foot[leg_id] = support_targets[leg_id]

        # 4) TOUCHDOWN swing leg back to stand z (keep x/y)
        x1, y1, _ = self.foot[swing_leg]
        self.set_pose(swing_leg, x1, y1, sz, PHASE_T)

        # 5) UNSHIFT: return all feet y back toward stand y smoothly (keep current x)
        # We move only y toward stand, keep x as is, keep z at stand.
        steps = max(1, int(PHASE_T / MOVE_DT))
        start = {i: self.foot[i] for i in (0, 1, 2, 3)}
        for s in range(steps + 1):
            u = s / steps
            for leg_id in (0, 1, 2, 3):
                x0, y0, z0 = start[leg_id]
                # keep x, move y back to stand y, keep z at stand
                xi = x0
                yi = lerp(y0, sy, u)
                zi = lerp(z0, sz, u)
                self.api.set_leg_xyz(leg_id, xi, yi, zi, debug=False)
            time.sleep(MOVE_DT)
        for leg_id in (0, 1, 2, 3):
            x0, _, _ = self.foot[leg_id]
            self.foot[leg_id] = (x0, sy, sz)

    def shutdown(self):
        self.api.go_center_pose(debug=True)


def main():
    drv = CrawlDriver()
    print("[INFO] Stand pose:", STAND_XYZ)
    print("[INFO] Keys: W/S/A/D/Q/E, other = stop. Ctrl+C to exit.")
    drv.go_stand(duration=0.6)

    cmd = Cmd(0, 0, 0)
    last_key_t = 0.0

    with KeyReader() as kr:
        try:
            while True:
                k = kr.read_key()
                now = time.time()

                if k is not None:
                    cmd = key_to_cmd(k)
                    last_key_t = now

                # auto stop if no input for a while
                if (now - last_key_t) > IDLE_HOLD:
                    cmd = Cmd(0, 0, 0)

                # execute one crawl step (or hold stand if stopped)
                drv.crawl_step(cmd)

        except KeyboardInterrupt:
            print("\n[CTRL+C] Exit")
        finally:
            drv.shutdown()


if __name__ == "__main__":
    main()