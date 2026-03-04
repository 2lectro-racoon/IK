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
# - z direction: in your robot, "bigger z lifts" (you already confirmed)
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
from ik_3dof_a0 import IKError
import ik_3dof_a0 as ikmod


# -------------------------
# Tunable parameters
# -------------------------
STAND_XYZ = (70.0, 120.0, -50.0)   # your current stable stand
LIFT_DZ = 60.0                     # lift: z_lift = z_stand - LIFT_DZ (smaller z lifts)
SHIFT_MAG = 0.0                   # magnitude of lateral shift (mm) before lifting
# --- Counterweight option #1: diagonal counter in X/Y/Z (local) ---
COUNTER_DX = 20.0                  # mm, temporary diagonal-leg counter in BODY X (front +)
COUNTER_DY = 20.0                  # mm, temporary diagonal-leg counter in BODY Y (left +)
COUNTER_DZ = 20.0                  # mm, temporary counter in Z (local z)
# --- Counterweight option #2: also apply to 2nd support leg (opposite side) ---
COUNTER2_ENABLE = True             # also counter on a 2nd support leg (opposite side)
COUNTER2_SCALE = 0.6               # scale for the 2nd leg counter (0..1)
STEP_FWD = 30.0                    # mm per step for forward/back command
STEP_LAT = 30.0                    # mm per step for left/right command
STEP_YAW = 40.0                    # mm per step for yaw command (as differential dx between sides)
# SWING arc
SWING_ARC_DZ = 15.0                # extra mm added at mid-swing for a nicer foot arc (set 0 to disable)

# Stance "body move" (Arduino-like): after placing legs, move ALL feet together opposite to command.
# This creates a clearer "push" / body-advance feel than a small sweep.
BODYMOVE_ENABLE = True
BODYMOVE_T = 0.35                 # seconds for the body-move phase (slower than swing)
# Use step-scaled deltas so the body move is comparable to the leg swing distance.
BODYMOVE_FWD = 1.0                # multiplier for STEP_FWD (1.0 means same magnitude)
BODYMOVE_LAT = 0.8                # multiplier for STEP_LAT
BODYMOVE_YAW = 0.6                # multiplier for STEP_YAW

# (legacy) Push parameters kept for reference; not used anymore.
PUSH_ENABLE = False
PUSH_T = 0.25
PUSH_FWD = 20.0
PUSH_LAT = 12.0
PUSH_YAW = 15.0
# STEP_FWD = 25.0                    # mm per step for forward/back command
# STEP_LAT = 20.0                    # mm per step for left/right command
# STEP_YAW = 20.0                    # mm per step for yaw command (as differential dx between sides)

# Timing
MOVE_DT = 0.02                     # 25 Hz interpolation
# MOVE_DT = 0.04                     # 25 Hz interpolation
PHASE_T = 0.2                     # seconds for each phase (shift/lift/swing/down/unshift)
# PHASE_T = 0.45                     # seconds for each phase (shift/lift/swing/down/unshift)
IDLE_HOLD = 0.35                   # if no key within this time -> cmd becomes 0 (stop)

# Crawl order: FR -> BR -> FL -> BL
CRAWL_ORDER = [0, 1, 3, 2]         # 0=FR,1=BR,2=BL,3=FL

# Leg side sets (for mapping body-left to local y)
RIGHT_LEGS = {0, 1}                # FR, BR
LEFT_LEGS = {2, 3}                 # BL, FL

# Leg front/back sets (for mapping body-forward to local x)
FRONT_LEGS = {0, 3}                # FR, FL

BACK_LEGS = {1, 2}                 # BR, BL

#
# Diagonal leg mapping (corner-to-corner)
DIAG_LEG = {0: 2, 2: 0, 1: 3, 3: 1}  # FR<->BL, BR<->FL

# Opposite-side legs (the side opposite of swing leg)
OPPOSITE_SIDE_LEGS = {0: LEFT_LEGS, 1: LEFT_LEGS, 2: RIGHT_LEGS, 3: RIGHT_LEGS}


# -------------------------
# Small helpers
# -------------------------
def smoothstep(t: float) -> float:
    """C1 continuous easing: 3t^2 - 2t^3 (t in [0,1])."""
    t = clampf(t, 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)


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


def body_x_to_local_x(leg_id: int, body_x: float) -> float:
    """Convert body-frame +X(forward) displacement to each leg's local x.

    You measured:
      - Front legs: local +x moves the foot forward
      - Back legs : local +x moves the foot backward

    Therefore:
      local_x = +body_x for FRONT legs
      local_x = -body_x for BACK legs
    """
    return body_x if leg_id in FRONT_LEGS else -body_x


def side_sign(leg_id: int) -> int:
    return +1 if leg_id in RIGHT_LEGS else -1


# -------------------------
# Debug IK helper (for debug print only)
# -------------------------
def _try_ik_angles_deg(x: float, y: float, z: float) -> Tuple[float, float, float] | None:
    """Best-effort IK debug helper.

    Tries to call an IK function from `ik_3dof_a0` if present. Returns (a0,a1,a2) in degrees.
    This is used ONLY for debug prints; motion behavior must not depend on this.
    """
    # Common function names used in different IK implementations
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

    # Normalize output
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

    # Heuristic: if values look like radians (small magnitudes), convert to degrees.
    # We assume degrees if any magnitude is > ~3.5 rad (~200 deg) is impossible, so:
    if max(abs(a0), abs(a1), abs(a2)) <= 6.3:  # ~360deg in rad is 6.28
        # Could still be degrees (e.g., 1~2 deg), but for debug we prefer radians->deg conversion.
        a0 = math.degrees(a0)
        a1 = math.degrees(a1)
        a2 = math.degrees(a2)

    return (a0, a1, a2)


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

    def drain_last_key(self) -> str | None:
        """Drain all pending keys and return only the last one.

        This prevents OS key-repeat from queueing many characters while gait steps are slow.
        """
        last = None
        while True:
            k = self.read_key()
            if k is None:
                return last
            last = k


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
        return Cmd(vx=0, vy=-1, wz=0)  # left
    if k == "d":
        return Cmd(vx=0, vy=+1, wz=0)  # right
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
        # Forward/back sequencer (Arduino-like): BL -> FL -> PUSH -> BR -> FR -> PUSH
        self.fb_idx = 0

    def _try_set_leg_xyz(self, leg_id: int, x: float, y: float, z: float) -> bool:
        """Set one leg target; return False if IK is unreachable."""
        try:
            self.api.set_leg_xyz(leg_id, x, y, z, debug=False)
            return True
        except IKError as e:
            print(f"[IKError] leg={leg_id} target=({x:.1f},{y:.1f},{z:.1f}) -> {e}")
            return False

    def set_pose(self, leg_id: int, x: float, y: float, z: float, duration: float) -> bool:
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
        # smooth all legs from current positions
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
        sx, sy, sz = self.stand
        _ = self.set_all(sx, sy, sz, duration)

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
        """Arduino-like BODY MOVE phase (all feet on ground).

        Moves ALL feet together opposite to the commanded body motion.
        Returns False if any IKError occurs.
        """
        if not BODYMOVE_ENABLE:
            return True
        if cmd.vx == 0 and cmd.vy == 0 and cmd.wz == 0:
            return True

        sx, sy, sz = self.stand

        # Opposite of commanded BODY motion, scaled from step sizes.
        body_dx = -(cmd.vx * STEP_FWD * BODYMOVE_FWD)
        body_dy = -(cmd.vy * STEP_LAT * BODYMOVE_LAT)
        body_dx_yaw = -(cmd.wz * STEP_YAW * BODYMOVE_YAW)

        print(
            f"[BODYMOVE] cmd(vx,vy,wz)=({cmd.vx:+d},{cmd.vy:+d},{cmd.wz:+d}) "
            f"body_dx={body_dx:+.1f} body_dy={body_dy:+.1f} body_dx_yaw={body_dx_yaw:+.1f} T={BODYMOVE_T:.2f}"
        )

        targets: Dict[int, Tuple[float, float, float]] = {}
        for leg_id in (0, 1, 2, 3):
            x_cur, y_cur, _ = self.foot[leg_id]

            # Include yaw differential per side in BODY frame, then convert to this leg's LOCAL.
            body_dx_leg = body_dx + body_dx_yaw * side_sign(leg_id)
            dx_local = body_x_to_local_x(leg_id, body_dx_leg)
            dy_local = body_y_to_local_y(leg_id, body_dy)

            targets[leg_id] = (x_cur + dx_local, y_cur + dy_local, sz)

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

    def _single_leg_step_no_push(self, swing_leg: int, cmd: Cmd) -> bool:
        """One single-leg step WITHOUT the PUSH phase.

        This performs: SHIFT -> COUNTER -> LIFT -> SWING -> TOUCHDOWN -> UNSHIFT.
        Returns False if any IKError occurs.
        """
        diag_leg = DIAG_LEG[swing_leg]

        # Second counter leg: the other support leg on the opposite side of the swing leg.
        opp_side = OPPOSITE_SIDE_LEGS[swing_leg]
        ctr2_leg = None
        if COUNTER2_ENABLE:
            cand = [lid for lid in opp_side if lid != diag_leg]
            ctr2_leg = cand[0] if cand else None

        # Temporary counter (LOCAL)
        dx_ctr_local = +COUNTER_DX
        dy_ctr_local = +COUNTER_DY
        dz_ctr_local = +COUNTER_DZ

        dx_ctr2_local = dx_ctr_local * COUNTER2_SCALE if ctr2_leg is not None else 0.0
        dy_ctr2_local = dy_ctr_local * COUNTER2_SCALE if ctr2_leg is not None else 0.0
        dz_ctr2_local = dz_ctr_local * COUNTER2_SCALE if ctr2_leg is not None else 0.0

        sx, sy, sz = self.stand
        z_lift = sz + LIFT_DZ

        # BODY command deltas
        body_dx = cmd.vx * STEP_FWD
        body_dy = cmd.vy * STEP_LAT
        body_dx_yaw = cmd.wz * STEP_YAW

        # Swing leg: BODY -> LOCAL
        body_dx_swing = body_dx + body_dx_yaw * side_sign(swing_leg)
        dx_leg = body_x_to_local_x(swing_leg, body_dx_swing)
        dy_leg = body_y_to_local_y(swing_leg, body_dy)

        support = [i for i in (0, 1, 2, 3) if i != swing_leg]
        body_dy_support = (-body_dy / len(support)) if support else 0.0

        # 1) SHIFT away from swing leg
        desired_body_shift = +SHIFT_MAG if swing_leg in RIGHT_LEGS else -SHIFT_MAG
        self.shift_body(swing_leg, desired_body_shift, PHASE_T)

        # 1b) TEMP COUNTER: move diagonal (and optional 2nd) support foot outward (XYZ)
        xd, yd, zd = self.foot[diag_leg]
        if not self.set_pose(diag_leg, xd + dx_ctr_local, yd + dy_ctr_local, zd + dz_ctr_local, PHASE_T):
            return False

        if ctr2_leg is not None:
            x2, y2, z2 = self.foot[ctr2_leg]
            if not self.set_pose(ctr2_leg, x2 + dx_ctr2_local, y2 + dy_ctr2_local, z2 + dz_ctr2_local, PHASE_T):
                return False

        # 2) LIFT swing leg
        x0, y0, _ = self.foot[swing_leg]
        if not self.set_pose(swing_leg, x0, y0, z_lift, PHASE_T):
            return False

        # 3) SWING: swing leg forward/back and small stance distribution
        swing_target = (x0 + dx_leg, y0 + dy_leg, z_lift)

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

            # swing leg interp
            xS0, yS0, zS0 = swing_start
            xSt, ySt, zSt = swing_target
            xi = lerp(xS0, xSt, ue)
            yi = lerp(yS0, ySt, ue)
            zi = lerp(zS0, zSt, ue)
            if SWING_ARC_DZ and SWING_ARC_DZ > 0:
                zi = zi + SWING_ARC_DZ * math.sin(math.pi * ue)
            if not self._try_set_leg_xyz(swing_leg, xi, yi, zi):
                return False

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
        for leg_id in support:
            self.foot[leg_id] = support_targets[leg_id]

        # 4) TOUCHDOWN
        x1, y1, _ = self.foot[swing_leg]
        if not self.set_pose(swing_leg, x1, y1, sz, PHASE_T):
            return False

        # 5) UNSHIFT (remove shift + remove temporary counter XY for diag/ctr2)
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
        """Arduino-like forward/back sequence (side pair, rear->front, then body-move).

        Sequence (repeats): BL -> FL -> BODYMOVE -> BR -> FR -> BODYMOVE
        """
        seq = [2, 3, -1, 1, 0, -1]  # -1 means BODYMOVE-only
        item = seq[self.fb_idx % len(seq)]
        self.fb_idx += 1

        if item == -1:
            print(f"[FB] step={self.fb_idx:04d} phase=BODYMOVE")
            ok = self._push_all(cmd)
            if not ok:
                self.go_stand(duration=0.3)
            return

        # Log swing target and IK (best effort) BEFORE executing
        x0, y0, z0 = self.foot[item]
        body_dx = cmd.vx * STEP_FWD
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
                f"swing_tgt=({swing_tgt[0]:+.1f},{swing_tgt[1]:+.1f},{swing_tgt[2]:+.1f}) a0/a1/a2=(n/a)"
            )
        else:
            a0d, a1d, a2d = ang
            print(
                f"[FB] step={self.fb_idx:04d} leg={item} (BL/FL/BR/FR)="
                f"{['FR','BR','BL','FL'][item]} "
                f"swing_tgt=({swing_tgt[0]:+.1f},{swing_tgt[1]:+.1f},{swing_tgt[2]:+.1f}) "
                f"a0={a0d:+.2f} a1={a1d:+.2f} a2={a2d:+.2f}"
            )

        ok = self._single_leg_step_no_push(item, cmd)
        if not ok:
            self.go_stand(duration=0.3)
            return

    def crawl_step(self, cmd: Cmd):
        """Execute one crawl step using current cmd."""
        if cmd.vx == 0 and cmd.vy == 0 and cmd.wz == 0:
            # no movement requested
            self.go_stand(duration=0.25)
            return

        # Forward/back special: Arduino-like sequence using side pairs (rear->front) with PUSH phases.
        if cmd.vx != 0 and cmd.vy == 0 and cmd.wz == 0:
            self.fb_step(cmd)
            return

        swing_leg = CRAWL_ORDER[self.order_idx % len(CRAWL_ORDER)]
        self.order_idx += 1

        diag_leg = DIAG_LEG[swing_leg]

        # Second counter leg: the other support leg on the opposite side of the swing leg.
        # Example: swing BR(1) -> opposite side is LEFT {BL,FL}; diag is FL so ctr2 is BL.
        opp_side = OPPOSITE_SIDE_LEGS[swing_leg]
        ctr2_leg = None
        if COUNTER2_ENABLE:
            cand = [lid for lid in opp_side if lid != diag_leg]
            ctr2_leg = cand[0] if cand else None

        # Temporary counter: push support feet OUTWARD in each leg's *local* XYZ.
        # Your local convention (per leg): (+x, +y) extends the leg outward (away from body).
        dx_ctr_local = +COUNTER_DX
        dy_ctr_local = +COUNTER_DY
        dz_ctr_local = +COUNTER_DZ

        dx_ctr2_local = dx_ctr_local * COUNTER2_SCALE if ctr2_leg is not None else 0.0
        dy_ctr2_local = dy_ctr_local * COUNTER2_SCALE if ctr2_leg is not None else 0.0
        dz_ctr2_local = dz_ctr_local * COUNTER2_SCALE if ctr2_leg is not None else 0.0

        # Debug: counter applied (LOCAL frame)
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
        z_lift = sz + LIFT_DZ  # smaller z lifts

        # Convert cmd into per-leg swing delta.
        # We treat commands in BODY frame first, then convert to each leg's LOCAL frame.
        body_dx = cmd.vx * STEP_FWD          # +X forward
        body_dy = cmd.vy * STEP_LAT          # +Y left
        body_dx_yaw = cmd.wz * STEP_YAW      # yaw contribution in BODY frame

        # Swing leg: BODY -> LOCAL
        body_dx_swing = body_dx + body_dx_yaw * side_sign(swing_leg)
        dx_leg = body_x_to_local_x(swing_leg, body_dx_swing)
        dy_leg = body_y_to_local_y(swing_leg, body_dy)
        # Debug: show swing leg command in local frame
        print(
            f"[SWING] leg={swing_leg} body_dx={body_dx_swing:+.1f} body_dy={body_dy:+.1f} "
            f"-> local_dx={dx_leg:+.1f} local_dy={dy_leg:+.1f}"
        )

        # Support legs (stance): distribute opposite motion per leg in BODY frame, then map per leg to LOCAL.
        support = [i for i in (0, 1, 2, 3) if i != swing_leg]
        if support:
            body_dy_support = -body_dy / len(support)
        else:
            body_dy_support = 0.0

        # 1) SHIFT away from swing leg
        # If swing leg is RIGHT, shift body LEFT (+Y). If swing leg is LEFT, shift body RIGHT (-Y).
        desired_body_shift = +SHIFT_MAG if swing_leg in RIGHT_LEGS else -SHIFT_MAG
        self.shift_body(swing_leg, desired_body_shift, PHASE_T)

        # 1b) TEMP COUNTER: move diagonal (and optional 2nd) support foot outward (XY)
        xd, yd, zd = self.foot[diag_leg]
        if not self.set_pose(diag_leg, xd + dx_ctr_local, yd + dy_ctr_local, zd + dz_ctr_local, PHASE_T):
            self.go_stand(duration=0.3)
            return

        if ctr2_leg is not None:
            x2, y2, z2 = self.foot[ctr2_leg]
            if not self.set_pose(ctr2_leg, x2 + dx_ctr2_local, y2 + dy_ctr2_local, z2 + dz_ctr2_local, PHASE_T):
                self.go_stand(duration=0.3)
                return

        # 2) LIFT swing leg (only z)
        x0, y0, _ = self.foot[swing_leg]
        if not self.set_pose(swing_leg, x0, y0, z_lift, PHASE_T):
            self.go_stand(duration=0.3)
            return

        # 3) SWING: move swing leg to new x/y while keeping lifted z.
        # Simultaneously move support legs a little opposite (stance).
        swing_target = (x0 + dx_leg, y0 + dy_leg, z_lift)
        # Debug: swing target XYZ and IK angles (if available)
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

            # BODY stance delta for this leg includes yaw (right forward / left backward), then map to LOCAL.
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

            # swing leg
            xS0, yS0, zS0 = swing_start
            xSt, ySt, zSt = swing_target
            xi = lerp(xS0, xSt, ue)
            yi = lerp(yS0, ySt, ue)
            zi = lerp(zS0, zSt, ue)

            # add a gentle mid-swing arc (keeps endpoints unchanged)
            if SWING_ARC_DZ and SWING_ARC_DZ > 0:
                zi = zi + SWING_ARC_DZ * math.sin(math.pi * ue)

            if not self._try_set_leg_xyz(swing_leg, xi, yi, zi):
                self.go_stand(duration=0.3)
                return

            # support legs
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

        # 4) TOUCHDOWN swing leg back to stand z (keep x/y)
        x1, y1, _ = self.foot[swing_leg]
        if not self.set_pose(swing_leg, x1, y1, sz, PHASE_T):
            self.go_stand(duration=0.3)
            return


        # 5) UNSHIFT: remove ONLY the temporary shift (phase 1), preserving commanded y changes.
        # During shift_body() we added dy_local = body_y_to_local_y(leg_id, desired_body_shift).
        # Here we subtract the same amount, while also removing the temporary diagonal-leg counter (x and y).
        steps = max(1, int(PHASE_T / MOVE_DT))
        start = {i: self.foot[i] for i in (0, 1, 2, 3)}

        # Target x/y for each leg: remove the temporary SHIFT, and also remove the temporary
        # counter components. Preserve other commanded stance motion.
        x_target = {}
        y_target = {}
        for leg_id in (0, 1, 2, 3):
            dy_local_shift = body_y_to_local_y(leg_id, desired_body_shift)
            x_cur, y_cur, _ = self.foot[leg_id]

            # Always remove the temporary lateral shift component
            y_t = y_cur - dy_local_shift
            x_t = x_cur

            # Additionally remove the temporary counter components
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
        self.api.leg_reset()
        time.sleep(15)
    
    def shutdown(self):
        self.api.go_center_pose(debug=True)


def main():
    
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
                k = kr.drain_last_key()
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