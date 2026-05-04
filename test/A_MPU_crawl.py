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
# - z direction: in your robot, "larger z lifts" (confirmed)
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
import afb2
from dataclasses import dataclass
from typing import Dict, Tuple

from A_quad_api import make_default_api
from A_ik_3dof_a0 import IKError


# -------------------------
# Tunable parameters
# -------------------------
STAND_XYZ = (120.0, 70.0, -50.0)   # your current stable stand
LIFT_DZ = 60.0                     # lift: z_lift = z_stand + LIFT_DZ (larger z lifts)
SHIFT_MAG = 20.0                   # magnitude of lateral shift (mm) before lifting
STEP_FWD = 30.0                    # mm per step for forward/back command
STEP_LAT = 30.0                    # mm per step for left/right command
STEP_YAW = 40.0                    # mm per step for yaw command (as differential dx between sides)
# SWING_ARC_DZ = 15.0                # extra mm added at mid-swing for a nicer foot arc (set 0 to disable)
SWING_ARC_DZ = 15.0                # extra mm added at mid-swing for a nicer foot arc (set 0 to disable)

# -------------------------
# Stand-only IMU stabilization (test mode)
# When cmd is STOP (vx=vy=wz=0), we keep the robot in stand and apply small per-leg Z deltas
# based on IMU roll/pitch error relative to a stored "zero" reference.
# Note: Since larger Z lifts the foot (shortens leg), to raise a low side of the body we generally
# need to DECREASE Z on that side (lengthen leg) and INCREASE Z on the opposite side.
# -------------------------
IMU_STAB_ENABLE = True
IMU_STAB_MAX_DZ = 15              # mm max per-leg Z adjustment
IMU_STAB_K_ROLL = 1.8              # mm per degree roll error (right-down +)
IMU_STAB_K_PITCH = 1.8             # mm per degree pitch error (front-down +)
IMU_STAB_ALPHA_DZ = 0.80           # smoothing for dz commands (0..1), higher = faster

IMU_ZERO_SEC = 0.6                 # seconds to average roll/pitch for zero reference

# -------------------------
# Gait IMU assist (phase-1): apply IMU Z compensation ONLY right after SHIFT and before LIFT
# -------------------------
IMU_SHIFT_ASSIST_ENABLE = True
IMU_SHIFT_ASSIST_TICKS = 8         # number of short correction ticks after SHIFT
IMU_SHIFT_ASSIST_MAX_DZ = 10.0      # mm clamp for this assist (keep small to avoid IK issues)

# -------------------------
# Gait IMU assist (phase-2): apply IMU Z compensation to STANCE legs during LIFT+SWING
# (helps most with tipping right when a leg is lifted)
# -------------------------
IMU_STANCE_ASSIST_ENABLE = True
IMU_STANCE_ASSIST_MAX_DZ = 8.0     # mm clamp during stance assist (start small)

# -------------------------
# IMU filtering (MPU6050) - complementary filter
# Units (confirmed): accel m/s^2, gyro rad/s
# Axis mapping (confirmed by your tests):
#   - roll  (right-down positive):  accel ax+, gyro gy+
#   - pitch (front-down positive):  accel ay+, gyro gx+
# -------------------------
# IMU filtering (MPU6050) - complementary filter
# Units (confirmed): accel m/s^2, gyro rad/s
# Axis mapping (confirmed by your tests):
#   - roll  (right-down positive):  accel ax+, gyro gy+
#   - pitch (front-down positive):  accel ay+, gyro gx+
IMU_ALPHA = 0.99          # higher -> trust gyro more (less accel noise)
IMU_CALIB_SEC = 2.0       # seconds of gyro bias averaging at startup
IMU_PRINT_HZ = 5.0        # how often to print roll/pitch (Hz)

# ---- Gait debug logging ----
GAIT_DEBUG_ENABLE = True
GAIT_DEBUG_HZ = 6.0   # rate-limit phase debug prints

# ---- IMU acceleration-magnitude gating for complementary filter ----
IMU_USE_ACCEL_GATING = True
IMU_G_MIN = 8.5           # m/s^2, lower bound to consider accel tilt reliable
IMU_G_MAX = 11.5          # m/s^2, upper bound to consider accel tilt reliable
# STEP_FWD = 25.0                    # mm per step for forward/back command
# STEP_LAT = 20.0                    # mm per step for left/right command
# STEP_YAW = 20.0                    # mm per step for yaw command (as differential dx between sides)

# Timing
MOVE_DT = 0.01                     # 100 Hz tick (used by stand IMU stabilization and interpolation)
# MOVE_DT = 0.04                     # 25 Hz interpolation
PHASE_T = 0.2                     # seconds for each phase (shift/lift/swing/down/unshift)
# PHASE_T = 0.45                     # seconds for each phase (shift/lift/swing/down/unshift)
IDLE_HOLD = 0.35                   # if no key within this time -> cmd becomes 0 (stop)

# Crawl order (stable): FR -> BL -> FL -> BR
CRAWL_ORDER = [0, 2, 3, 1]         # 0=FR,1=BR,2=BL,3=FL

# Leg side sets (for mapping body-left to local y)
RIGHT_LEGS = {0, 1}                # FR, BR
LEFT_LEGS = {2, 3}                 # BL, FL

# Leg front/back sets (for mapping body-forward to local x)
FRONT_LEGS = {0, 3}                # FR, FL

BACK_LEGS = {1, 2}                 # BR, BL

# Leg id to name mapping
LEG_NAME = {0: "FR", 1: "BR", 2: "BL", 3: "FL"}

# -------------------------
# IMU shared state (updated in main loop)
# Used by CrawlDriver for gait-phase IMU assist.
# -------------------------
IMU_LATEST_ROLL_DEG = 0.0
IMU_LATEST_PITCH_DEG = 0.0
IMU_ZERO_ROLL_DEG = 0.0
IMU_ZERO_PITCH_DEG = 0.0


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


def rad2deg(r: float) -> float:
    return r * 180.0 / math.pi


class IMUComplementary:
    """Simple complementary filter for roll/pitch.

    roll  : right-down positive
    pitch : front-down positive

    Uses gyro integration for fast response and accel tilt for drift correction.
    """

    def __init__(self, alpha: float = IMU_ALPHA):
        self.alpha = clampf(alpha, 0.90, 0.999)
        self.gx_bias = 0.0
        self.gy_bias = 0.0
        self.gz_bias = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self._t_prev: float | None = None

    def calibrate_gyro(self, seconds: float = IMU_CALIB_SEC, sample_hz: float = 200.0):
        """Estimate gyro bias while robot is stationary."""
        n = max(1, int(seconds * sample_hz))
        dt = 1.0 / sample_hz
        sx = sy = sz = 0.0
        for _ in range(n):
            ax, ay, az, gx, gy, gz = afb2.sensor.mpu()
            sx += gx
            sy += gy
            sz += gz
            time.sleep(dt)
        self.gx_bias = sx / n
        self.gy_bias = sy / n
        self.gz_bias = sz / n
        # Initialize roll/pitch from accel once after calibration
        ax, ay, az, gx, gy, gz = afb2.sensor.mpu()
        self.roll = math.atan2(ax, az)
        self.pitch = math.atan2(ay, az)
        self._t_prev = time.time()

    def update(self, ax: float, ay: float, az: float, gx: float, gy: float, gz: float, now: float | None = None) -> tuple[float, float]:
        """Update filter and return (roll, pitch) in radians."""
        if now is None:
            now = time.time()
        if self._t_prev is None:
            self._t_prev = now
            # Initialize from accel
            self.roll = math.atan2(ax, az)
            self.pitch = math.atan2(ay, az)
            return self.roll, self.pitch

        dt = now - self._t_prev
        self._t_prev = now
        # Guard dt (in case the gait step blocks for a while)
        dt = clampf(dt, 0.001, 0.05)

        # Apply gyro bias
        gx -= self.gx_bias
        gy -= self.gy_bias
        # gz bias not used for roll/pitch, but kept for completeness

        # Your mapping: roll_rate=gy, pitch_rate=gx
        roll_gyro = self.roll + gy * dt
        pitch_gyro = self.pitch + gx * dt

        # Accel tilt (valid when linear accel is small)
        # During gait, |a| can deviate from g due to impacts/linear acceleration.
        # Gate accel correction to avoid corrupting roll/pitch.
        use_accel = True
        if IMU_USE_ACCEL_GATING:
            amag = math.sqrt(ax * ax + ay * ay + az * az)
            use_accel = (IMU_G_MIN <= amag <= IMU_G_MAX)

        if use_accel:
            roll_acc = math.atan2(ax, az)
            pitch_acc = math.atan2(ay, az)
            a = self.alpha
            self.roll = a * roll_gyro + (1.0 - a) * roll_acc
            self.pitch = a * pitch_gyro + (1.0 - a) * pitch_acc
        else:
            # Gyro-only update for this tick
            self.roll = roll_gyro
            self.pitch = pitch_gyro

        return self.roll, self.pitch

    def update_from_mpu(self) -> tuple[float, float]:
        ax, ay, az, gx, gy, gz = afb2.sensor.mpu()
        return self.update(ax, ay, az, gx, gy, gz)


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
        self._dbg_last_t = 0.0
        self._dbg_dt = 1.0 / max(0.1, GAIT_DEBUG_HZ)

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

    def set_targets(self, targets: Dict[int, Tuple[float, float, float]]) -> bool:
        """Set multiple legs to explicit (x,y,z) targets (no interpolation)."""
        ok = True
        for leg_id, (x, y, z) in targets.items():
            if not self._try_set_leg_xyz(leg_id, x, y, z):
                ok = False
        if ok:
            for leg_id, xyz in targets.items():
                self.foot[leg_id] = xyz
        return ok

    def imu_stand_targets(self, roll_deg: float, pitch_deg: float, roll0_deg: float, pitch0_deg: float, dz_state: Dict[int, float]) -> Tuple[Dict[int, Tuple[float, float, float]], Dict]:
        """Compute per-leg stand targets with small Z adjustments from IMU.

        roll_deg  : filtered roll in degrees (right-down +)
        pitch_deg : filtered pitch in degrees (front-down +)
        roll0_deg/pitch0_deg : stored zero reference

        dz_state: per-leg smoothed dz state (mm), updated in-place by caller.
        """
        # Error relative to zero reference
        er = roll_deg - roll0_deg
        ep = pitch_deg - pitch0_deg

        # Desired dz components (mm). Positive dz means lifting foot (shorter leg).
        # Feedback direction (original):
        #   er>0 (right-down)  -> dz_roll>0
        #   ep>0 (front-down)  -> dz_pitch>0
        dz_roll = clampf(IMU_STAB_K_ROLL * er, -IMU_STAB_MAX_DZ, IMU_STAB_MAX_DZ)
        dz_pitch = clampf(IMU_STAB_K_PITCH * ep, -IMU_STAB_MAX_DZ, IMU_STAB_MAX_DZ)

        dbg = {
            "er": er,
            "ep": ep,
            "dz_roll": dz_roll,
            "dz_pitch": dz_pitch,
            "legs": {}
        }

        targets: Dict[int, Tuple[float, float, float]] = {}
        for leg_id in (0, 1, 2, 3):
            x, y, _ = self.stand  # keep stand x/y
            # Sign per side/front-back
            s_side = +1 if leg_id in LEFT_LEGS else -1   # LEFT legs get +, RIGHT legs get -
            s_fb = +1 if leg_id in BACK_LEGS else -1     # BACK legs get +, FRONT legs get -

            dz_cmd = s_side * dz_roll + s_fb * dz_pitch
            dz_cmd = clampf(dz_cmd, -IMU_STAB_MAX_DZ, IMU_STAB_MAX_DZ)

            # Smooth per-leg dz to avoid chatter
            prev = dz_state.get(leg_id, 0.0)
            dz_s = (1.0 - IMU_STAB_ALPHA_DZ) * prev + IMU_STAB_ALPHA_DZ * dz_cmd
            dz_state[leg_id] = dz_s

            dbg["legs"][leg_id] = {"dz_cmd": dz_cmd, "dz_s": dz_s}

            z = self.stand[2] + dz_s
            targets[leg_id] = (x, y, z)

        return targets, dbg

    def imu_z_comp_targets_current(self, dz_state: Dict[int, float], max_dz: float, z_ref: Dict[int, float] | None = None) -> Tuple[Dict[int, Tuple[float, float, float]], Dict]:
        """Compute per-leg targets keeping current x/y and adjusting ONLY z using IMU roll/pitch error.

        Uses module-level globals updated in main:
          IMU_LATEST_ROLL_DEG / IMU_LATEST_PITCH_DEG
          IMU_ZERO_ROLL_DEG / IMU_ZERO_PITCH_DEG
        z_ref: optional dict of per-leg baseline z (absolute reference for compensation)
        """
        roll_deg = IMU_LATEST_ROLL_DEG
        pitch_deg = IMU_LATEST_PITCH_DEG
        er = roll_deg - IMU_ZERO_ROLL_DEG
        ep = pitch_deg - IMU_ZERO_PITCH_DEG

        dz_roll = clampf(IMU_STAB_K_ROLL * er, -max_dz, max_dz)
        dz_pitch = clampf(IMU_STAB_K_PITCH * ep, -max_dz, max_dz)

        dbg = {"er": er, "ep": ep, "dz_roll": dz_roll, "dz_pitch": dz_pitch, "legs": {}}
        targets: Dict[int, Tuple[float, float, float]] = {}

        for leg_id in (0, 1, 2, 3):
            x, y, z_cur = self.foot[leg_id]
            z_base = z_cur if z_ref is None else z_ref.get(leg_id, z_cur)

            # Same sign convention as stand stabilization
            s_side = +1 if leg_id in LEFT_LEGS else -1   # LEFT legs get +, RIGHT legs get -
            s_fb = +1 if leg_id in BACK_LEGS else -1     # BACK legs get +, FRONT legs get -

            dz_cmd = s_side * dz_roll + s_fb * dz_pitch
            dz_cmd = clampf(dz_cmd, -max_dz, max_dz)

            prev = dz_state.get(leg_id, 0.0)
            dz_s = (1.0 - IMU_STAB_ALPHA_DZ) * prev + IMU_STAB_ALPHA_DZ * dz_cmd
            dz_state[leg_id] = dz_s

            dbg["legs"][leg_id] = {"dz_cmd": dz_cmd, "dz_s": dz_s}
            targets[leg_id] = (x, y, z_base + dz_s)

        return targets, dbg


    def imu_z_comp_targets_subset(
        self,
        leg_ids: Tuple[int, ...],
        dz_state: Dict[int, float],
        max_dz: float,
        xy_ref: Dict[int, Tuple[float, float]],
        z_ref: Dict[int, float],
    ) -> Tuple[Dict[int, Tuple[float, float, float]], Dict]:
        """Compute z-comp targets for a subset of legs using absolute z_ref and provided x/y refs.

        - Keeps x/y exactly as given in xy_ref
        - Sets z = z_ref[leg] + dz_s (absolute offset; prevents drift)
        - Uses IMU_LATEST_* and IMU_ZERO_* module-level globals
        """
        roll_deg = IMU_LATEST_ROLL_DEG
        pitch_deg = IMU_LATEST_PITCH_DEG
        er = roll_deg - IMU_ZERO_ROLL_DEG
        ep = pitch_deg - IMU_ZERO_PITCH_DEG

        dz_roll = clampf(IMU_STAB_K_ROLL * er, -max_dz, max_dz)
        dz_pitch = clampf(IMU_STAB_K_PITCH * ep, -max_dz, max_dz)

        dbg = {
            "er": er,
            "ep": ep,
            "roll_deg": roll_deg,
            "pitch_deg": pitch_deg,
            "dz_roll": dz_roll,
            "dz_pitch": dz_pitch,
            "max_dz": max_dz,
            "support": list(leg_ids),
            "legs": {},
        }
        targets: Dict[int, Tuple[float, float, float]] = {}

        for leg_id in leg_ids:
            x, y = xy_ref[leg_id]
            z_base = z_ref[leg_id]

            s_side = +1 if leg_id in LEFT_LEGS else -1   # LEFT legs get +, RIGHT legs get -
            s_fb = +1 if leg_id in BACK_LEGS else -1     # BACK legs get +, FRONT legs get -

            dz_raw = s_side * dz_roll + s_fb * dz_pitch
            dz_clamped = clampf(dz_raw, -max_dz, max_dz)

            # IMPORTANT (gait stance assist): in this robot, larger Z lifts the foot.
            # For SUPPORT legs we generally never want to LIFT a support foot (dz>0),
            # because that can reduce ground reaction force and worsen tipping.
            # So clamp to only "push down" (dz<=0).
            dz_after = dz_clamped
            clamp_pos = False
            if dz_after > 0.0:
                dz_after = 0.0
                clamp_pos = True

            prev = dz_state.get(leg_id, 0.0)
            dz_s = (1.0 - IMU_STAB_ALPHA_DZ) * prev + IMU_STAB_ALPHA_DZ * dz_after
            dz_state[leg_id] = dz_s

            dbg["legs"][leg_id] = {
                "s_side": s_side,
                "s_fb": s_fb,
                "dz_raw": dz_raw,
                "dz_clamped": dz_clamped,
                "clamp_pos": clamp_pos,
                "dz_cmd": dz_after,
                "dz_s": dz_s,
            }
            targets[leg_id] = (x, y, z_base + dz_s)

        return targets, dbg

    def go_stand(self, duration: float = 0.4):
        sx, sy, sz = self.stand
        _ = self.set_all(sx, sy, sz, duration)

    def dbg_gait(self, phase: str, swing_leg: int, cmd: Cmd, dbg: Dict | None = None):
        """Rate-limited gait debug print.

        Prints phase/swing leg/cmd and current filtered IMU roll/pitch.
        If dbg is provided (from IMU z-comp functions), also prints per-leg dz_cmd/dz_s.
        """
        if not GAIT_DEBUG_ENABLE:
            return
        now = time.time()
        if (now - self._dbg_last_t) < self._dbg_dt:
            return
        self._dbg_last_t = now

        roll_deg = IMU_LATEST_ROLL_DEG
        pitch_deg = IMU_LATEST_PITCH_DEG
        leg_nm = LEG_NAME.get(swing_leg, str(swing_leg))
        print(
            f"[GAIT] phase={phase:<7} swing={leg_nm}({swing_leg}) cmd(vx,vy,wz)=({cmd.vx:+d},{cmd.vy:+d},{cmd.wz:+d}) "
            f"imu(roll,pitch)=({roll_deg:+6.2f},{pitch_deg:+6.2f})"
        )

        if dbg is None:
            return

        # Expect dbg format: {er, ep, dz_roll, dz_pitch, legs:{leg_id:{dz_cmd,dz_s}}}
        er = dbg.get("er", 0.0)
        ep = dbg.get("ep", 0.0)
        dzr = dbg.get("dz_roll", 0.0)
        dzp = dbg.get("dz_pitch", 0.0)
        print(f"[GAIT]   imu_err(er,ep)=({er:+6.2f},{ep:+6.2f}) dz_roll={dzr:+5.2f} dz_pitch={dzp:+5.2f}")

        support = dbg.get("support", None)
        if support is not None:
            sup_names = ",".join(LEG_NAME.get(l, str(l)) for l in support)
            print(f"[GAIT]   support=[{sup_names}] max_dz={dbg.get('max_dz', 0.0):.2f}")

        legs = dbg.get("legs", {})
        if legs:
            parts = []
            for lid, info in legs.items():
                nm = LEG_NAME.get(lid, str(lid))
                s_side = info.get("s_side", 0)
                s_fb = info.get("s_fb", 0)
                dz_raw = info.get("dz_raw", 0.0)
                dz_cl = info.get("dz_clamped", 0.0)
                dz_cmd = info.get("dz_cmd", 0.0)
                dz_s = info.get("dz_s", 0.0)
                cpos = info.get("clamp_pos", False)
                parts.append(
                    f"{nm}: s({s_side:+d},{s_fb:+d}) raw={dz_raw:+5.2f} cl={dz_cl:+5.2f} cmd={dz_cmd:+5.2f}{'*' if cpos else ' '} ->{dz_s:+5.2f}"
                )
            print("[GAIT]   zcomp " + " | ".join(parts))

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

    def crawl_step(self, cmd: Cmd):
        """Execute one crawl step using current cmd."""
        if cmd.vx == 0 and cmd.vy == 0 and cmd.wz == 0:
            # no movement requested
            self.go_stand(duration=0.25)
            return

        swing_leg = CRAWL_ORDER[self.order_idx % len(CRAWL_ORDER)]
        self.order_idx += 1

        self.dbg_gait("START", swing_leg, cmd)

        sx, sy, sz = self.stand
        z_lift = sz + LIFT_DZ  # larger z lifts

        # Convert cmd into per-leg swing delta.
        # We treat commands in BODY frame first, then convert to each leg's LOCAL frame.
        body_dx = cmd.vx * STEP_FWD          # +X forward
        body_dy = cmd.vy * STEP_LAT          # +Y left
        body_dx_yaw = cmd.wz * STEP_YAW      # yaw contribution in BODY frame

        # Swing leg: BODY -> LOCAL
        body_dx_swing = body_dx + body_dx_yaw * side_sign(swing_leg)
        dx_leg = body_x_to_local_x(swing_leg, body_dx_swing)
        dy_leg = body_y_to_local_y(swing_leg, body_dy)

        # Support legs (stance): distribute opposite motion per leg in BODY frame, then map per leg to LOCAL.
        support = [i for i in (0, 1, 2, 3) if i != swing_leg]
        if support:
            body_dy_support = -body_dy / len(support)
        else:
            body_dy_support = 0.0

        # 1) SHIFT away from swing leg (body-frame Y logic depends on your mapping)
        # (kept as-is for now; IMU stabilization test runs only when STOP)
        desired_body_shift = +SHIFT_MAG if swing_leg in RIGHT_LEGS else -SHIFT_MAG
        self.shift_body(swing_leg, desired_body_shift, PHASE_T)
        self.dbg_gait("SHIFT", swing_leg, cmd)

        # IMU assist (phase-1): after SHIFT and before LIFT, apply a few Z-only compensation ticks.
        if IMU_SHIFT_ASSIST_ENABLE:
            if not hasattr(self, "_imu_dz_state"):
                self._imu_dz_state = {0: 0.0, 1: 0.0, 2: 0.0, 3: 0.0}
            # Baseline z reference for this assist window (prevents cumulative drift)
            z_ref = {leg_id: self.foot[leg_id][2] for leg_id in (0, 1, 2, 3)}
            for _ in range(max(0, IMU_SHIFT_ASSIST_TICKS)):
                targets, dbg = self.imu_z_comp_targets_current(
                    dz_state=self._imu_dz_state,
                    max_dz=IMU_SHIFT_ASSIST_MAX_DZ,
                    z_ref=z_ref,
                )
                self.set_targets(targets)
                self.dbg_gait("SHIFT_IMU", swing_leg, cmd, dbg=dbg)
                time.sleep(MOVE_DT)

        # 2) LIFT swing leg (only z)
        # While lifting, apply IMU z-compensation to STANCE legs (support legs) to reduce tipping.
        x0, y0, zS0 = self.foot[swing_leg]

        steps_lift = max(1, int(PHASE_T / MOVE_DT))
        # Absolute baseline z for the support legs during this LIFT window
        z_ref_lift = {leg_id: self.foot[leg_id][2] for leg_id in support}
        # Keep a per-driver dz smoothing state for stance assist
        if not hasattr(self, "_imu_dz_state_stance"):
            self._imu_dz_state_stance = {0: 0.0, 1: 0.0, 2: 0.0, 3: 0.0}

        self.dbg_gait("LIFT", swing_leg, cmd)
        for s in range(steps_lift + 1):
            u = s / steps_lift
            ue = smoothstep(u)

            # swing leg lift interpolation
            zi = lerp(zS0, z_lift, ue)
            if not self._try_set_leg_xyz(swing_leg, x0, y0, zi):
                self.go_stand(duration=0.3)
                return

            # stance assist on support legs (z only)
            if IMU_STANCE_ASSIST_ENABLE and support:
                xy_ref = {leg_id: (self.foot[leg_id][0], self.foot[leg_id][1]) for leg_id in support}
                targets_sup, dbg_sup = self.imu_z_comp_targets_subset(
                    leg_ids=tuple(support),
                    dz_state=self._imu_dz_state_stance,
                    max_dz=IMU_STANCE_ASSIST_MAX_DZ,
                    xy_ref=xy_ref,
                    z_ref=z_ref_lift,
                )
                self.set_targets(targets_sup)
                self.dbg_gait("LIFT_IMU", swing_leg, cmd, dbg=dbg_sup)

            time.sleep(MOVE_DT)

        # finalize swing leg state after lift
        self.foot[swing_leg] = (x0, y0, z_lift)

        # 3) SWING: move swing leg to new x/y while keeping lifted z.
        # Simultaneously move support legs a little opposite (stance).
        swing_target = (x0 + dx_leg, y0 + dy_leg, z_lift)
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

        z_ref_swing: Dict[int, float] = {}

        self.dbg_gait("SWING", swing_leg, cmd)
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

            # support legs: keep planned x/y trajectory, but apply IMU z-compensation around an absolute z_ref
            if IMU_STANCE_ASSIST_ENABLE and support:
                # absolute baseline z for this SWING window
                if s == 0:
                    z_ref_swing = {leg_id: self.foot[leg_id][2] for leg_id in support}

                xy_ref = {}
                for leg_id in support:
                    xA0, yA0, _zA0 = support_start[leg_id]
                    xAt, yAt, _zAt = support_targets[leg_id]
                    xj = lerp(xA0, xAt, ue)
                    yj = lerp(yA0, yAt, ue)
                    xy_ref[leg_id] = (xj, yj)

                targets_sup, dbg_sup = self.imu_z_comp_targets_subset(
                    leg_ids=tuple(support),
                    dz_state=self._imu_dz_state_stance,
                    max_dz=IMU_STANCE_ASSIST_MAX_DZ,
                    xy_ref=xy_ref,
                    z_ref=z_ref_swing,
                )

                self.dbg_gait("SWING_IMU", swing_leg, cmd, dbg=dbg_sup)
                for leg_id in support:
                    xj, yj, zj = targets_sup[leg_id]
                    if not self._try_set_leg_xyz(leg_id, xj, yj, zj):
                        self.go_stand(duration=0.3)
                        return
            else:
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

        self.dbg_gait("DOWN", swing_leg, cmd)
        # 4) TOUCHDOWN swing leg back to stand z (keep x/y)
        x1, y1, _ = self.foot[swing_leg]
        if not self.set_pose(swing_leg, x1, y1, sz, PHASE_T):
            self.go_stand(duration=0.3)
            return

        self.dbg_gait("UNSHIFT", swing_leg, cmd)
        # 5) UNSHIFT: remove ONLY the temporary shift (phase 1), preserving commanded y changes.
        # During shift_body() we added dy_local = body_y_to_local_y(leg_id, desired_body_shift).
        # Here we subtract the same amount, while keeping x as-is and returning z to stand.
        steps = max(1, int(PHASE_T / MOVE_DT))
        start = {i: self.foot[i] for i in (0, 1, 2, 3)}

        # Target y for each leg: current y minus the temporary shift component
        y_target = {}
        for leg_id in (0, 1, 2, 3):
            dy_local_shift = body_y_to_local_y(leg_id, desired_body_shift)
            _, y_cur, _ = self.foot[leg_id]
            y_target[leg_id] = y_cur - dy_local_shift

        for s in range(steps + 1):
            u = s / steps
            ue = smoothstep(u)
            for leg_id in (0, 1, 2, 3):
                x0, y0, z0 = start[leg_id]
                xi = x0
                yi = lerp(y0, y_target[leg_id], ue)
                zi = lerp(z0, sz, ue)
                if not self._try_set_leg_xyz(leg_id, xi, yi, zi):
                    self.go_stand(duration=0.3)
                    return
            time.sleep(MOVE_DT)

        for leg_id in (0, 1, 2, 3):
            x0, _, _ = self.foot[leg_id]
            self.foot[leg_id] = (x0, y_target[leg_id], sz)
    def reset(self):
        self.api.leg_reset()
        time.sleep(15)
    
    def shutdown(self):
        self.api.go_center_pose(debug=True)


def main():

    drv = CrawlDriver()
    drv.reset()

    # IMU filter (roll/pitch)
    imu = IMUComplementary(alpha=IMU_ALPHA)
    print(f"[IMU] Calibrating gyro bias for {IMU_CALIB_SEC:.1f}s... (keep robot still)")
    imu.calibrate_gyro(seconds=IMU_CALIB_SEC)
    print(f"[IMU] Gyro bias: gx={imu.gx_bias:+.4f} gy={imu.gy_bias:+.4f} gz={imu.gz_bias:+.4f} (rad/s)")
    print("[IMU] roll/pitch sign: roll>0 right-down, pitch>0 front-down")

    print("[INFO] Stand pose:", STAND_XYZ)
    print("[INFO] Keys: W/S/A/D/Q/E, other = stop. Ctrl+C to exit.")
    drv.go_stand(duration=0.6)

    global IMU_LATEST_ROLL_DEG, IMU_LATEST_PITCH_DEG, IMU_ZERO_ROLL_DEG, IMU_ZERO_PITCH_DEG

    cmd = Cmd(0, 0, 0)
    last_key_t = 0.0

    imu_print_dt = 1.0 / max(0.1, IMU_PRINT_HZ)
    last_imu_print = 0.0

    # Capture IMU zero reference in stand pose (average a short window)
    print(f"[IMU] Capturing zero reference for {IMU_ZERO_SEC:.1f}s in stand...")
    t0 = time.time()
    s_roll = 0.0
    s_pitch = 0.0
    n0 = 0
    while (time.time() - t0) < IMU_ZERO_SEC:
        ax, ay, az, gx, gy, gz = afb2.sensor.mpu()
        r, p = imu.update(ax, ay, az, gx, gy, gz, now=time.time())
        s_roll += rad2deg(r)
        s_pitch += rad2deg(p)
        n0 += 1
        time.sleep(0.01)
    roll0_deg = s_roll / max(1, n0)
    pitch0_deg = s_pitch / max(1, n0)
    print(f"[IMU] Zero(ref): roll0={roll0_deg:+.2f} deg  pitch0={pitch0_deg:+.2f} deg")

    IMU_ZERO_ROLL_DEG = roll0_deg
    IMU_ZERO_PITCH_DEG = pitch0_deg

    dz_state: Dict[int, float] = {0: 0.0, 1: 0.0, 2: 0.0, 3: 0.0}

    stab_print_dt = 0.5  # seconds (2 Hz)
    last_stab_print = 0.0

    with KeyReader() as kr:
        try:
            while True:
                k = kr.drain_last_key()
                now = time.time()

                # Read MPU once per loop, use the same sample for filter + debug
                ax, ay, az, gx, gy, gz = afb2.sensor.mpu()
                roll, pitch = imu.update(ax, ay, az, gx, gy, gz, now=now)
                IMU_LATEST_ROLL_DEG = rad2deg(roll)
                IMU_LATEST_PITCH_DEG = rad2deg(pitch)

                # Debug: compute accel-only tilt and gating status from the SAME sample
                amag = (ax * ax + ay * ay + az * az) ** 0.5
                use_accel = (IMU_G_MIN <= amag <= IMU_G_MAX)

                # Accel-only angles (deg) using the SAME mapping as the filter
                roll_acc_deg = rad2deg(math.atan2(ax, az))
                pitch_acc_deg = rad2deg(math.atan2(ay, az))

                if (now - last_imu_print) >= imu_print_dt:
                    last_imu_print = now
                    print(
                        f"[IMU] filt roll={IMU_LATEST_ROLL_DEG:+6.2f} pitch={IMU_LATEST_PITCH_DEG:+6.2f} | "
                        f"acc  roll={roll_acc_deg:+6.2f} pitch={pitch_acc_deg:+6.2f} | "
                        f"|a|={amag:5.2f} use_accel={use_accel} | gx={gx:+.3f} gy={gy:+.3f}"
                    )

                if k is not None:
                    cmd = key_to_cmd(k)
                    last_key_t = now

                # auto stop if no input for a while
                if (now - last_key_t) > IDLE_HOLD:
                    cmd = Cmd(0, 0, 0)

                # If STOP, run stand-only IMU stabilization test (no gait).
                if IMU_STAB_ENABLE and cmd.vx == 0 and cmd.vy == 0 and cmd.wz == 0:
                    # Use the most recent filtered angles (roll/pitch are in radians)
                    roll_deg = rad2deg(roll)
                    pitch_deg = rad2deg(pitch)
                    targets, dbg = drv.imu_stand_targets(roll_deg, pitch_deg, roll0_deg, pitch0_deg, dz_state)
                    drv.set_targets(targets)

                    if (now - last_stab_print) >= stab_print_dt:
                        last_stab_print = now
                        er = dbg["er"]
                        ep = dbg["ep"]
                        dzr = dbg["dz_roll"]
                        dzp = dbg["dz_pitch"]
                        legs = dbg["legs"]
                        # print(f"[STAB] roll_err={er:+6.2f}deg pitch_err={ep:+6.2f}deg | dz_roll={dzr:+5.2f} dz_pitch={dzp:+5.2f}")
                        for leg_id in (0, 1, 2, 3):
                            info = legs.get(leg_id, {})
                            # print(f"[STAB] leg{leg_id} dz_cmd={info.get('dz_cmd', 0.0):+5.2f} dz_s={info.get('dz_s', 0.0):+5.2f}")

                    time.sleep(MOVE_DT)
                else:
                    # execute one crawl step
                    drv.crawl_step(cmd)
                # time.sleep(0.05)

        except KeyboardInterrupt:
            print("\n[CTRL+C] Exit")
        finally:
            drv.shutdown()


if __name__ == "__main__":
    main()