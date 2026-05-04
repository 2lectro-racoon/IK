# imu_stabilizer_only_pid_afb2.py
# ==========================================
# Quadruped IMU Stabilizer Only (PID Version)
# - AFB2 버전
# ==========================================

import time
import math
import afb2

from A_quad_api import make_default_api
from A_ik_3dof_a0 import IKError


# ==========================================
# 기본 자세
# ==========================================
STAND_X = 120
STAND_Y = 70
STAND_Z = -50

MOVE_DT = 0.03


# ==========================================
# IMU 파라미터
# ==========================================
IMU_ALPHA = 0.99
IMU_CALIB_SEC = 2.0

IMU_STAB_MAX_DZ = 40

# PID gains
IMU_KP_ROLL = 1.4
IMU_KP_PITCH = 3.2

IMU_KI_ROLL = 0.00
IMU_KI_PITCH = 0.01

IMU_KD_ROLL = 0.0
IMU_KD_PITCH = 0.1

IMU_I_LIMIT = 10


RIGHT_LEGS = {0,1}
LEFT_LEGS = {2,3}

FRONT_LEGS = {0,3}
BACK_LEGS = {1,2}


# ==========================================
# 보조 함수
# ==========================================
def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


def rad2deg(r):
    return r * 180 / math.pi


# ==========================================
# IMU Filter
# ==========================================
class IMUFilter:

    def __init__(self):
        self.roll = 0
        self.pitch = 0

        self.gx_bias = 0
        self.gy_bias = 0
        self.gz_bias = 0

        self.prev = None


    def calibrate(self):

        print("IMU calibration (움직이지 마세요)...")

        sx = sy = sz = 0
        n = 200

        for _ in range(n):
            ax, ay, az, gx, gy, gz = afb2.sensor.mpu()

            sx += gx
            sy += gy
            sz += gz

            time.sleep(0.01)

        self.gx_bias = sx / n
        self.gy_bias = sy / n
        self.gz_bias = sz / n

        print("calibration done")


    def update(self):

        ax, ay, az, gx, gy, gz = afb2.sensor.mpu()

        now = time.time()

        if self.prev is None:
            self.prev = now

            self.roll = math.atan2(ax, az)
            self.pitch = math.atan2(ay, az)

            return self.roll, self.pitch

        dt = now - self.prev
        self.prev = now

        gx -= self.gx_bias
        gy -= self.gy_bias

        roll_gyro = self.roll + gy * dt
        pitch_gyro = self.pitch + gx * dt

        roll_acc = math.atan2(ax, az)
        pitch_acc = math.atan2(ay, az)

        a = IMU_ALPHA

        self.roll = a * roll_gyro + (1 - a) * roll_acc
        self.pitch = a * pitch_gyro + (1 - a) * pitch_acc

        return self.roll, self.pitch


# ==========================================
# Stabilizer
# ==========================================
class Stabilizer:

    def __init__(self):

        self.api = make_default_api()

        self.foot = {
            0: (STAND_X, STAND_Y, STAND_Z),
            1: (STAND_X, STAND_Y, STAND_Z),
            2: (STAND_X, STAND_Y, STAND_Z),
            3: (STAND_X, STAND_Y, STAND_Z)
        }

        # PID state
        self.roll_i = 0
        self.pitch_i = 0

        self.prev_er = 0
        self.prev_ep = 0

        self.prev_t = None


    def set_leg(self, leg, x, y, z):

        try:
            self.api.set_leg_xyz(leg, x, y, z)
        except IKError:
            pass


    def go_stand(self):

        for leg in range(4):
            self.set_leg(leg, STAND_X, STAND_Y, STAND_Z)


    def stabilize(self, roll_deg, pitch_deg, roll0, pitch0):

        er = roll_deg - roll0
        ep = pitch_deg - pitch0

        now = time.time()

        if self.prev_t is None:
            self.prev_t = now

        dt = now - self.prev_t
        self.prev_t = now

        # ---------- Integral ----------
        self.roll_i += er * dt
        self.pitch_i += ep * dt

        self.roll_i = clamp(self.roll_i, -IMU_I_LIMIT, IMU_I_LIMIT)
        self.pitch_i = clamp(self.pitch_i, -IMU_I_LIMIT, IMU_I_LIMIT)

        # ---------- Derivative ----------
        der_roll = (er - self.prev_er) / dt if dt > 0 else 0
        der_pitch = (ep - self.prev_ep) / dt if dt > 0 else 0

        self.prev_er = er
        self.prev_ep = ep

        # ---------- PID ----------
        dz_roll = (
            IMU_KP_ROLL * er
            + IMU_KI_ROLL * self.roll_i
            + IMU_KD_ROLL * der_roll
        )

        dz_pitch = (
            IMU_KP_PITCH * ep
            + IMU_KI_PITCH * self.pitch_i
            + IMU_KD_PITCH * der_pitch
        )

        dz_roll = clamp(dz_roll, -IMU_STAB_MAX_DZ, IMU_STAB_MAX_DZ)
        dz_pitch = clamp(dz_pitch, -IMU_STAB_MAX_DZ, IMU_STAB_MAX_DZ)

        for leg in range(4):

            x, y, _ = self.foot[leg]

            side = 1 if leg in LEFT_LEGS else -1
            fb   = 1 if leg in BACK_LEGS else -1

            dz = side * dz_roll + fb * dz_pitch

            dz = clamp(dz, -IMU_STAB_MAX_DZ, IMU_STAB_MAX_DZ)

            z = STAND_Z + dz

            self.set_leg(leg, x, y, z)


# ==========================================
# main
# ==========================================
def main():

    api = make_default_api()

    api.leg_reset()
    time.sleep(4)

    imu = IMUFilter()
    imu.calibrate()

    stab = Stabilizer()

    stab.go_stand()

    print("IMU PID stabilizer running")

    # 기준 자세
    r0, p0 = imu.update()

    roll0 = rad2deg(r0)
    pitch0 = rad2deg(p0)

    while True:

        r, p = imu.update()

        roll = rad2deg(r)
        pitch = rad2deg(p)

        stab.stabilize(roll, pitch, roll0, pitch0)

        print(f"roll={roll:+5.2f} pitch={pitch:+5.2f}")

        time.sleep(MOVE_DT)


if __name__ == "__main__":
    main()