import afb2
import time
import math

yaw = 0
prev_time = time.time()

print("YPR monitor started")

while True:

    imu = afb2.sensor.mpu()

    if imu is None:
        print("imu: NL")
        continue

    ax, ay, az, gx, gy, gz = imu

    if "NL" in imu:
        print("imu data missing")
        continue

    now = time.time()
    dt = now - prev_time
    prev_time = now

    # pitch
    pitch = math.degrees(math.atan2(-ax, math.sqrt(ay*ay + az*az)))

    # roll
    roll = math.degrees(math.atan2(ay, az))

    # yaw (gyro integration)
    yaw += math.degrees(gz * dt)

    print(f"Yaw:{yaw:.2f}  Pitch:{pitch:.2f}  Roll:{roll:.2f}")

    time.sleep(0.05)