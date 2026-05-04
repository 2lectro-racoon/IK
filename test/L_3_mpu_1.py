import afb2
import time

print("MPU monitor started")

while True:

    imu = afb2.sensor.mpu()

    if imu is None:
        print("imu: NL")
    else:
        ax, ay, az, gx, gy, gz = imu

        print(
            f"AX:{ax}  AY:{ay}  AZ:{az} | "
            f"GX:{gx}  GY:{gy}  GZ:{gz}"
        )

    time.sleep(0.05)