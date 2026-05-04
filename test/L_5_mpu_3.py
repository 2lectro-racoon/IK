import afb2
import time
import math

# -----------------------------
# Filter parameter
# -----------------------------
ALPHA = 0.98

# orientation state
roll = 0.0
pitch = 0.0
yaw = 0.0

# gyro bias
gx_bias = 0.0
gy_bias = 0.0
gz_bias = 0.0

# -----------------------------
# Gyro Calibration
# -----------------------------
print("MPU Gyro calibration... Keep robot still")

samples = 200
sx = sy = sz = 0.0

for i in range(samples):

    ax, ay, az, gx, gy, gz = afb2.sensor.mpu()

    sx += gx
    sy += gy
    sz += gz

    time.sleep(0.01)

gx_bias = sx / samples
gy_bias = sy / samples
gz_bias = sz / samples

print("Calibration complete")
print("gx_bias:", gx_bias)
print("gy_bias:", gy_bias)
print("gz_bias:", gz_bias)

# -----------------------------
# Init orientation from accel
# -----------------------------
ax, ay, az, gx, gy, gz = afb2.sensor.mpu()

roll = math.atan2(ax, az)
pitch = math.atan2(ay, az)

prev_time = time.time()

print("MPU monitor started")

# -----------------------------
# Main loop
# -----------------------------
while True:

    ax, ay, az, gx, gy, gz = afb2.sensor.mpu()

    now = time.time()
    dt = now - prev_time
    prev_time = now

    # remove gyro bias
    gx -= gx_bias
    gy -= gy_bias
    gz -= gz_bias

    # -----------------------------
    # gyro integration
    # -----------------------------
    roll_gyro = roll + gy * dt
    pitch_gyro = pitch + gx * dt
    yaw += gz * dt

    # -----------------------------
    # accel tilt
    # -----------------------------
    roll_acc = math.atan2(ax, az)
    pitch_acc = math.atan2(ay, az)

    # -----------------------------
    # complementary filter
    # -----------------------------
    roll = ALPHA * roll_gyro + (1 - ALPHA) * roll_acc
    pitch = ALPHA * pitch_gyro + (1 - ALPHA) * pitch_acc

    # rad → deg
    roll_deg = roll * 180 / math.pi
    pitch_deg = pitch * 180 / math.pi
    yaw_deg = yaw * 180 / math.pi

    print(
        f"Roll:{roll_deg:7.2f}  "
        f"Pitch:{pitch_deg:7.2f}  "
        f"Yaw:{yaw_deg:7.2f}"
    )

    time.sleep(0.02)