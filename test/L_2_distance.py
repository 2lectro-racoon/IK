import afb2
import time

print("Distance monitor started")

while True:

    d = afb2.sensor.distance()

    if d is None:
        print("distance: NL")
    else:
        print(f"distance: {d} mm")

    time.sleep(0.05)