import afb2
import time
from A_quad_api import make_default_api
from A_ik_3dof_a0 import IKError

api = make_default_api()

START_FOOT = {
    0: (120, 70, -50),
    1: (120, 70, -50),
    2: (120, 70, -50),
    3: (120, 70, -50),
}

def go_start_pose():
    for leg_id, (x, y, z) in START_FOOT.items():
        try:
            api.set_leg_xyz(leg_id, x, y, z, debug=True)
        except IKError as e:
            print(f"IK Error on leg {leg_id}: {e}")
            
afb2.gpio.reset()
time.sleep(4)
go_start_pose()

print("leg_id: 0~3 OR fr/br/bl/fl")
print("Example: fr 120 70 -20")

try:
    while True:
        s = input("cmd > ").strip()

        if s.lower() in ("q", "quit"):
            break

        try:
            leg_id_str, x, y, z = s.split()

            if leg_id_str.isdigit():
                leg_id = int(leg_id_str)
            else:
                leg_id = leg_id_str

            api.set_leg_xyz(leg_id, float(x), float(y), float(z), debug=True)

        except Exception as e:
            print("Error:", e)

finally:
    api.go_center_pose()