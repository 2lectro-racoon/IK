# calib_tool.py
from typing import List
import time

# 너 프로젝트에 맞게 import 경로만 맞춰줘
# 예) from afb2 import quad
import afb2  # 같은 폴더에 quad.py가 있다면 OK
import logging

from A_calib import load_calibration, save_calibration, Calibration


def send_neutral(neutral: List[float], calib: Calibration) -> None:
    """Send calibrated neutral pose by quad.servo()."""
    for ch, a in enumerate(neutral):
        out = calib.apply_one(ch, a)
        afb2.quad.servo(ch, out)


def main():
    logging.getLogger('werkzeug').setLevel(logging.ERROR)
    afb2.gpio.reset()
    time.sleep(8)
    afb2.flask.startServer()
    path = "calib_quad.json"
    calib = load_calibration(path)

    # 시작 중립 포즈(일단 90도)
    neutral = [40.0, 0.0, 180.0, 135.0, 180.0, 0.0, 40.0, 0.0, 180.0, 135.0, 180.0, 0.0]

    # Use the robot's reset pose as per-channel neutral center.
    # Direction flip (d) will mirror around this center, not around 90°.
    calib.center_deg = list(neutral)

    ch = 0
    step = 1.0

    print("=== Quad Calibration Tool (no quad.py edits) ===")
    print("Commands:")
    print("  ch <0-11>        select channel")
    print("  + / -            add/sub offset by step")
    print("  step <deg>       set step (float)")
    print("  d                toggle direction (+1 <-> -1)")
    print("  lim <lo> <hi>    set min/max for selected ch")
    print("  p                print selected ch calib")
    print("  r                resend neutral (calibrated)")
    print("  s                save")
    print("  q                quit")
    print("")
    send_neutral(neutral, calib)
    print(f"[sent] neutral, loaded {path}")
    print(f"[state] ch={ch}, step={step}")

    while True:
        try:
            cmd = input("calib> ").strip()
        except (EOFError, KeyboardInterrupt):
            print("\n[exit]")
            break

        if not cmd:
            continue

        if cmd in {"q", "quit", "exit"}:
            break

        if cmd in {"r", "resend"}:
            send_neutral(neutral, calib)
            continue

        if cmd in {"+", "plus"}:
            calib.offset_deg[ch] += step
            send_neutral(neutral, calib)
            continue

        if cmd in {"-", "minus"}:
            calib.offset_deg[ch] -= step
            send_neutral(neutral, calib)
            continue

        if cmd in {"d", "dir"}:
            calib.direction[ch] = -1 if calib.direction[ch] == 1 else 1
            print(f"[dir] ch{ch} => {calib.direction[ch]}")
            send_neutral(neutral, calib)
            continue

        if cmd in {"p", "print"}:
            print(
                f"ch{ch}: center={calib.center_deg[ch]:.1f}, "
                f"offset={calib.offset_deg[ch]:.2f}, "
                f"dir={calib.direction[ch]}, "
                f"lim=[{calib.min_deg[ch]:.1f},{calib.max_deg[ch]:.1f}]"
            )
            continue

        if cmd in {"s", "save"}:
            save_calibration(calib, path)
            print(f"[saved] {path}")
            continue

        parts = cmd.split()

        if parts[0] == "ch" and len(parts) == 2:
            try:
                v = int(parts[1])
                if 0 <= v < 12:
                    ch = v
                    print(f"[select] ch={ch}")
                else:
                    print("usage: ch <0-11>")
            except ValueError:
                print("usage: ch <0-11>")
            continue

        if parts[0] == "step" and len(parts) == 2:
            try:
                v = float(parts[1])
                if v <= 0:
                    raise ValueError
                step = v
                print(f"[step] {step}")
            except ValueError:
                print("usage: step <positive number>")
            continue

        if parts[0] == "lim" and len(parts) == 3:
            try:
                lo = float(parts[1])
                hi = float(parts[2])
                if hi < lo:
                    lo, hi = hi, lo
                calib.min_deg[ch] = lo
                calib.max_deg[ch] = hi
                print(f"[lim] ch{ch} => [{lo}, {hi}]")
                send_neutral(neutral, calib)
            except ValueError:
                print("usage: lim <lo> <hi>")
            continue

        print("unknown command. try: ch/+/ -/step/d/lim/r/p/s/q")

    # 종료 시 자동 저장은 취향인데, 실수 방지로 물어보지 않고 저장하자면:
    save_calibration(calib, path)
    print(f"[auto-saved] {path}")


if __name__ == "__main__":
    main()