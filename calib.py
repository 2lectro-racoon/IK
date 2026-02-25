# calib.py
import json
from dataclasses import dataclass, asdict
from pathlib import Path
from typing import List, Dict, Any


def clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


@dataclass
class Calibration:
    offset_deg: List[float]
    direction: List[int]
    center_deg: List[float]
    min_deg: List[float]
    max_deg: List[float]

    @staticmethod
    def default(n: int = 12) -> "Calibration":
        return Calibration(
            offset_deg=[0.0] * n,
            direction=[1] * n,
            center_deg=[90.0] * n,
            min_deg=[0.0] * n,
            max_deg=[180.0] * n,
        )

    def validate(self, n: int = 12) -> None:
        for name in ("offset_deg", "direction", "center_deg", "min_deg", "max_deg"):
            v = getattr(self, name)
            if not isinstance(v, list) or len(v) != n:
                raise ValueError(f"{name} must be a list of length {n}")
        # direction sanity
        self.direction = [1 if int(d) >= 0 else -1 for d in self.direction]

    def apply_one(self, ch: int, angle_deg: float) -> int:
        """Return calibrated integer angle to send to quad.servo().

        Direction is applied as a mirror around 90° (neutral center), then offset and limits.
        """
        ch = int(ch)
        a = float(angle_deg)
        off = float(self.offset_deg[ch])
        d = int(self.direction[ch])
        lo = float(self.min_deg[ch])
        hi = float(self.max_deg[ch])

        # IMPORTANT:
        # Direction flip should be around a neutral center, not around 0 degrees.
        # Otherwise `d=-1` will send negative angles and clamp to 0.
        center = float(self.center_deg[ch])

        # Apply offset in the logical joint domain first,
        # then apply direction as a mirror around center.
        # This ensures +/- trim behaves consistently with direction.
        #
        # Logical angle with trim:
        #   a_logical = a + off
        #
        # Physical angle:
        #   out = center + d * (a_logical - center)
        a_logical = a + off
        a = center + d * (a_logical - center)

        a = clamp(a, lo, hi)
        return int(round(a))

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)

    @staticmethod
    def from_dict(d: Dict[str, Any], n: int = 12) -> "Calibration":
        c = Calibration.default(n)
        for k in ("offset_deg", "direction", "center_deg", "min_deg", "max_deg"):
            if k in d and isinstance(d[k], list) and len(d[k]) == n:
                setattr(c, k, d[k])
        c.validate(n)
        # coerce types
        c.offset_deg = [float(x) for x in c.offset_deg]
        c.direction = [1 if int(x) >= 0 else -1 for x in c.direction]
        c.center_deg = [float(x) for x in c.center_deg]
        c.min_deg = [float(x) for x in c.min_deg]
        c.max_deg = [float(x) for x in c.max_deg]
        return c


def load_calibration(path: str = "calib_quad.json", n: int = 12) -> Calibration:
    p = Path(path)
    if not p.exists():
        return Calibration.default(n)

    data = json.loads(p.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        return Calibration.default(n)
    return Calibration.from_dict(data, n)


def save_calibration(calib: Calibration, path: str = "calib_quad.json") -> None:
    p = Path(path)
    p.write_text(
        json.dumps(calib.to_dict(), indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )