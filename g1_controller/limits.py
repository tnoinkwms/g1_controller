from __future__ import annotations

from .types import JointLimit

# Axis ID -> [min_rad, max_rad]
LIMITS: dict[int, JointLimit] = {
    12: JointLimit(-2.618, 2.618),
    13: JointLimit(-0.52, 0.52),
    14: JointLimit(-0.52, 0.52),

    15: JointLimit(-3.0892, 2.6704),
    16: JointLimit(-1.5882, 2.2515),
    17: JointLimit(-2.618, 2.618),
    18: JointLimit(-1.0472, 2.0944),
    19: JointLimit(-1.9722, 1.9722),
    20: JointLimit(-1.6144, 1.6144),
    21: JointLimit(-1.6144, 1.6144),

    22: JointLimit(-3.0892, 2.6704),
    23: JointLimit(-1.5882, 2.2515),
    24: JointLimit(-2.618, 2.618),
    25: JointLimit(-1.0472, 2.0944),
    26: JointLimit(-1.9722, 1.9722),
    27: JointLimit(-1.6144, 1.6144),
    28: JointLimit(-1.6144, 1.6144),
}
