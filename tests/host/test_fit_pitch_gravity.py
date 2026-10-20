"""验证 tools/fit_pitch_gravity.py 能从保持输出还原 gain 与 offset_rad。"""
from pathlib import Path
import math
import random
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'tools'))
from fit_pitch_gravity import fit  # noqa: E402

angles = [-0.45, -0.3, -0.1, 0.1, 0.3, 0.5, 0.7]
random.seed(1)
for gain, offset in [(-2600, 0.12), (2600, -0.3), (-1200, 0.0), (1800, 1.2)]:
    samples = [(a, gain * math.cos(a + offset)) for a in angles]
    g, o = fit(samples)
    assert abs(g - gain) < 1e-6 and abs(o - offset) < 1e-9, (gain, offset, g, o)
    # 带静摩擦量级的噪声，仍应接近真值，且 offset 不跳到 ±π 附近
    noisy = [(a, u + random.uniform(-80, 80)) for a, u in samples]
    g, o = fit(noisy)
    assert abs(g - gain) < 150 and abs(o - offset) < 0.1, (gain, offset, g, o)
    assert abs(o) <= math.pi / 2

try:
    fit([(0.2, 100.0)] * 4)
except ValueError:
    pass
else:
    raise AssertionError('角度完全相同时应报错')

print('PASS: gravity fit recovers signed gain/offset; rejects degenerate angles')
