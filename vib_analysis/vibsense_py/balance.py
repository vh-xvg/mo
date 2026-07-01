# MIT License
#
# Copyright (c) 2026 Adrian Port
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

from __future__ import annotations

from dataclasses import dataclass, asdict
from typing import Dict, List, Tuple
import math
import cmath

from .io import read_window_csv
from .analysis import order_result, wrap_deg

@dataclass
class BalanceSolution:
    baseline_ips: float
    baseline_phase_deg: float
    trial_ips: float
    trial_phase_deg: float
    trial_weight_g: float
    trial_angle_deg: float
    influence_ips_per_g: float
    influence_phase_deg_per_g: float
    correction_weight_g: float
    correction_angle_deg: float
    residual_estimate_ips: float

    def to_dict(self) -> Dict:
        return asdict(self)


def polar_to_complex(mag: float, deg: float) -> complex:
    return mag * cmath.exp(1j * math.radians(deg))


def complex_to_mag_deg(z: complex) -> Tuple[float, float]:
    return abs(z), wrap_deg(math.degrees(cmath.phase(z)))


def solve_single_plane_from_vectors(v0: complex, v1: complex, trial_weight: complex) -> BalanceSolution:
    if abs(trial_weight) < 1e-12:
        raise ValueError("trial weight must be non-zero")
    influence = (v1 - v0) / trial_weight
    if abs(influence) < 1e-12:
        raise ValueError("trial run did not produce a useful influence coefficient")
    correction = -v0 / influence
    residual = v0 + influence * correction

    bmag, bdeg = complex_to_mag_deg(v0)
    tmag, tdeg = complex_to_mag_deg(v1)
    trial_mag, trial_deg = complex_to_mag_deg(trial_weight)
    imag, ideg = complex_to_mag_deg(influence)
    cmag, cdeg = complex_to_mag_deg(correction)

    return BalanceSolution(
        baseline_ips=bmag,
        baseline_phase_deg=bdeg,
        trial_ips=tmag,
        trial_phase_deg=tdeg,
        trial_weight_g=trial_mag,
        trial_angle_deg=trial_deg,
        influence_ips_per_g=imag,
        influence_phase_deg_per_g=ideg,
        correction_weight_g=cmag,
        correction_angle_deg=cdeg,
        residual_estimate_ips=abs(residual),
    )


def solve_single_plane(baseline_csv: str, trial_csv: str, trial_weight_g: float, trial_angle_deg: float, axis: str = "x") -> BalanceSolution:
    b = read_window_csv(baseline_csv)
    t = read_window_csv(trial_csv)
    rb = order_result(b, axis, 1.0)
    rt = order_result(t, axis, 1.0)
    v0 = polar_to_complex(rb.amp_ips, rb.phase_deg_velocity)
    v1 = polar_to_complex(rt.amp_ips, rt.phase_deg_velocity)
    w = polar_to_complex(trial_weight_g, trial_angle_deg)
    return solve_single_plane_from_vectors(v0, v1, w)


def split_weight_between_locations(weight_g: float, angle_deg: float, locations_deg: List[float]) -> List[Tuple[float, float]]:
    """Split a vector weight between the two adjacent available locations.

    Returns [(grams_at_location_a, loc_a_deg), (grams_at_location_b, loc_b_deg)].
    If the requested angle exactly matches a location, returns one non-zero weight.
    """
    if not locations_deg:
        raise ValueError("locations_deg must not be empty")
    locs = sorted([wrap_deg(x) for x in locations_deg])
    target = wrap_deg(angle_deg)
    if len(locs) == 1:
        return [(weight_g, locs[0])]

    # Find bounding pair clockwise around target.
    extended = locs + [locs[0] + 360.0]
    target_ext = target
    if target < locs[0]:
        target_ext += 360.0
    for a, b in zip(extended[:-1], extended[1:]):
        if a <= target_ext <= b:
            a0, b0 = wrap_deg(a), wrap_deg(b)
            break
    else:
        a0, b0 = locs[-1], locs[0]

    va = polar_to_complex(1.0, a0)
    vb = polar_to_complex(1.0, b0)
    vt = polar_to_complex(weight_g, target)
    # Solve ma*va + mb*vb = vt in R2.
    A = [[va.real, vb.real], [va.imag, vb.imag]]
    det = A[0][0]*A[1][1] - A[0][1]*A[1][0]
    if abs(det) < 1e-12:
        return [(weight_g, a0)]
    ma = (vt.real*A[1][1] - A[0][1]*vt.imag) / det
    mb = (A[0][0]*vt.imag - vt.real*A[1][0]) / det
    if abs(ma) < 1e-9:
        ma = 0.0
    if abs(mb) < 1e-9:
        mb = 0.0
    return [(ma, a0), (mb, b0)]
