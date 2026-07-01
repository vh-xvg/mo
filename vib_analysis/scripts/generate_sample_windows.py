#!/usr/bin/env python3
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

import argparse
import math
from pathlib import Path
import numpy as np
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from vibsense_py.io import WindowData, write_window_csv

G_IN_PER_S2 = 386.08858267716535


def ips_to_counts(ips: float, rpm: float, bits: int, range_g: float, order: float = 1.0) -> float:
    freq = rpm / 60.0 * order
    acc_g = ips * (2.0 * math.pi * freq) / G_IN_PER_S2
    return acc_g * float(1 << (bits - 1)) / range_g


def make_window(
    path: Path,
    name: str,
    rpm: float,
    revs: int,
    sample_rate_hz: float,
    vib_ips: float,
    phase_deg: float,
    harmonic_ips: dict[float, float] | None = None,
    harmonic_phase_deg: dict[float, float] | None = None,
    noise_counts: float = 18.0,
    seed: int = 1,
    range_g: int = 16,
    bits: int = 16,
    source: int = 2,
) -> None:
    harmonic_ips = harmonic_ips or {}
    harmonic_phase_deg = harmonic_phase_deg or {}
    rng = np.random.default_rng(seed)
    rev_hz = rpm / 60.0
    duration = revs / rev_hz
    n = int(round(duration * sample_rate_hz))
    t = np.arange(n) / sample_rate_hz
    rev_float = t * rev_hz
    rev_offset = np.floor(rev_float).astype(np.uint32)
    phase_deg_arr = (rev_float - rev_offset) * 360.0
    tick = np.round(t * 1_000_000.0).astype(np.uint64)

    def axis_signal(axis_phase_shift: float = 0.0, gain: float = 1.0) -> np.ndarray:
        # Convert velocity IPS to acceleration counts. Since accel leads velocity by +90 deg,
        # synthetic acceleration phase is velocity phase + 90 deg.
        sig = np.zeros(n, dtype=float)
        acc_phase = math.radians(phase_deg + 90.0 + axis_phase_shift)
        a1 = ips_to_counts(vib_ips * gain, rpm, bits, range_g, 1.0)
        theta = 2.0 * math.pi * rev_float
        sig += a1 * np.cos(theta - acc_phase)
        for order, hips in harmonic_ips.items():
            hp = harmonic_phase_deg.get(order, phase_deg + 20.0 * order)
            a = ips_to_counts(hips * gain, rpm, bits, range_g, order)
            sig += a * np.cos(order * theta - math.radians(hp + 90.0))
        sig += 120.0 * np.sin(2.0 * math.pi * 3.7 * t)  # low unrelated wobble
        sig += rng.normal(0.0, noise_counts, n)
        return sig

    ax = np.round(axis_signal(0.0, 1.0)).astype(np.int32)
    ay = np.round(axis_signal(42.0, 0.55)).astype(np.int32)
    az = np.round(axis_signal(115.0, 0.30) + 40.0).astype(np.int32)

    w = WindowData(
        tick=tick,
        rev_offset=rev_offset,
        phase_deg=phase_deg_arr,
        ax=ax,
        ay=ay,
        az=az,
        source=np.full(n, source, dtype=np.uint8),
        range_g=np.full(n, range_g, dtype=np.uint8),
        bits=np.full(n, bits, dtype=np.uint16),
        metadata={
            "vibsense_csv_version": "2",
            "sample_name": name,
            "tick_rate_hz": "1000000",
            "rpm_mean": f"{rpm:.3f}",
            "target_revolutions": str(revs),
            "accel_source": "IIS3DWBG1" if source == 2 else "ADXL355",
            "accel_range_g": str(range_g),
            "accel_bits": str(bits),
            "axis_orientation": "x=primary_balance_axis,y=secondary,z=fore_aft",
            "optical_reference": "0_deg_top_dead_center_viewed_from_spinner_side",
            "rotation_direction": "CW_viewed_from_spinner_side",
        },
    )
    write_window_csv(path, w)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="samples")
    ap.add_argument("--rpm", type=float, default=2400.0)
    ap.add_argument("--revs", type=int, default=120)
    ap.add_argument("--sample-rate", type=float, default=6666.75)
    args = ap.parse_args()

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    # Construct baseline/trial/corrected using a simple influence model so the
    # balance solver has a coherent known answer.
    baseline_vib = 0.180
    baseline_phase = 112.0
    influence_mag = 0.0045  # IPS per gram
    influence_phase = 22.0  # response phase for weight at 0 deg
    trial_w = 20.0
    trial_angle = 90.0

    # Complex velocity vector convention.
    v0 = baseline_vib * np.exp(1j * np.deg2rad(baseline_phase))
    alpha = influence_mag * np.exp(1j * np.deg2rad(influence_phase))
    wtrial = trial_w * np.exp(1j * np.deg2rad(trial_angle))
    vtrial = v0 + alpha * wtrial
    wcorr = -v0 / alpha
    vcorr = v0 + alpha * wcorr  # nominally zero; add small residual in file

    make_window(out / "baseline.csv", "baseline", args.rpm, args.revs, args.sample_rate,
                abs(v0), np.rad2deg(np.angle(v0)) % 360,
                harmonic_ips={0.5: 0.030, 2.0: 0.055, 3.0: 0.025},
                seed=2)
    make_window(out / "trial_20g_90deg.csv", "trial_20g_90deg", args.rpm, args.revs, args.sample_rate,
                abs(vtrial), np.rad2deg(np.angle(vtrial)) % 360,
                harmonic_ips={0.5: 0.025, 2.0: 0.050, 3.0: 0.026},
                seed=3)
    make_window(out / "corrected.csv", "corrected", args.rpm, args.revs, args.sample_rate,
                0.035, 171.0,
                harmonic_ips={0.5: 0.022, 2.0: 0.048, 3.0: 0.020},
                seed=4)
    make_window(out / "rough_harmonics.csv", "rough_harmonics", 2300.0, args.revs, args.sample_rate,
                0.115, 70.0,
                harmonic_ips={0.5: 0.090, 2.0: 0.160, 3.0: 0.080, 4.0: 0.035},
                seed=5)

    # Write known answer for checking.
    with (out / "known_solution.txt").open("w") as f:
        f.write(f"Synthetic influence coefficient: {influence_mag:.6f} IPS/g @ {influence_phase:.3f} deg\n")
        f.write(f"Trial weight: {trial_w:.3f} g @ {trial_angle:.3f} deg\n")
        f.write(f"Ideal correction: {abs(wcorr):.3f} g @ {(np.rad2deg(np.angle(wcorr)) % 360):.3f} deg\n")
    print(f"Wrote sample windows to {out}")
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
