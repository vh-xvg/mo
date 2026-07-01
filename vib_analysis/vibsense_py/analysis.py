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
import numpy as np

from .io import WindowData, counts_to_g, infer_scale

G_IN_PER_S2 = 386.08858267716535

@dataclass
class OrderResult:
    order: float
    frequency_hz: float
    rpm_mean: float
    amp_g: float
    phase_deg_accel: float
    amp_ips: float
    phase_deg_velocity: float

@dataclass
class WindowAnalysis:
    axis: str
    sample_count: int
    revolutions: int
    rpm_mean: float
    rpm_stddev: float
    order1_ips: float
    order1_phase_deg: float
    order1_accel_g: float
    harmonic_orders: List[OrderResult]

    def to_dict(self) -> Dict:
        d = asdict(self)
        d["harmonic_orders"] = [asdict(h) for h in self.harmonic_orders]
        return d


def wrap_deg(deg: float) -> float:
    deg = math.fmod(deg, 360.0)
    if deg < 0:
        deg += 360.0
    return deg


def estimate_rpm(window: WindowData) -> Tuple[float, float]:
    if window.sample_count < 2:
        return 0.0, 0.0
    tick_rate = window.tick_rate_hz
    revs = window.rev_offset.astype(float) + window.phase_deg.astype(float) / 360.0
    t = (window.tick.astype(float) - float(window.tick[0])) / tick_rate
    dt = np.diff(t)
    drev = np.diff(revs)
    good = dt > 0
    if not np.any(good):
        return 0.0, 0.0
    rpm_inst = 60.0 * drev[good] / dt[good]
    # Suppress edge artefacts and occasional duplicate samples.
    rpm_inst = rpm_inst[np.isfinite(rpm_inst)]
    if rpm_inst.size == 0:
        return 0.0, 0.0
    lo, hi = np.percentile(rpm_inst, [5, 95])
    rpm_inst = rpm_inst[(rpm_inst >= lo) & (rpm_inst <= hi)]
    return float(np.mean(rpm_inst)), float(np.std(rpm_inst))


def _axis_g(window: WindowData, axis: str) -> np.ndarray:
    range_g, bits = infer_scale(window)
    axis = axis.lower()
    if axis in ("x", "y", "z"):
        return counts_to_g(window.axis_raw(axis), range_g, bits)
    if axis == "xy":
        x = counts_to_g(window.ax, range_g, bits)
        y = counts_to_g(window.ay, range_g, bits)
        # Scalar radial-ish magnitude with sign from x. Useful for display/testing,
        # but a single physical accelerometer axis is preferred for balance solves.
        return np.hypot(x, y) * np.sign(np.where(np.abs(x) > 1e-12, x, 1.0))
    raise ValueError("axis must be x, y, z, or xy")


def order_project(window: WindowData, axis: str = "x", order: float = 1.0) -> Tuple[complex, float, float]:
    """Return complex acceleration coefficient in g, mean RPM, and RPM stddev."""
    signal_g = _axis_g(window, axis)
    signal_g = signal_g - np.mean(signal_g)
    theta = np.deg2rad(window.phase_deg + 360.0 * window.rev_offset)
    coeff = (2.0 / max(1, signal_g.size)) * np.sum(signal_g * np.exp(-1j * order * theta))
    rpm_mean, rpm_std = estimate_rpm(window)
    return complex(coeff), rpm_mean, rpm_std


def order_result(window: WindowData, axis: str, order: float) -> OrderResult:
    coeff_g, rpm_mean, _ = order_project(window, axis, order)
    amp_g = abs(coeff_g)
    accel_phase = wrap_deg(math.degrees(math.atan2(coeff_g.imag, coeff_g.real)))
    freq = (rpm_mean / 60.0) * order
    if freq > 1e-9:
        amp_ips = amp_g * G_IN_PER_S2 / (2.0 * math.pi * freq)
    else:
        amp_ips = 0.0
    # v = integral(a): divide by j*omega, i.e. -90 degrees in this convention.
    vel_phase = wrap_deg(accel_phase - 90.0)
    return OrderResult(order, freq, rpm_mean, amp_g, accel_phase, amp_ips, vel_phase)


def harmonic_table(window: WindowData, axis: str = "x", max_order: float = 6.0) -> List[OrderResult]:
    orders = [0.5, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
    return [order_result(window, axis, o) for o in orders if o <= max_order]


def analyze_window(window: WindowData, axis: str = "x", max_order: float = 6.0) -> WindowAnalysis:
    harmonics = harmonic_table(window, axis, max_order=max_order)
    h1 = min(harmonics, key=lambda h: abs(h.order - 1.0))
    rpm_mean, rpm_std = estimate_rpm(window)
    return WindowAnalysis(
        axis=axis,
        sample_count=window.sample_count,
        revolutions=window.completed_revolutions,
        rpm_mean=rpm_mean,
        rpm_stddev=rpm_std,
        order1_ips=h1.amp_ips,
        order1_phase_deg=h1.phase_deg_velocity,
        order1_accel_g=h1.amp_g,
        harmonic_orders=harmonics,
    )


def fft_spectrum(window: WindowData, axis: str = "x") -> Tuple[np.ndarray, np.ndarray, float, float]:
    """Return frequency Hz and velocity IPS spectrum from interpolated uniform samples."""
    signal_g = _axis_g(window, axis)
    signal_g = signal_g - np.mean(signal_g)
    t = (window.tick.astype(float) - float(window.tick[0])) / window.tick_rate_hz
    if t.size < 4 or t[-1] <= t[0]:
        return np.array([]), np.array([]), 0.0, 0.0
    dt = np.median(np.diff(t))
    fs = 1.0 / dt
    t_uniform = np.arange(t[0], t[-1], dt)
    y = np.interp(t_uniform, t, signal_g)
    win = np.hanning(y.size)
    yw = y * win
    spec_g = np.fft.rfft(yw)
    freqs = np.fft.rfftfreq(yw.size, d=dt)
    # Coherent-gain corrected peak amplitude spectrum in g.
    cg = np.sum(win) / y.size
    amp_g = (2.0 / y.size) * np.abs(spec_g) / max(cg, 1e-12)
    amp_ips = np.zeros_like(amp_g)
    good = freqs > 1e-9
    amp_ips[good] = amp_g[good] * G_IN_PER_S2 / (2.0 * math.pi * freqs[good])
    rpm_mean, _ = estimate_rpm(window)
    return freqs, amp_ips, fs, rpm_mean / 60.0
