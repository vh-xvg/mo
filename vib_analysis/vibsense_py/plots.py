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

from pathlib import Path
import math
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from .io import WindowData
from .analysis import analyze_window, fft_spectrum


def plot_polar(window: WindowData, out_png: str | Path, axis: str = "x") -> None:
    result = analyze_window(window, axis=axis)
    theta = math.radians(result.order1_phase_deg)
    r = result.order1_ips
    fig = plt.figure(figsize=(5, 5))
    ax = fig.add_subplot(111, projection="polar")
    ax.set_theta_zero_location("N")
    ax.set_theta_direction(-1)
    max_r = max(0.25, r * 1.25)
    ax.set_rlim(0, max_r)
    ax.plot([theta], [r], marker="o", markersize=10)
    ax.plot([0, theta], [0, r], linewidth=1)
    ax.set_title(f"1/rev vibration: {r:.3f} IPS @ {result.order1_phase_deg:.0f}°\nRPM {result.rpm_mean:.0f}, axis {axis}")
    fig.tight_layout()
    fig.savefig(out_png, dpi=140)
    plt.close(fig)


def plot_spectrum(window: WindowData, out_png: str | Path, axis: str = "x", max_hz: float | None = None) -> None:
    freqs, ips, fs, rev_hz = fft_spectrum(window, axis=axis)
    if freqs.size == 0:
        raise ValueError("not enough samples for spectrum")
    if max_hz is None:
        max_hz = min(freqs[-1], max(300.0, rev_hz * 8.0))
    mask = freqs <= max_hz
    fig, ax = plt.subplots(figsize=(9, 4.5))
    ax.plot(freqs[mask], ips[mask], linewidth=1)
    if rev_hz > 0:
        for order in [0.5, 1, 2, 3, 4, 5, 6]:
            f = rev_hz * order
            if f <= max_hz:
                ax.axvline(f, linestyle="--", linewidth=0.8)
                ax.text(f, ax.get_ylim()[1] * 0.85, f"{order:g}x", rotation=90, va="top", ha="right")
    ax.set_xlabel("Frequency (Hz)")
    ax.set_ylabel("Velocity amplitude (IPS)")
    ax.set_title(f"Velocity spectrum, axis {axis}")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(out_png, dpi=140)
    plt.close(fig)
