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

import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def run(cmd):
    print("+", " ".join(str(c) for c in cmd))
    subprocess.check_call([str(c) for c in cmd], cwd=ROOT)


def main() -> int:
    sample_dir = ROOT / "samples_test"
    plot_dir = ROOT / "plots_test"
    sample_dir.mkdir(exist_ok=True)
    plot_dir.mkdir(exist_ok=True)
    run([sys.executable, "scripts/generate_sample_windows.py", "--out", sample_dir])
    run([sys.executable, "scripts/analyze_window.py", sample_dir / "baseline.csv", "--out", plot_dir / "baseline"])
    run([
        sys.executable, "scripts/solve_balance.py",
        "--baseline", sample_dir / "baseline.csv",
        "--trial", sample_dir / "trial_20g_90deg.csv",
        "--trial-weight-g", "20",
        "--trial-angle-deg", "90",
        "--locations-deg", "0,30,60,90,120,150,180,210,240,270,300,330",
        "--out", plot_dir / "solution.json",
    ])
    d = json.loads((plot_dir / "solution.json").read_text())
    assert d["correction_weight_g"] > 1.0
    assert 0.0 <= d["correction_angle_deg"] < 360.0
    print("OK")
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
