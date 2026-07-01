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

import argparse, json, sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from vibsense_py.io import read_window_csv
from vibsense_py.analysis import analyze_window
from vibsense_py.plots import plot_polar, plot_spectrum


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("--axis", default="x", choices=["x", "y", "z", "xy"])
    ap.add_argument("--out", default=None, help="output prefix")
    ap.add_argument("--max-order", type=float, default=6.0)
    args = ap.parse_args()

    w = read_window_csv(args.csv)
    result = analyze_window(w, axis=args.axis, max_order=args.max_order)
    prefix = Path(args.out) if args.out else Path(args.csv).with_suffix("")
    prefix.parent.mkdir(parents=True, exist_ok=True)
    with Path(str(prefix) + "_summary.json").open("w") as f:
        json.dump(result.to_dict(), f, indent=2)
    plot_polar(w, Path(str(prefix) + "_polar.png"), axis=args.axis)
    plot_spectrum(w, Path(str(prefix) + "_spectrum.png"), axis=args.axis)
    print(json.dumps(result.to_dict(), indent=2))
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
