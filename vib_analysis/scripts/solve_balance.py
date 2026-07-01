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

from vibsense_py.balance import solve_single_plane, split_weight_between_locations


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--baseline", required=True)
    ap.add_argument("--trial", required=True)
    ap.add_argument("--trial-weight-g", type=float, required=True)
    ap.add_argument("--trial-angle-deg", type=float, required=True)
    ap.add_argument("--axis", default="x", choices=["x", "y", "z"])
    ap.add_argument("--locations-deg", default="", help="comma separated bolt/weight angles, optional")
    ap.add_argument("--out", default="balance_solution.json")
    args = ap.parse_args()

    sol = solve_single_plane(args.baseline, args.trial, args.trial_weight_g, args.trial_angle_deg, args.axis)
    d = sol.to_dict()
    if args.locations_deg:
        locs = [float(x) for x in args.locations_deg.split(",") if x.strip()]
        d["split_weights"] = [
            {"weight_g": w, "angle_deg": a}
            for w, a in split_weight_between_locations(sol.correction_weight_g, sol.correction_angle_deg, locs)
        ]
    with Path(args.out).open("w") as f:
        json.dump(d, f, indent=2)
    print(json.dumps(d, indent=2))
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
