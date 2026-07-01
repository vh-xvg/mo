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

from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, Iterable, List, Tuple
import csv
import numpy as np

REQUIRED_COLUMNS = ["tick", "rev_offset", "phase_deg", "ax", "ay", "az", "source", "range_g", "bits"]

@dataclass
class WindowData:
    tick: np.ndarray
    rev_offset: np.ndarray
    phase_deg: np.ndarray
    ax: np.ndarray
    ay: np.ndarray
    az: np.ndarray
    source: np.ndarray
    range_g: np.ndarray
    bits: np.ndarray
    metadata: Dict[str, str] = field(default_factory=dict)

    @property
    def phase_rad(self) -> np.ndarray:
        return np.deg2rad(self.phase_deg)

    @property
    def sample_count(self) -> int:
        return int(self.tick.size)

    @property
    def completed_revolutions(self) -> int:
        if self.rev_offset.size == 0:
            return 0
        return int(np.max(self.rev_offset) + 1)

    @property
    def tick_rate_hz(self) -> float:
        if "tick_rate_hz" in self.metadata:
            try:
                return float(self.metadata["tick_rate_hz"])
            except ValueError:
                pass
        # Current STM32 design uses TIM2 at 1 MHz; use as default if absent.
        return 1_000_000.0

    def axis_raw(self, axis: str) -> np.ndarray:
        axis = axis.lower()
        if axis == "x":
            return self.ax.astype(float)
        if axis == "y":
            return self.ay.astype(float)
        if axis == "z":
            return self.az.astype(float)
        raise ValueError(f"Unsupported single axis {axis!r}; use x, y, or z")


def _parse_metadata_line(line: str) -> Tuple[str, str] | None:
    line = line[1:].strip()
    if not line or ":" not in line:
        return None
    key, value = line.split(":", 1)
    return key.strip(), value.strip()


def read_window_csv(path: str | Path) -> WindowData:
    path = Path(path)
    metadata: Dict[str, str] = {}
    rows: List[Dict[str, str]] = []

    with path.open("r", newline="") as f:
        non_comment_lines: List[str] = []
        for line in f:
            if line.startswith("#"):
                item = _parse_metadata_line(line)
                if item:
                    metadata[item[0]] = item[1]
            elif line.strip():
                non_comment_lines.append(line)

    if not non_comment_lines:
        raise ValueError(f"No CSV data rows in {path}")

    reader = csv.DictReader(non_comment_lines)
    missing = [c for c in REQUIRED_COLUMNS if c not in (reader.fieldnames or [])]
    if missing:
        raise ValueError(f"{path} missing required columns: {missing}")

    for row in reader:
        rows.append(row)

    def arr(name: str, dtype):
        return np.asarray([r[name] for r in rows], dtype=dtype)

    return WindowData(
        tick=arr("tick", np.uint64),
        rev_offset=arr("rev_offset", np.uint32),
        phase_deg=arr("phase_deg", float),
        ax=arr("ax", np.int32),
        ay=arr("ay", np.int32),
        az=arr("az", np.int32),
        source=arr("source", np.uint8),
        range_g=arr("range_g", np.uint8),
        bits=arr("bits", np.uint16),
        metadata=metadata,
    )


def write_window_csv(path: str | Path, window: WindowData, metadata: Dict[str, str] | None = None) -> None:
    path = Path(path)
    md = dict(window.metadata)
    if metadata:
        md.update(metadata)
    with path.open("w", newline="") as f:
        for key in sorted(md.keys()):
            f.write(f"# {key}: {md[key]}\n")
        writer = csv.writer(f)
        writer.writerow(REQUIRED_COLUMNS)
        for i in range(window.sample_count):
            writer.writerow([
                int(window.tick[i]), int(window.rev_offset[i]), f"{float(window.phase_deg[i]):.9f}",
                int(window.ax[i]), int(window.ay[i]), int(window.az[i]),
                int(window.source[i]), int(window.range_g[i]), int(window.bits[i]),
            ])


def counts_to_g(raw: np.ndarray, range_g: float, bits: int) -> np.ndarray:
    # For signed two's complement accelerometer counts spanning +/- range_g.
    return raw.astype(float) * float(range_g) / float(1 << (int(bits) - 1))


def infer_scale(window: WindowData) -> Tuple[float, int]:
    if window.sample_count == 0:
        return 16.0, 16
    rg = int(np.median(window.range_g))
    bits = int(np.median(window.bits))
    return float(rg), bits
