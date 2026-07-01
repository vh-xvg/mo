#!/usr/bin/env python3
"""
Plot SDSEFI electrical load logs.

This is a Python 3 replacement for that reads the *.dat files written
by the logger and creates an interactive Plotly HTML file.

Typical use:
    python3 efi_plot.py 20260612_125702
    python3 efi_plot.py 20260612_125702 -o loads.html --show

The positional data directory may be omitted when running from inside a log
folder containing the log_*.dat files.
"""

from __future__ import annotations

import argparse
import sys
import webbrowser
from pathlib import Path
from typing import Iterable

import numpy as np
import plotly.graph_objects as go
from plotly.subplots import make_subplots


TRACE_FILES = [
    ("log_injector_0.dat", "Injector 1", (0.0, 1.0)),
    ("log_injector_1.dat", "Injector 2", (0.0, 1.0)),
    ("log_injector_2.dat", "Injector 3", (0.0, 1.0)),
    ("log_injector_3.dat", "Injector 4", (0.0, 1.0)),
    ("log_injector_4.dat", "Injector 5", (0.0, 1.0)),
    ("log_injector_5.dat", "Injector 6", (0.0, 1.0)),
    ("log_coil_left.dat", "Left Coilpack", (0.0, 7.0)),
    ("log_coil_right.dat", "Right Coilpack", (0.0, 7.0)),
    ("log_fuelpump_left.dat", "Left Fuelpump", (0.0, 7.0)),
    ("log_fuelpump_right.dat", "Right Fuelpump", (0.0, 7.0)),
]


def load_xy(path: Path) -> np.ndarray:
    """Load a two-column data file as an N x 2 floating-point array."""
    try:
        data = np.loadtxt(path, delimiter=",", ndmin=2)
    except ValueError:
        # Some older detail files are whitespace separated; allow that if needed.
        data = np.loadtxt(path, ndmin=2)

    data = np.asarray(data, dtype=float)
    if data.size == 0:
        raise ValueError(f"{path} is empty")
    if data.shape[1] < 2:
        raise ValueError(f"{path} does not contain at least two columns")
    return data[:, :2]


def build_figure(data_dir: Path, title: str, inj_scale: float) -> go.Figure:
    missing = [name for name, _label, _yrange in TRACE_FILES if not (data_dir / name).is_file()]
    if missing:
        missing_text = "\n  ".join(missing)
        raise FileNotFoundError(
            f"The data directory {data_dir} is missing required files:\n  {missing_text}"
        )

    fig = make_subplots(
        rows=len(TRACE_FILES),
        cols=1,
        shared_xaxes=True,
        vertical_spacing=0.02,
    )

    for row, (filename, label, yrange) in enumerate(TRACE_FILES, start=1):
        if row <= 6:
            yrange = (0.0, inj_scale)
        xy = load_xy(data_dir / filename)
        trace_mode = "markers" if "fuelpump" in filename else "lines"
        scatter_kwargs = {
            "x": xy[:, 0],
            "y": xy[:, 1],
            "name": label,
            "mode": trace_mode,
        }
        if trace_mode == "markers":
            scatter_kwargs["marker"] = {"size": 5}

        fig.add_trace(
            go.Scatter(**scatter_kwargs),
            row=row,
            col=1,
        )
        fig.update_yaxes(range=list(yrange), fixedrange=False, title_text=label, row=row, col=1)

    fig.update_xaxes(title_text="Time (s)", row=len(TRACE_FILES), col=1)
    fig.update_layout(
        title_text=title,
        title_font_size=20,
        height=1200,
        width=1600,
        hovermode="x unified",
        legend_traceorder="normal",
        margin=dict(l=90, r=30, t=70, b=60),
    )
    return fig


def parse_args(argv: Iterable[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Plot injector, coilpack, and fuel-pump log_*.dat files."
    )
    parser.add_argument(
        "data_dir",
        nargs="?",
        default=".",
        help="directory containing the log_*.dat files, default: current directory",
    )
    parser.add_argument(
        "-o",
        "--output",
        default="efi_plot.html",
        help="output HTML filename, default: electrical_loads.html",
    )
    parser.add_argument(
        "--title",
        default="SDSEFI Electrical Loads",
        help="plot title",
    )
    parser.add_argument(
        "--show",
        action="store_true",
        help="open the generated HTML file in the default web browser",
    )
    parser.add_argument(
        "-i",
        "--inj-scale",
        type=float,
        default=1.0,
        metavar="MAX",
        help="maximum Y-axis value for injector plots, default: 1.0",
    )
    return parser.parse_args(list(argv))


def main(argv: Iterable[str] = sys.argv[1:]) -> int:
    args = parse_args(argv)
    data_dir = Path(args.data_dir).expanduser().resolve()
    output = Path(args.output).expanduser()
    if not output.is_absolute():
        output = (Path.cwd() / output).resolve()

    if args.inj_scale <= 0.0:
        print("efi_plot.py: error: --inj-scale must be greater than zero", file=sys.stderr)
        return 1

    try:
        fig = build_figure(data_dir, args.title, args.inj_scale)
        fig.write_html(output, include_plotlyjs="cdn", full_html=True)
    except Exception as exc:
        print(f"efi_plot.py: error: {exc}", file=sys.stderr)
        return 1

    print(f"  Wrote {output.name}")
    if args.show:
        webbrowser.open(output.as_uri())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
