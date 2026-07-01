# vibsense Python analysis toolkit

This archive contains a small Python toolkit for analysing CSV files written by
`vibsense_write_window_csv()` from `libvibsense.a`.

It supports:

- reading plain window CSV files with columns:
  `tick,rev_offset,phase_deg,ax,ay,az,source,range_g,bits`
- reading enhanced CSV files with optional comment metadata lines beginning with `#`
- extracting 1/rev vibration magnitude and phase in IPS
- extracting harmonic vibration levels by order projection
- plotting polar vibration results
- plotting spectra with harmonic markers
- solving single-plane propeller balance corrections using baseline and trial-weight runs
- generating synthetic test CSV files

## Install dependencies

```bash
python3 -m venv .venv
. .venv/bin/activate
pip install numpy matplotlib
```

Pandas is not required.

## Generate sample data

```bash
python3 scripts/generate_sample_windows.py --out samples
```

## Analyse one window

```bash
python3 scripts/analyze_window.py samples/baseline.csv --axis x --out plots/baseline
```

This writes:

- `plots/baseline_summary.json`
- `plots/baseline_polar.png`
- `plots/baseline_spectrum.png`

## Solve a balance correction

```bash
python3 scripts/solve_balance.py \
  --baseline samples/baseline.csv \
  --trial samples/trial_20g_90deg.csv \
  --trial-weight-g 20 \
  --trial-angle-deg 90 \
  --axis x \
  --out plots/balance_solution.json
```

## Important CSV metadata recommendation

The raw `vibsense_write_window_csv()` rows are enough for basic order analysis if
the axis scale can be inferred from `source`, `range_g`, and `bits`, and if RPM
can be derived from tick and phase. For robust offline analysis, add comment
metadata lines at the top of each CSV, for example:

```text
# vibsense_csv_version: 2
# aircraft: VH-XVG
# engine: Lycoming IO-540
# sample_name: baseline
# utc_time: 2026-05-20T10:15:00Z
# tick_rate_hz: 1000000
# target_revolutions: 200
# rpm_mean: 2400.0
# accel_source: IIS3DWBG1
# accel_range_g: 16
# accel_bits: 16
# axis_orientation: x=vertical,y=lateral,z=fore-aft
# optical_reference: 0_deg_top_dead_center_viewed_from_spinner_side
# rotation_direction: CW_viewed_from_spinner_side
# notes: cowling off, warm engine, target cruise RPM
```

The parser in this toolkit accepts those comment headers but does not require them.
