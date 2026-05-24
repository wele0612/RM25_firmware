#!/usr/bin/env python3
"""
Fit a quadratic polynomial to power_profile.csv using least squares and
output the coefficients as a C header file.

Model:
    power = K0
          + K1 * current
          + K2 * speed
          + K3 * current * speed
          + K4 * current^2
          + K5 * speed^2
"""

import argparse
import csv
import os
import sys
from pathlib import Path

try:
    import numpy as np
except ImportError:
    print("numpy is required. Install it with: pip install numpy")
    sys.exit(1)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Fit a quadratic polynomial to a CSV using least squares and\noutput the coefficients as a C header file."
    )
    parser.add_argument(
        "-d", "--data", type=str, default="power_profile.csv",
        help="Input CSV file (default: power_profile.csv)"
    )
    return parser.parse_args()


def load_csv(csv_path):
    current = []
    speed = []
    power = []
    with open(csv_path, "r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            current.append(float(row["current"]))
            speed.append(float(row["speed"]))
            power.append(float(row["power"]))
    return np.array(current), np.array(speed), np.array(power)


def fit_model(current, speed, power):
    # Build design matrix: [1, I, S, I*S, I^2, S^2]
    A = np.vstack([
        np.ones_like(current),
        current,
        speed,
        current * speed,
        current ** 2,
        speed ** 2,
    ]).T

    b = power

    # Least squares solution
    coeffs, residuals, rank, s = np.linalg.lstsq(A, b, rcond=None)

    # Compute R^2
    ss_res = np.sum((b - A @ coeffs) ** 2)
    ss_tot = np.sum((b - np.mean(b)) ** 2)
    r2 = 1.0 - ss_res / ss_tot if ss_tot != 0 else float('nan')

    return coeffs, r2


def format_coefficient(val):
    """Format a float in C-friendly notation."""
    # Use repr-like precision but force lowercase e for exponent
    s = f"{val:.8g}"
    if 'e' in s:
        s = s.replace('e', 'e')
    # Ensure there's a decimal point or exponent so it parses as float
    if '.' not in s and 'e' not in s:
        s += '.0'
    return s


def write_header(coeffs, output_path):
    names = ["K0", "K1", "K2", "K3", "K4", "K5"]
    lines = ["const float POLYMODEL[]={"]
    for i, (c, name) in enumerate(zip(coeffs, names)):
        suffix = "," if i < len(coeffs) - 1 else ""
        lines.append(f"    {format_coefficient(c)}{suffix: <4}// {name}")
    lines.append("};")

    content = "\n".join(lines) + "\n"
    with open(output_path, "w", encoding="utf-8") as f:
        f.write(content)
    print(f"Header written to {output_path}")


def main():
    args = parse_args()
    script_dir = Path(__file__).parent.resolve()
    csv_path = script_dir / args.data
    header_path = script_dir / "power_model.h"

    if not csv_path.exists():
        print(f"Error: {csv_path} not found. Run plot_dataset.py first.")
        sys.exit(1)

    print(f"Loading {csv_path} ...")
    current, speed, power = load_csv(str(csv_path))
    print(f"Data points: {len(power)}")

    print("Fitting quadratic model ...")
    coeffs, r2 = fit_model(current, speed, power)

    print("\nCoefficients:")
    names = ["K0", "K1", "K2", "K3", "K4", "K5"]
    for name, c in zip(names, coeffs):
        print(f"  {name} = {c:.8g}")
    print(f"\nR^2 = {r2:.6f}")

    write_header(coeffs, str(header_path))

    # Also print to stdout in the exact C format
    print("\n--- Generated C header ---")
    with open(header_path, "r", encoding="utf-8") as f:
        print(f.read(), end="")


if __name__ == "__main__":
    main()
