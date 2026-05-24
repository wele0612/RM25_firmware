#!/usr/bin/env python3
"""
Filter power_profile.csv by downsampling over-represented bins.

Usage:
    python power_data_profile_filter.py -n 30 -t 0.9

Arguments:
    -n  Number of bins per axis for the 2D histogram (e.g. 30 or 50)
    -t  Percentile threshold (0..1). Bins with counts above this percentile
        are randomly subsampled down to the threshold count.

Outputs:
    power_profile_filtered.csv
    current_speed_heatmap_comparison.png
"""

import argparse
import csv
import os
import sys
from pathlib import Path

try:
    import matplotlib
    import matplotlib.pyplot as plt
    import matplotlib.colors as mcolors
except ImportError:
    print("matplotlib is required. Install it with: pip install matplotlib")
    sys.exit(1)

try:
    import numpy as np
except ImportError:
    print("numpy is required. Install it with: pip install numpy")
    sys.exit(1)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Filter power profile by bin-density subsampling."
    )
    parser.add_argument(
        "-n", type=int, required=True,
        help="Number of bins per axis (e.g. 30 or 50). Must be > 0."
    )
    parser.add_argument(
        "-t", type=float, required=True,
        help="Percentile threshold (0..1, e.g. 0.9 for 90%%)."
    )
    args = parser.parse_args()

    if args.n <= 0:
        parser.error("-n must be a positive integer.")
    if not (0.0 < args.t < 1.0):
        parser.error("-t must be between 0 and 1 (exclusive).")

    return args


def load_csv(csv_path):
    current, speed, power = [], [], []
    with open(csv_path, "r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            current.append(float(row["current"]))
            speed.append(float(row["speed"]))
            power.append(float(row["power"]))
    return np.array(current), np.array(speed), np.array(power)


def write_csv(current, speed, power, csv_path):
    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["current", "speed", "power"])
        for c, s, p in zip(current, speed, power):
            writer.writerow([c, s, p])
    print(f"Filtered CSV written to {csv_path}")


def filter_data(current, speed, power, n_bins, percentile):
    # Compute 2D histogram counts and edges
    counts, xedges, yedges = np.histogram2d(current, speed, bins=n_bins)

    # Determine threshold from non-empty bins
    nonzero_counts = counts[counts > 0]
    if len(nonzero_counts) == 0:
        print("Error: no data points found.")
        sys.exit(1)

    threshold = int(np.floor(np.percentile(nonzero_counts, percentile * 100)))
    if threshold < 1:
        threshold = 1

    print(f"Total data points: {len(current)}")
    print(f"Non-empty bins: {len(nonzero_counts)} / {n_bins * n_bins}")
    print(f"Percentile threshold ({percentile * 100:.0f}%): {threshold} points per bin")

    # Map each point to its bin index
    x_bin = np.digitize(current, xedges) - 1
    y_bin = np.digitize(speed, yedges) - 1
    x_bin = np.clip(x_bin, 0, n_bins - 1)
    y_bin = np.clip(y_bin, 0, n_bins - 1)

    keep_mask = np.ones(len(current), dtype=bool)
    over_bins = np.argwhere(counts > threshold)

    discarded = 0
    for i, j in over_bins:
        # counts[i, j]: i = current bin, j = speed bin
        bin_mask = (x_bin == i) & (y_bin == j)
        indices = np.where(bin_mask)[0]
        n_keep = threshold
        kept = np.random.choice(indices, size=n_keep, replace=False)
        # discard all in this bin, then re-keep the chosen ones
        keep_mask[indices] = False
        keep_mask[kept] = True
        discarded += len(indices) - n_keep

    print(f"Bins above threshold: {len(over_bins)}")
    print(f"Discarded points: {discarded}")
    print(f"Retained points: {keep_mask.sum()}")

    return current[keep_mask], speed[keep_mask], power[keep_mask]


def plot_comparison(current, speed, current_f, speed_f, n_bins, output_path):
    fig, axes = plt.subplots(1, 2, figsize=(14, 6))

    # Compute shared color scale
    H1, _, _ = np.histogram2d(current, speed, bins=n_bins)
    H2, _, _ = np.histogram2d(current_f, speed_f, bins=n_bins)
    vmax = max(H1.max(), H2.max())
    vmin = 1
    norm = mcolors.LogNorm(vmin=vmin, vmax=vmax)

    im = None
    for ax, (c, s), title in zip(
        axes,
        [(current, speed), (current_f, speed_f)],
        ["Original", "Filtered"]
    ):
        _, _, _, im = ax.hist2d(c, s, bins=n_bins, cmap="hot", norm=norm)
        ax.set_xlabel("Current")
        ax.set_ylabel("Speed")
        ax.set_title(title)

    cbar = fig.colorbar(im, ax=axes, orientation="vertical", fraction=0.02, pad=0.04)
    cbar.set_label("Sample Count")

    plt.tight_layout()
    plt.savefig(output_path, dpi=300)
    print(f"Comparison heatmap saved to {output_path}")


def main():
    args = parse_args()
    script_dir = Path(__file__).parent.resolve()
    input_csv = script_dir / "power_profile.csv"
    output_csv = script_dir / "power_profile_filtered.csv"
    output_png = script_dir / "current_speed_heatmap_comparison.png"

    if not input_csv.exists():
        print(f"Error: {input_csv} not found.")
        sys.exit(1)

    print(f"Loading {input_csv} ...")
    current, speed, power = load_csv(str(input_csv))

    current_f, speed_f, power_f = filter_data(
        current, speed, power, args.n, args.t
    )

    write_csv(current_f, speed_f, power_f, str(output_csv))
    plot_comparison(current, speed, current_f, speed_f, args.n, str(output_png))


if __name__ == "__main__":
    main()
