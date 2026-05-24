#!/usr/bin/env python3
"""
Read all JSON files from the dataset folder, concatenate channel data by filename order,
extract three specified channels into power_profile.csv, and plot them.

Usage:
    python plot_dataset.py -c <current_ch> -s <speed_ch> -p <power_ch>
"""

import argparse
import csv
import json
import glob
import os
import sys
from pathlib import Path

try:
    import matplotlib
    import matplotlib.pyplot as plt
except ImportError:
    print("matplotlib is required. Install it with: pip install matplotlib")
    sys.exit(1)

try:
    import numpy as np
    HAS_NUMPY = True
except ImportError:
    HAS_NUMPY = False


def parse_args():
    parser = argparse.ArgumentParser(
        description="Concatenate oscilloscope JSONs and export current/speed/power CSV."
    )
    parser.add_argument(
        "-c", "--current", type=int, required=True, help="Channel ID for current"
    )
    parser.add_argument(
        "-s", "--speed", type=int, required=True, help="Channel ID for speed"
    )
    parser.add_argument(
        "-p", "--power", type=int, required=True, help="Channel ID for power"
    )
    return parser.parse_args()


def load_and_concatenate(dataset_dir: str):
    json_files = sorted(glob.glob(os.path.join(dataset_dir, "*.json")))
    if not json_files:
        print(f"No JSON files found in {dataset_dir}")
        sys.exit(1)

    print(f"Found {len(json_files)} JSON file(s):")
    for f in json_files:
        print("  ", os.path.basename(f))

    all_channels = None
    total_samples_per_channel = None

    for filepath in json_files:
        with open(filepath, "r", encoding="utf-8") as f:
            data = json.load(f)

        channels = data["channels"]  # list of lists
        n_ch = len(channels)

        if all_channels is None:
            all_channels = [[] for _ in range(n_ch)]
            total_samples_per_channel = [0] * n_ch
        else:
            if n_ch != len(all_channels):
                raise ValueError(
                    f"Channel count mismatch in {filepath}: expected {len(all_channels)}, got {n_ch}"
                )

        for i in range(n_ch):
            all_channels[i].extend(channels[i])
            total_samples_per_channel[i] += len(channels[i])

    print("\nTotal samples per channel after concatenation:")
    for i, n in enumerate(total_samples_per_channel):
        print(f"  Channel {i}: {n} samples")

    return all_channels


def write_csv(all_channels, ids, output_path):
    current_id, speed_id, power_id = ids
    n = len(all_channels[current_id])
    if len(all_channels[speed_id]) != n or len(all_channels[power_id]) != n:
        print("Warning: selected channels have different lengths. Using minimum length.")
        n = min(n, len(all_channels[speed_id]), len(all_channels[power_id]))

    print(f"\nWriting CSV -> {output_path} ({n} rows)")
    with open(output_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["current", "speed", "power"])
        for i in range(n):
            writer.writerow([
                all_channels[current_id][i],
                all_channels[speed_id][i],
                all_channels[power_id][i],
            ])
    print("Done.")


def plot_from_csv(csv_path, max_points_per_channel: int = 200_000):
    print(f"\nPlotting from {csv_path} ...")
    current = []
    speed = []
    power = []
    with open(csv_path, "r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            current.append(float(row["current"]))
            speed.append(float(row["speed"]))
            power.append(float(row["power"]))

    series = {
        "current": current,
        "speed": speed,
        "power": power,
    }

    fig, ax = plt.subplots(figsize=(14, 7))

    for name, y in series.items():
        if not y:
            continue
        original_len = len(y)
        x = list(range(original_len))

        if original_len > max_points_per_channel:
            x, y = downsample_minmax(x, y, max_points=max_points_per_channel)
            print(f"  {name}: downsampled {original_len} -> {len(y)} points")

        ax.plot(x, y, label=name, linewidth=0.7)

    ax.set_xlabel("Sample Index")
    ax.set_ylabel("Value")
    ax.set_title("Power Profile")
    ax.legend()
    ax.grid(True, linestyle="--", alpha=0.5)
    plt.tight_layout()

    # Save a PNG alongside the CSV
    png_path = str(Path(csv_path).with_suffix(".png"))
    plt.savefig(png_path, dpi=300)
    print(f"Plot saved to {png_path}")

    plt.show()


def downsample_minmax(x, y, max_points: int = 200_000):
    """Downsample by keeping min/max per bin to preserve visual envelope."""
    if len(y) <= max_points:
        return x, y

    n_bins = max_points // 2
    bin_size = max(len(y) // n_bins, 2)
    n_bins = len(y) // bin_size

    if HAS_NUMPY and isinstance(y, list):
        y = np.array(y)
        x = np.array(x) if x is not None else None

    if HAS_NUMPY and isinstance(y, np.ndarray):
        trim_len = n_bins * bin_size
        y_trim = y[:trim_len]
        y_reshape = y_trim.reshape(n_bins, bin_size)
        y_min = y_reshape.min(axis=1)
        y_max = y_reshape.max(axis=1)
        y_out = np.empty(n_bins * 2, dtype=y.dtype)
        y_out[0::2] = y_min
        y_out[1::2] = y_max

        if x is not None:
            x_trim = x[:trim_len]
            x_reshape = x_trim.reshape(n_bins, bin_size)
            x_min = x_reshape.min(axis=1)
            x_max = x_reshape.max(axis=1)
            x_out = np.empty(n_bins * 2, dtype=x.dtype)
            x_out[0::2] = x_min
            x_out[1::2] = x_max
            # sort each pair so x is monotonic within the bin
            swap = x_out[0::2] > x_out[1::2]
            x_out[0::2][swap], x_out[1::2][swap] = x_out[1::2][swap], x_out[0::2][swap]
            return x_out.tolist(), y_out.tolist()
        return None, y_out.tolist()
    else:
        y_out = []
        x_out = []
        for i in range(0, n_bins * bin_size, bin_size):
            chunk = y[i:i + bin_size]
            min_val = min(chunk)
            max_val = max(chunk)
            y_out.append(min_val)
            y_out.append(max_val)
            if x is not None:
                x_out.append(x[i + chunk.index(min_val)])
                x_out.append(x[i + chunk.index(max_val)])
        return (x_out if x is not None else None), y_out


def main():
    args = parse_args()
    script_dir = Path(__file__).parent.resolve()
    dataset_dir = script_dir / "dataset"
    csv_path = script_dir / "power_profile.csv"

    all_channels = load_and_concatenate(str(dataset_dir))
    ids = (args.current, args.speed, args.power)

    # Validate IDs
    for name, ch_id in zip(["current", "speed", "power"], ids):
        if ch_id < 0 or ch_id >= len(all_channels):
            print(f"Error: {name} channel id {ch_id} is out of range (0-{len(all_channels)-1})")
            sys.exit(1)

    write_csv(all_channels, ids, str(csv_path))
    plot_from_csv(str(csv_path))


if __name__ == "__main__":
    main()
