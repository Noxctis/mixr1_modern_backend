#!/usr/bin/env python3
"""Plot thesis-friendly comparisons from generated PWM summary CSV files."""

import argparse
import csv
import math
import os

import matplotlib.pyplot as plt


def read_summary(path, label):
    rows = []
    with open(path, newline="") as handle:
        for row in csv.DictReader(handle):
            parsed = {}
            for key, value in row.items():
                if key == "condition":
                    parsed[key] = value
                else:
                    parsed[key] = float(value) if value else float("nan")
            parsed["label"] = label
            rows.append(parsed)
    return rows


def plot_metric(axis, datasets, key, ylabel, title, clean=False, log=False):
    for rows in datasets:
        pwm = [row["pwm_percent"] for row in rows]
        values = [row[key] for row in rows]
        axis.plot(pwm, values, marker="o", linewidth=2, label=rows[0]["label"])
    axis.set_title(title)
    axis.set_xlabel("PWM (%)")
    axis.set_ylabel(ylabel)
    axis.grid(alpha=0.3)
    if log:
        axis.set_yscale("log")
    axis.legend(fontsize=8)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("fifo_summary")
    parser.add_argument("no_fifo_summary")
    parser.add_argument("--output", default="fifo_vs_no_fifo_summary.png")
    args = parser.parse_args()

    datasets = [
        read_summary(args.fifo_summary, "FIFO"),
        read_summary(args.no_fifo_summary, "No FIFO"),
    ]

    figure, axes = plt.subplots(3, 2, figsize=(14, 16))
    figure.suptitle("FIFO versus No-FIFO motor and control-loop behavior", fontsize=16)

    # Motor output: mean speed and its repeatability.
    for rows in datasets:
        pwm = [row["pwm_percent"] for row in rows]
        mean_rpm = [row["mean_raw_rpm"] for row in rows]
        std_rpm = [row["std_raw_rpm"] for row in rows]
        axes[0, 0].errorbar(pwm, mean_rpm, yerr=std_rpm, marker="o", capsize=3,
                            linewidth=2, label=rows[0]["label"])
    axes[0, 0].set_title("Motor speed and RPM variation")
    axes[0, 0].set_xlabel("PWM (%)")
    axes[0, 0].set_ylabel("Measured speed (RPM)")
    axes[0, 0].grid(alpha=0.3)
    axes[0, 0].legend()
    axes[0, 0].text(0.02, 0.03, "Point = mean raw RPM\nError bar = +/- 1 standard deviation",
                    transform=axes[0, 0].transAxes, fontsize=9,
                    bbox={"facecolor": "white", "alpha": 0.8})

    plot_metric(axes[0, 1], datasets, "std_raw_rpm", "RPM", "RPM standard deviation")
    axes[0, 1].text(0.02, 0.03, "Measures variation in motor speed\nLower = more repeatable speed",
                    transform=axes[0, 1].transAxes, fontsize=9,
                    bbox={"facecolor": "white", "alpha": 0.8})

    for rows in datasets:
        pwm = [row["pwm_percent"] for row in rows]
        axes[1, 0].plot(pwm, [row["rms_period_us"] for row in rows], marker="o",
                        linewidth=2, label=f'{rows[0]["label"]} RMS')
        axes[1, 0].plot(pwm, [row["p95_abs_jitter_us"] for row in rows], marker="^",
                        linestyle="--", label=f'{rows[0]["label"]} P95')
        axes[1, 0].plot(pwm, [row["p99_abs_jitter_us"] for row in rows], marker="s",
                        linestyle=":", label=f'{rows[0]["label"]} P99')
    axes[1, 0].set_title("Control-loop timing jitter")
    axes[1, 0].set_xlabel("PWM (%)")
    axes[1, 0].set_ylabel("Absolute period deviation (us)")
    axes[1, 0].grid(alpha=0.3)
    axes[1, 0].legend(fontsize=8, ncol=2)
    axes[1, 0].text(0.02, 0.03, "Measured from loop_period_us\nrelative to the mean loop period",
                    transform=axes[1, 0].transAxes, fontsize=9,
                    bbox={"facecolor": "white", "alpha": 0.8})

    plot_metric(axes[1, 1], datasets, "mean_period_us", "Period (us)", "Mean control-loop period")
    axes[1, 1].axhline(10000.0, color="black", linestyle="--", linewidth=1)
    axes[1, 1].text(0.02, 0.03, "Time between consecutive loop iterations\nTarget period = 10,000 us",
                    transform=axes[1, 1].transAxes, fontsize=9,
                    bbox={"facecolor": "white", "alpha": 0.8})

    for rows in datasets:
        pwm = [row["pwm_percent"] for row in rows]
        axes[2, 0].plot(pwm, [row["p95_late_us"] for row in rows], marker="o",
                        linewidth=2, label=f'{rows[0]["label"]} P95 lateness')
        axes[2, 0].plot(pwm, [row["max_late_us"] for row in rows], marker="s",
                        linestyle=":", label=f'{rows[0]["label"]} maximum')
    axes[2, 0].set_title("Scheduled-loop lateness")
    axes[2, 0].set_xlabel("PWM (%)")
    axes[2, 0].set_ylabel("Delay after scheduled wake (us)")
    axes[2, 0].set_yscale("log")
    axes[2, 0].grid(alpha=0.3)
    axes[2, 0].legend(fontsize=8)
    axes[2, 0].text(0.02, 0.03, "Measures delay against the absolute wake schedule\nLog scale shows rare large delays",
                    transform=axes[2, 0].transAxes, fontsize=9,
                    bbox={"facecolor": "white", "alpha": 0.8})

    for rows in datasets:
        pwm = [row["pwm_percent"] for row in rows]
        rate = [100.0 * row["timing_outliers"] / row["samples"] for row in rows]
        axes[2, 1].plot(pwm, rate, marker="o", linewidth=2, label=rows[0]["label"])
    axes[2, 1].set_title("Timing-outlier rate")
    axes[2, 1].set_xlabel("PWM (%)")
    axes[2, 1].set_ylabel("Samples above 500 us (%)")
    axes[2, 1].grid(alpha=0.3)
    axes[2, 1].legend()
    axes[2, 1].text(0.02, 0.03, "timing_outliers / samples x 100\nLower = fewer severe interruptions",
                    transform=axes[2, 1].transAxes, fontsize=9,
                    bbox={"facecolor": "white", "alpha": 0.8})

    figure.tight_layout(rect=[0, 0, 1, 0.97])
    figure.savefig(args.output, dpi=180)
    print(f"Saved {args.output}")


if __name__ == "__main__":
    main()
