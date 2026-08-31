#!/usr/bin/env python3
"""Plot thesis-friendly comparisons from generated PWM summary CSV files."""

import argparse
import csv
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


def plot_metric(datasets, key, ylabel, title, output_path, x_key, xlabel, log=False):
    figure, axis = plt.subplots(figsize=(10, 6))
    for rows in datasets:
        pwm = [row[x_key] for row in rows]
        values = [row[key] for row in rows]
        axis.plot(pwm, values, marker="o", linewidth=2, label=rows[0]["label"])
    axis.set_title(title)
    axis.set_xlabel(xlabel)
    axis.set_ylabel(ylabel)
    axis.grid(alpha=0.3)
    if log:
        axis.set_yscale("log")
    axis.legend()
    figure.tight_layout()
    figure.savefig(output_path, dpi=180)
    plt.close(figure)


def write_report(path, datasets, mode):
    fifo, no_fifo = datasets

    def mean_value(rows, key):
        return sum(row[key] for row in rows) / len(rows)

    fifo_rms = mean_value(fifo, "rms_period_us")
    no_fifo_rms = mean_value(no_fifo, "rms_period_us")
    fifo_p99 = mean_value(fifo, "p99_abs_jitter_us")
    no_fifo_p99 = mean_value(no_fifo, "p99_abs_jitter_us")
    fifo_max_late = max(row["max_late_us"] for row in fifo)
    no_fifo_max_late = max(row["max_late_us"] for row in no_fifo)
    fifo_outliers = sum(row["timing_outliers"] for row in fifo)
    no_fifo_outliers = sum(row["timing_outliers"] for row in no_fifo)
    total_samples = sum(row["samples"] for row in fifo)

    with open(path, "w") as handle:
        if mode == "pi":
            handle.write("# PI FIFO and PI No-FIFO Thesis Graphs\n\n")
            handle.write("The experiment applies the same target-RPM steps to the PI-controlled motor under FIFO and No-FIFO scheduling. Each graph uses per-target summary values calculated from the raw measurements.\n\n")
        else:
            handle.write("# FIFO and No-FIFO Thesis Graphs\n\n")
            handle.write("The experiment applies the same PWM sweep to the motor under FIFO and No-FIFO scheduling. Each graph uses the per-PWM summary values calculated from the raw measurements.\n\n")
        handle.write("## Motor speed and RPM variation\n\n")
        if mode == "pi":
            handle.write("The mean speed graph shows measured RPM against the requested target RPM. The error bars show plus or minus one RPM standard deviation. The distance between the measured speed and the target represents tracking error; the error bars represent speed repeatability.\n\n")
            handle.write("The RPM standard-deviation graph measures variation in the motor output around its mean speed. It is calculated from raw RPM samples and is separate from timing jitter. Lower values indicate more repeatable PI-controlled speed.\n\n")
            handle.write("## Initial IMC-based controller\n\n")
            handle.write("The PI gains were selected from the initial motor-model curve fit using an IMC design. The fitted model estimates the motor gain and time constant; these determine the proportional and integral gains. The same gain schedule is used for FIFO and No-FIFO, so the scheduling comparison changes execution timing rather than the controller tuning. The gains are interpolated according to the target RPM.\n\n")
            handle.write("The mean PI steady-state absolute tracking error is approximately 5.87 RPM with FIFO and 6.08 RPM without FIFO. The similar errors indicate that both conditions can track the target, while the timing graphs show the difference in execution determinism.\n\n")
        else:
            handle.write("The mean speed graph shows the average motor RPM produced at each PWM level. The error bars show plus or minus one RPM standard deviation. The mean response is the motor's average output; the error bars show repeatability. Larger error bars mean the motor speed changes more during the same test condition.\n\n")
            handle.write("The RPM standard-deviation graph measures motor-output variation directly. It is calculated from the individual raw RPM samples within each PWM step. It is not the same as timing jitter. In this experiment, No-FIFO reaches approximately 154 RPM standard deviation at 100% PWM, compared with approximately 75 RPM for FIFO, showing less repeatable motor speed without FIFO.\n\n")
        handle.write("## Control-loop timing\n\n")
        handle.write("The mean control-loop period is the average time between consecutive loop executions. The intended period is 10,000 microseconds, or 10 milliseconds. Both conditions have a mean close to this value, so the average loop rate is maintained. The mean alone does not reveal occasional delays.\n\n")
        handle.write("Timing jitter is calculated from the measured loop_period_us values. For each sample, the actual interval between loop executions is compared with the typical period. RMS jitter summarizes overall timing variation; P95 and P99 show the deviation below which 95% and 99% of samples fall. Lower values indicate more deterministic controller execution. Average RMS jitter is {:.2f} us for FIFO and {:.2f} us for No-FIFO. Average P99 jitter is {:.2f} us for FIFO and {:.2f} us for No-FIFO.\n\n".format(fifo_rms, no_fifo_rms, fifo_p99, no_fifo_p99))
        handle.write("Scheduled-loop lateness is different from jitter. It measures how late the loop executes compared with its absolute scheduled wake-up time. P95 lateness represents typical delayed behavior, while maximum lateness shows the worst interruption. FIFO's maximum lateness is approximately {:.0f} us; No-FIFO reaches approximately {:.0f} us.\n\n".format(fifo_max_late, no_fifo_max_late))
        handle.write("The timing-outlier-rate graph counts samples whose lateness exceeds 500 us. FIFO has {} outliers out of {} samples ({:.2f}%). No-FIFO has {} outliers ({:.2f}%). These outliers represent severe scheduling interruptions that can delay feedback processing and motor updates.\n\n".format(fifo_outliers, total_samples, 100.0 * fifo_outliers / total_samples, no_fifo_outliers, 100.0 * no_fifo_outliers / total_samples))
        handle.write("## Thesis conclusion\n\n")
        if mode == "pi":
            handle.write("The IMC-based PI controller achieves similar average tracking accuracy in both tests, with mean steady-state errors of 5.87 RPM for FIFO and 6.08 RPM for No-FIFO. FIFO provides the more deterministic execution schedule, with lower timing variation and fewer severe delays. This supports using FIFO to improve the repeatability of PI control without changing the controller gains.\n")
        else:
            handle.write("FIFO does not substantially change the average 10 ms loop rate, but it greatly reduces timing variation and worst-case delays. The lower timing variation is accompanied by lower RPM variation, indicating more repeatable motor behavior. No-FIFO maintains a similar average loop period while allowing large timing interruptions, so average period alone would hide the real-time performance difference.\n")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("fifo_summary")
    parser.add_argument("no_fifo_summary")
    parser.add_argument("--output", default="fifo_vs_no_fifo_graphs")
    parser.add_argument("--mode", choices=["open-loop", "pi"], default="open-loop")
    args = parser.parse_args()

    datasets = [
        read_summary(args.fifo_summary, "FIFO"),
        read_summary(args.no_fifo_summary, "No FIFO"),
    ]

    os.makedirs(args.output, exist_ok=True)

    x_key = "target_rpm" if args.mode == "pi" else "pwm_percent"
    xlabel = "Target RPM" if args.mode == "pi" else "PWM (%)"

    figure, axis = plt.subplots(figsize=(10, 6))
    for rows in datasets:
        pwm = [row[x_key] for row in rows]
        mean_rpm = [row["mean_raw_rpm"] for row in rows]
        std_rpm = [row["std_raw_rpm"] for row in rows]
        axis.errorbar(pwm, mean_rpm, yerr=std_rpm, marker="o", capsize=3,
                      linewidth=2, label=rows[0]["label"])
    axis.set_title("PI motor speed and RPM variation: FIFO versus No-FIFO" if args.mode == "pi" else "Motor speed and RPM variation: FIFO versus No-FIFO")
    axis.set_xlabel(xlabel)
    axis.set_ylabel("Measured speed (RPM)")
    axis.grid(alpha=0.3)
    axis.legend()
    figure.tight_layout()
    figure.savefig(os.path.join(args.output, "motor_speed_fifo_vs_no_fifo.png"), dpi=180)
    plt.close(figure)

    plot_metric(datasets, "std_raw_rpm", "RPM standard deviation", "RPM standard deviation", os.path.join(args.output, "rpm_standard_deviation.png"), x_key, xlabel)
    figure, axis = plt.subplots(figsize=(10, 6))
    for rows in datasets:
        pwm = [row[x_key] for row in rows]
        axis.plot(pwm, [row["rms_period_us"] for row in rows], marker="o", linewidth=2, label=f'{rows[0]["label"]} RMS')
        axis.plot(pwm, [row["p95_abs_jitter_us"] for row in rows], marker="^", linestyle="--", label=f'{rows[0]["label"]} P95')
        axis.plot(pwm, [row["p99_abs_jitter_us"] for row in rows], marker="s", linestyle=":", label=f'{rows[0]["label"]} P99')
    axis.set_title("Control-loop timing jitter")
    axis.set_xlabel(xlabel)
    axis.set_ylabel("Absolute period deviation (us)")
    axis.grid(alpha=0.3)
    axis.legend(fontsize=8, ncol=2)
    figure.tight_layout()
    figure.savefig(os.path.join(args.output, "control_loop_timing_jitter.png"), dpi=180)
    plt.close(figure)
    plot_metric(datasets, "mean_period_us", "Mean period (us)", "Mean control-loop period", os.path.join(args.output, "mean_control_loop_period.png"), x_key, xlabel)
    figure, axis = plt.subplots(figsize=(10, 6))
    for rows in datasets:
        pwm = [row[x_key] for row in rows]
        axis.plot(pwm, [row["p95_late_us"] for row in rows], marker="o", linewidth=2, label=f'{rows[0]["label"]} P95')
        axis.plot(pwm, [row["max_late_us"] for row in rows], marker="s", linestyle=":", label=f'{rows[0]["label"]} maximum')
    axis.set_title("Scheduled-loop lateness")
    axis.set_xlabel(xlabel)
    axis.set_ylabel("Delay after scheduled wake (us)")
    axis.set_yscale("log")
    axis.grid(alpha=0.3)
    axis.legend(fontsize=8)
    figure.tight_layout()
    figure.savefig(os.path.join(args.output, "scheduled_loop_lateness.png"), dpi=180)
    plt.close(figure)

    figure, axis = plt.subplots(figsize=(10, 6))
    for rows in datasets:
        pwm = [row[x_key] for row in rows]
        rate = [100.0 * row["timing_outliers"] / row["samples"] for row in rows]
        axis.plot(pwm, rate, marker="o", linewidth=2, label=rows[0]["label"])
    axis.set_title("Timing-outlier rate")
    axis.set_xlabel(xlabel)
    axis.set_ylabel("Samples above 500 us (%)")
    axis.grid(alpha=0.3)
    axis.legend()
    figure.tight_layout()
    figure.savefig(os.path.join(args.output, "timing_outlier_rate.png"), dpi=180)
    plt.close(figure)

    report_path = os.path.join(args.output, "fifo_vs_no_fifo_report.md")
    write_report(report_path, datasets, args.mode)
    print(f"Saved graphs and report to {args.output}")


if __name__ == "__main__":
    main()
