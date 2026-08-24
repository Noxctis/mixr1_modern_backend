#!/usr/bin/env python3
"""Analyze mixr1_daemon PWM sweep CSV files."""

import argparse
import csv
import math
import os
import statistics
import sys

try:
    import matplotlib.pyplot as plt
except ImportError:
    plt = None


def read_csv(path):
    rows = []
    with open(path, newline="") as handle:
        for row in csv.DictReader(handle):
            try:
                rows.append({key: float(value) for key, value in row.items()})
            except (TypeError, ValueError):
                continue
    return rows


def percentile(values, fraction):
    if not values:
        return float("nan")
    ordered = sorted(values)
    position = (len(ordered) - 1) * fraction
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    return ordered[lower] + (ordered[upper] - ordered[lower]) * (position - lower)


def mean(values):
    return statistics.fmean(values) if values else float("nan")


def stddev(values):
    return statistics.stdev(values) if len(values) > 1 else 0.0


def response_metrics(group, settling_band):
    target = group[-1]["target_rpm"]
    if target <= 0.0:
        return {"rise_time_s": float("nan"), "settling_time_s": float("nan"),
                "overshoot_percent": float("nan"), "steady_state_error_rpm": float("nan")}

    start = group[0]["elapsed_s"]
    times = [row["elapsed_s"] - start for row in group]
    rpm = [row["filtered_rpm"] for row in group]
    lower = 0.1 * target
    upper = 0.9 * target
    rise_start = next((time for time, value in zip(times, rpm) if value >= lower), float("nan"))
    rise_end = next((time for time, value in zip(times, rpm) if value >= upper), float("nan"))
    rise_time = rise_end - rise_start if math.isfinite(rise_start) and math.isfinite(rise_end) else float("nan")
    peak = max(rpm)
    overshoot = max(0.0, (peak - target) / target * 100.0)
    band = settling_band * target
    last_outside = max((time for time, value in zip(times, rpm) if abs(value - target) > band), default=0.0)
    settling = last_outside if last_outside < times[-1] else float("nan")
    tail_count = max(1, len(rpm) // 5)
    steady_error = mean([abs(target - value) for value in rpm[-tail_count:]])
    return {"rise_time_s": rise_time, "settling_time_s": settling,
            "overshoot_percent": overshoot, "steady_state_error_rpm": steady_error}


def summarize(rows, settle_seconds, late_limit_us, settling_band):
    step_starts = {}
    for row in rows:
        step = int(row["step_index"])
        step_starts[step] = min(row["elapsed_s"], step_starts.get(step, float("inf")))

    all_groups = {}
    for row in rows:
        step = int(row["step_index"])
        all_groups.setdefault(step, []).append(row)

    grouped = {}
    summary = []
    for step in sorted(all_groups):
        full_group = all_groups[step]
        group = [row for row in full_group if row["elapsed_s"] >= step_starts[step] + settle_seconds]
        if not group:
            continue
        pwm = int(round(full_group[-1]["pwm_percent"]))
        grouped[pwm] = group
        raw = [row["raw_rpm"] for row in group]
        periods = [row["loop_period_us"] for row in group]
        late = [row["late_us"] for row in group]
        clean = [row for row in group if row["late_us"] <= late_limit_us]
        clean_raw = [row["raw_rpm"] for row in clean]
        response = response_metrics(full_group, settling_band)
        target = mean([row["target_rpm"] for row in full_group])
        summary.append({
            "pwm_percent": pwm,
            "step_index": step,
            "target_rpm": target,
            "samples": len(group),
            "mean_raw_rpm": mean(raw),
            "std_raw_rpm": stddev(raw),
            "p05_raw_rpm": percentile(raw, 0.05),
            "p95_raw_rpm": percentile(raw, 0.95),
            "clean_samples": len(clean),
            "clean_mean_raw_rpm": mean(clean_raw),
            "clean_std_raw_rpm": stddev(clean_raw),
            "mean_period_us": mean(periods),
            "std_period_us": stddev(periods),
            "p95_late_us": percentile(late, 0.95),
            "max_late_us": max(late) if late else float("nan"),
            "late_cycles": sum(value > 0.0 for value in late),
            "timing_outliers": sum(value > late_limit_us for value in late),
            **response,
        })
    return grouped, summary


def write_summary(path, summary):
    if not summary:
        return
    with open(path, "w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(summary[0]))
        writer.writeheader()
        writer.writerows(summary)


def label(path):
    name = os.path.basename(path).lower()
    controller = "PI" if "pi" in name else "Open loop"
    if "fifo" in name and "no_fifo" not in name:
        return f"{controller} FIFO"
    return f"{controller} no FIFO"


def fit_open_loop(summary):
    points = [(row["pwm_percent"], row["clean_mean_raw_rpm"])
              for row in summary if math.isfinite(row["clean_mean_raw_rpm"])]
    if len(points) < 2:
        return None
    mean_x = mean([point[0] for point in points])
    mean_y = mean([point[1] for point in points])
    denominator = sum((x - mean_x) ** 2 for x, _ in points)
    if denominator == 0.0:
        return None
    slope = sum((x - mean_x) * (y - mean_y) for x, y in points) / denominator
    intercept = mean_y - slope * mean_x
    return slope, intercept


def baseline_error(target, open_loop_summary):
    points = [(row["pwm_percent"], row["clean_mean_raw_rpm"])
              for row in open_loop_summary
              if math.isfinite(row["clean_mean_raw_rpm"])]
    points.sort()
    if target <= 0.0 or len(points) < 2:
        return float("nan")
    requested_pwm = min(100.0, max(0.0, target / 2500.0 * 100.0))
    if requested_pwm <= points[0][0]:
        predicted = points[0][1]
    elif requested_pwm >= points[-1][0]:
        predicted = points[-1][1]
    else:
        for (lower_pwm, lower_rpm), (upper_pwm, upper_rpm) in zip(points, points[1:]):
            if lower_pwm <= requested_pwm <= upper_pwm:
                fraction = (requested_pwm - lower_pwm) / (upper_pwm - lower_pwm)
                predicted = lower_rpm + fraction * (upper_rpm - lower_rpm)
                break
    return abs(target - predicted)


def write_controller_comparison(path, pi_summary, open_loop_summary):
    fields = ["target_rpm", "pi_steady_state_error_rpm", "open_loop_interpolated_error_rpm",
              "improvement_percent"]
    with open(path, "w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for row in pi_summary:
            target = row["target_rpm"]
            pi_error = row["steady_state_error_rpm"]
            open_error = baseline_error(target, open_loop_summary)
            improvement = ((open_error - pi_error) / open_error * 100.0
                           if math.isfinite(open_error) and open_error > 0.0 else float("nan"))
            writer.writerow({"target_rpm": target,
                             "pi_steady_state_error_rpm": pi_error,
                             "open_loop_interpolated_error_rpm": open_error,
                             "improvement_percent": improvement})


def write_report(path, datasets, settling_band, late_limit_us):
    open_loop = next((summary for name, (_, summary) in datasets.items()
                      if "Open loop" in name), None)
    pi_datasets = [(name, summary) for name, (_, summary) in datasets.items() if "PI" in name]
    with open(path, "w") as handle:
        handle.write("# Motor and PI Controller Test Report\n\n")
        handle.write("## Experiment interpretation\n\n")
        handle.write("The open-loop test applies fixed PWM levels from 0% to 100%. "
                     "The PI test applies matching RPM setpoints. Open-loop data is the plant baseline; "
                     "PI data measures feedback tracking. FIFO and non-FIFO runs are scheduling comparisons.\n\n")
        handle.write("## Metrics\n\n")
        handle.write(f"Steady-state statistics discard the first configured settling interval and remove timing outliers above {late_limit_us:.0f} us. "
                     f"Transient settling uses filtered RPM and a +/-{settling_band * 100:.0f}% target band.\n\n")
        handle.write("- Mean RPM: average measured raw RPM.\n")
        handle.write("- Standard deviation: spread of raw RPM, including timing effects.\n")
        handle.write("- Clean standard deviation: raw RPM spread after timing outliers are excluded.\n")
        handle.write("- Loop jitter: standard deviation of the measured loop period.\n")
        handle.write("- Maximum lateness: largest delay after the absolute scheduled wake time.\n")
        handle.write("- Timing outliers: samples whose lateness exceeds the configured threshold.\n")
        handle.write("- Rise time: time for filtered RPM to move from 10% to 90% of target.\n")
        handle.write("- Settling time: time after which filtered RPM remains within the settling band.\n")
        handle.write("- Overshoot: maximum amount above target, expressed as a percentage.\n")
        handle.write("- Steady-state error: mean absolute target error in the final fifth of a step.\n\n")
        handle.write("## First-order model\n\n")
        handle.write("Use the identified motor model:\n\n")
        handle.write("$$G(s)=\\frac{K}{s+a}=\\frac{K_m}{\\tau s+1},\\qquad \\tau=\\frac{1}{a}$$\n\n")
        handle.write("Here, K (or K_m) is the input-to-speed gain, a is the response rate, and tau is the time constant. "
                     "Estimate these from the raw open-loop step responses, not from the PI response. The PWM sweep supplies the steady-state input/output relationship; "
                     "the time trace supplies the dynamic time constant.\n\n")
        handle.write("## IMC and gain selection\n\n")
        handle.write("For a first-order-plus-delay model, an IMC PI rule is:\n\n")
        handle.write("$$G(s)=\\frac{K_m e^{-\\theta s}}{\\tau s+1},\\qquad "
                     "K_p=\\frac{\\tau}{K_m(\\lambda+\\theta)},\\qquad K_i=\\frac{K_p}{\\tau}$$\n\n")
        handle.write("lambda sets the desired closed-loop speed: smaller lambda is faster but less robust, while larger lambda is slower but more robust. "
                     "The current C++ implementation uses the measured gain schedule in config.hpp and linearly interpolates Kp and Ki by target RPM. "
                     "Describe these as IMC-derived only if the scheduled values were calculated with the IMC equations above; otherwise describe them as experimentally tuned gain-schedule values.\n\n")
        handle.write("## Controller improvement\n\n")
        handle.write("PI improvement is evaluated against the open-loop RPM predicted at the corresponding PWM percentage:\n\n")
        handle.write("$$Improvement=100\\left(1-\\frac{e_{PI}}{e_{open}}\\right)\\%$$\n\n")
        handle.write("where e is mean absolute steady-state RPM error. Use the generated comparison CSV and PI performance plot. "
                     "Report water-loaded and no-load tests separately because water turbulence changes the plant.\n\n")
        if open_loop:
            model = fit_open_loop(open_loop)
            if model:
                handle.write(f"The current open-loop settled data has an approximate linear steady-state fit of RPM = {model[0]:.2f} * PWM(%) + {model[1]:.2f}. "
                             "This is a static characterization, not the dynamic first-order model by itself.\n\n")
        for name, summary in pi_datasets:
            finite_errors = [row["steady_state_error_rpm"] for row in summary
                             if math.isfinite(row["steady_state_error_rpm"])]
            if finite_errors:
                handle.write(f"{name}: mean PI steady-state error across valid targets = {mean(finite_errors):.2f} RPM.\n")


def make_plots(datasets, output_dir):
    if plt is None:
        print("Plots skipped: matplotlib is not installed.", file=sys.stderr)
        return

    colors = {"PI FIFO": "tab:blue", "PI no FIFO": "tab:orange",
              "Open loop FIFO": "tab:green", "Open loop no FIFO": "tab:red"}
    figure, axis = plt.subplots(figsize=(10, 6))
    for name, (_, summary) in datasets.items():
        pwm = [row["target_rpm"] if "PI" in name else row["pwm_percent"] for row in summary]
        rpm = [row["clean_mean_raw_rpm"] for row in summary]
        spread = [row["clean_std_raw_rpm"] for row in summary]
        axis.errorbar(pwm, rpm, yerr=spread, marker="o", capsize=3,
                      label=name, color=colors.get(name))
    axis.set(title="RPM characterization and PI tracking", xlabel="PWM (%) or target RPM", ylabel="RPM")
    axis.grid(alpha=0.3)
    axis.legend()
    figure.tight_layout()
    figure.savefig(os.path.join(output_dir, "rpm_vs_pwm.png"), dpi=160)
    plt.close(figure)

    figure, axis = plt.subplots(figsize=(10, 6))
    for name, (_, summary) in datasets.items():
        pwm = [row["target_rpm"] if "PI" in name else row["pwm_percent"] for row in summary]
        jitter = [row["std_period_us"] for row in summary]
        axis.plot(pwm, jitter, marker="o", label=name, color=colors.get(name))
    axis.set(title="Loop-period jitter", xlabel="PWM (%)", ylabel="Period standard deviation (us)")
    axis.grid(alpha=0.3)
    axis.legend()
    figure.tight_layout()
    figure.savefig(os.path.join(output_dir, "jitter_vs_pwm.png"), dpi=160)
    plt.close(figure)

    for pwm in sorted({pwm for _, (groups, _) in datasets.items() for pwm in groups}):
        figure, axis = plt.subplots(figsize=(10, 6))
        plotted = False
        for name, (groups, _) in datasets.items():
            if pwm not in groups:
                continue
            values = [row["raw_rpm"] for row in groups[pwm]]
            axis.hist(values, bins=25, alpha=0.5, label=name)
            plotted = True
        if plotted:
            axis.set(title=f"Raw RPM distribution at {pwm}% PWM",
                     xlabel="Raw RPM", ylabel="Samples")
            axis.grid(alpha=0.3)
            axis.legend()
            figure.tight_layout()
            figure.savefig(os.path.join(output_dir, f"rpm_hist_{pwm:03d}.png"), dpi=160)
        plt.close(figure)

    figure, axes = plt.subplots(2, 2, figsize=(12, 8), sharex=False)
    metrics = [("rise_time_s", "Rise time (s)"),
               ("settling_time_s", "Settling time (s)"),
               ("overshoot_percent", "Overshoot (%)"),
               ("steady_state_error_rpm", "Steady-state error (RPM)")]
    for axis, (key, title) in zip(axes.flat, metrics):
        for name, (_, summary) in datasets.items():
            if "PI" not in name:
                continue
            x_values = [row["target_rpm"] for row in summary]
            y_values = [row[key] for row in summary]
            axis.plot(x_values, y_values, marker="o", label=name, color=colors.get(name))
        axis.set_title(title)
        axis.grid(alpha=0.3)
    axes[0, 0].legend()
    axes[1, 0].set_xlabel("Target RPM")
    axes[1, 1].set_xlabel("Target RPM")
    figure.suptitle("PI controller performance")
    figure.tight_layout()
    figure.savefig(os.path.join(output_dir, "pi_performance.png"), dpi=160)
    plt.close(figure)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", nargs="+", help="Sweep CSV files to compare")
    parser.add_argument("--settle", type=float, default=2.0,
                        help="Seconds discarded at the start of each PWM step (default: 2)")
    parser.add_argument("--late-limit-us", type=float, default=500.0,
                        help="Timing-outlier threshold for clean RPM statistics (default: 500)")
    parser.add_argument("--settling-band", type=float, default=0.05,
                        help="Settling band as a fraction of target (default: 0.05)")
    parser.add_argument("--output", default="sweep_analysis",
                        help="Output directory (default: sweep_analysis)")
    args = parser.parse_args()

    os.makedirs(args.output, exist_ok=True)
    datasets = {}
    for path in args.csv:
        rows = read_csv(path)
        if not rows:
            print(f"No valid rows in {path}", file=sys.stderr)
            continue
        name = label(path)
        if name in datasets:
            name = os.path.basename(path)
        groups, summary = summarize(rows, args.settle, args.late_limit_us, args.settling_band)
        datasets[name] = (groups, summary)
        summary_path = os.path.join(args.output, name.lower().replace(" ", "_") + "_summary.csv")
        write_summary(summary_path, summary)
        print(f"{name}: {len(rows)} rows, summary saved to {summary_path}")
        print("PWM  mean RPM  std RPM  clean std  jitter std(us)  max late(us)  outliers  rise(s)  settle(s)  overshoot(%)  ss error")
        for item in summary:
            print(f"{item['pwm_percent']:3.0f}  {item['mean_raw_rpm']:8.1f}  "
                  f"{item['std_raw_rpm']:7.1f}  {item['clean_std_raw_rpm']:9.1f}  "
                  f"{item['std_period_us']:14.1f}  {item['max_late_us']:12.1f}  "
                  f"{item['timing_outliers']:8d}  {item['rise_time_s']:7.3f}  "
                  f"{item['settling_time_s']:9.3f}  {item['overshoot_percent']:12.2f}  "
                  f"{item['steady_state_error_rpm']:8.1f}")

    open_loop = next((summary for name, (_, summary) in datasets.items()
                      if "Open loop" in name), None)
    if open_loop is not None:
        for name, (_, summary) in datasets.items():
            if "PI" in name:
                comparison_path = os.path.join(args.output, name.lower().replace(" ", "_") + "_comparison.csv")
                write_controller_comparison(comparison_path, summary, open_loop)
                print(f"Controller comparison saved to {comparison_path}")

    report_path = os.path.join(args.output, "controller_analysis_report.md")
    write_report(report_path, datasets, args.settling_band, args.late_limit_us)
    print(f"Report saved to {report_path}")

    make_plots(datasets, args.output)
    if plt is not None:
        print(f"Plots saved to {args.output}/")


if __name__ == "__main__":
    main()
