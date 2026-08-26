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
    """Read CSV rows into a list of dicts.

    Preserve known string metadata fields (intended_mode, intended_fifo, fifo_active, condition)
    and convert numeric fields to float when possible. Skip rows missing essential numeric
    fields such as elapsed_s or step_index.
    """
    rows = []
    string_fields = {"intended_mode", "intended_fifo", "fifo_active", "condition"}
    with open(path, newline="") as handle:
        reader = csv.DictReader(handle)
        for raw_row in reader:
            parsed = {}
            # Convert numeric-looking fields and preserve known string metadata
            for key, value in raw_row.items():
                if value is None:
                    parsed[key] = float("nan")
                    continue
                val = value.strip()
                if key in string_fields:
                    parsed[key] = val
                else:
                    try:
                        parsed[key] = float(val) if val != "" else float("nan")
                    except (TypeError, ValueError):
                        parsed[key] = float("nan")
            # Require essential numeric fields to be present and finite
            try:
                if math.isnan(parsed.get("elapsed_s", float("nan"))) or math.isnan(parsed.get("step_index", float("nan"))):
                    continue
            except Exception:
                continue
            rows.append(parsed)
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

        # period statistics
        mean_period = mean(periods)
        # population RMS jitter (root-mean-square of deviations from mean)
        if periods:
            rms_jitter = math.sqrt(sum((p - mean_period) ** 2 for p in periods) / len(periods))
            abs_devs = [abs(p - mean_period) for p in periods]
            p95_abs = percentile(abs_devs, 0.95)
            p99_abs = percentile(abs_devs, 0.99)
            cv_period = (rms_jitter / mean_period) if mean_period != 0.0 else float("nan")
        else:
            rms_jitter = float("nan")
            p95_abs = float("nan")
            p99_abs = float("nan")
            cv_period = float("nan")

        # capture canonical metadata if present
        meta_mode = group[0].get("intended_mode") if group and group[0].get("intended_mode") else ""
        meta_fifo = group[0].get("intended_fifo") if group and group[0].get("intended_fifo") else ""
        meta_fifo_active = group[0].get("fifo_active") if group and group[0].get("fifo_active") else ""
        meta_condition = group[0].get("condition") if group and group[0].get("condition") else ""

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
            "mean_period_us": mean_period,
            "std_period_us": stddev(periods),
            "rms_period_us": rms_jitter,
            "p95_abs_jitter_us": p95_abs,
            "p99_abs_jitter_us": p99_abs,
            "cv_period": cv_period,
            "p95_late_us": percentile(late, 0.95),
            "max_late_us": max(late) if late else float("nan"),
            "late_cycles": sum(value > 0.0 for value in late),
            "timing_outliers": sum(value > late_limit_us for value in late),
            "intended_mode": meta_mode,
            "intended_fifo": meta_fifo,
            "fifo_active": meta_fifo_active,
            "condition": meta_condition,
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
              if math.isfinite(row["pwm_percent"])
              and math.isfinite(row["clean_mean_raw_rpm"])]
    points.sort()
    if target <= 0.0 or len(points) < 2:
        return float("nan")

    requested_pwm = min(100.0, max(0.0, target / 2500.0 * 100.0))
    if requested_pwm <= points[0][0]:
        predicted = points[0][1]
    elif requested_pwm >= points[-1][0]:
        predicted = points[-1][1]
    else:
        # Keep a fallback so static analyzers know this is always assigned.
        predicted = points[-1][1]
        for (lower_pwm, lower_rpm), (upper_pwm, upper_rpm) in zip(points, points[1:]):
            if lower_pwm <= requested_pwm <= upper_pwm:
                fraction = (requested_pwm - lower_pwm) / (upper_pwm - lower_pwm)
                predicted = lower_rpm + fraction * (upper_rpm - lower_rpm)
                break

    return abs(target - predicted)


def _interpolate_metric_by_pwm(target, open_loop_summary, metric_name):
    """Interpolate a metric from open-loop summary rows based on the equivalent PWM for a target RPM."""
    points = [(row["pwm_percent"], row.get(metric_name, float("nan")))
              for row in open_loop_summary
              if math.isfinite(row.get("pwm_percent", float("nan"))) and math.isfinite(row.get(metric_name, float("nan")))]
    points.sort()
    if not points:
        return float("nan")
    requested_pwm = min(100.0, max(0.0, target / 2500.0 * 100.0))
    if requested_pwm <= points[0][0]:
        return points[0][1]
    if requested_pwm >= points[-1][0]:
        return points[-1][1]
    for (lp, lv), (up, uv) in zip(points, points[1:]):
        if lp <= requested_pwm <= up:
            fraction = (requested_pwm - lp) / (up - lp) if up != lp else 0.0
            return lv + fraction * (uv - lv)
    return float("nan")


def write_controller_comparison(path, pi_summary, open_loop_summary):
    fields = ["target_rpm", "pi_steady_state_error_rpm", "open_loop_interpolated_error_rpm",
              "improvement_percent",
              # PI-side jitter metrics (aggregated at this target row)
              "pi_rms_period_us", "pi_p95_abs_jitter_us", "pi_p99_abs_jitter_us", "pi_cv_period",
              # Open-loop interpolated jitter metrics for the equivalent PWM
              "open_rms_period_us", "open_p95_abs_jitter_us", "open_p99_abs_jitter_us", "open_cv_period"]

    with open(path, "w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for row in pi_summary:
            target = row["target_rpm"]
            pi_error = row.get("steady_state_error_rpm", float("nan"))
            open_error = baseline_error(target, open_loop_summary)
            improvement = ((open_error - pi_error) / open_error * 100.0
                           if math.isfinite(open_error) and open_error > 0.0 else float("nan"))

            # PI jitter metrics directly from the PI summary row
            pi_rms = row.get("rms_period_us", float("nan"))
            pi_p95 = row.get("p95_abs_jitter_us", float("nan"))
            pi_p99 = row.get("p99_abs_jitter_us", float("nan"))
            pi_cv = row.get("cv_period", float("nan"))

            # Interpolate equivalent open-loop jitter metrics by PWM
            open_rms = _interpolate_metric_by_pwm(target, open_loop_summary, "rms_period_us")
            open_p95 = _interpolate_metric_by_pwm(target, open_loop_summary, "p95_abs_jitter_us")
            open_p99 = _interpolate_metric_by_pwm(target, open_loop_summary, "p99_abs_jitter_us")
            open_cv = _interpolate_metric_by_pwm(target, open_loop_summary, "cv_period")

            writer.writerow({"target_rpm": target,
                             "pi_steady_state_error_rpm": pi_error,
                             "open_loop_interpolated_error_rpm": open_error,
                             "improvement_percent": improvement,
                             "pi_rms_period_us": pi_rms,
                             "pi_p95_abs_jitter_us": pi_p95,
                             "pi_p99_abs_jitter_us": pi_p99,
                             "pi_cv_period": pi_cv,
                             "open_rms_period_us": open_rms,
                             "open_p95_abs_jitter_us": open_p95,
                             "open_p99_abs_jitter_us": open_p99,
                             "open_cv_period": open_cv})


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

        handle.write("## Thesis interpretation of the plots\n\n")
        handle.write("The RPM-versus-PWM plot characterizes the motor plant in open loop. Each point is the mean measured raw speed after the settling interval; error bars show the standard deviation of the measured speed. A higher standard deviation means that the motor speed is less repeatable at that operating point.\n\n")
        handle.write("The RPM-variability plot separates motor-speed variation from execution timing. Clean RPM standard deviation excludes samples whose measured lateness exceeds the configured timing-outlier limit, while the full standard deviation retains them. Comparing both values shows whether unusual speed readings are associated with scheduler interruptions.\n\n")
        handle.write("The loop-period plot shows the actual time between successive control-loop executions. Its reference value is the configured loop delay, 10,000 us in this experiment. The loop-period jitter plot shows the spread of those measured periods: RMS jitter is the root-mean-square deviation from the mean period, and P95/P99 show the deviation below which 95%/99% of samples fall. Lower values indicate more deterministic scheduling.\n\n")
        handle.write("The lateness plot measures delay relative to the absolute scheduled wake time, rather than speed error. P95 lateness describes typical worst-case behavior, whereas maximum lateness exposes rare severe interruptions. The timing-outlier-rate plot reports the percentage of samples with lateness above the configured threshold.\n\n")
        handle.write("The RPM histograms show the frequency distribution of individual speed samples at each PWM level. A narrow distribution indicates repeatable speed; a broad or multi-peaked distribution indicates variation, changing load, encoder quantization, or transient effects. Histograms should be interpreted together with the numerical standard deviation and timing plots, not as a replacement for them.\n\n")
        handle.write("For the scheduling comparison, the principal thesis measures are RMS or standard-deviation jitter, P95 and P99 jitter, maximum lateness, timing-outlier rate, and RPM standard deviation. Report FIFO and non-FIFO using identical PWM levels, duration, settling time, and outlier threshold. Timing jitter describes when the controller runs; RPM variation describes how the motor responds. These are related but distinct outcomes.\n\n")

        # Jitter metrics summary across each dataset (averaged over PWM/step points)
        handle.write("### Jitter metrics\n\n")
        for name, (_, summary) in datasets.items():
            # prefer canonical condition label when present
            display_name = summary[0].get("condition") if summary and summary[0].get("condition") else name
            display_label = display_name.replace("_", " ")
            rms_vals = [row.get("rms_period_us", float("nan")) for row in summary if math.isfinite(row.get("rms_period_us", float("nan")))]
            p95_vals = [row.get("p95_abs_jitter_us", float("nan")) for row in summary if math.isfinite(row.get("p95_abs_jitter_us", float("nan")))]
            p99_vals = [row.get("p99_abs_jitter_us", float("nan")) for row in summary if math.isfinite(row.get("p99_abs_jitter_us", float("nan")))]
            cv_vals = [row.get("cv_period", float("nan")) for row in summary if math.isfinite(row.get("cv_period", float("nan")))]
            if rms_vals:
                mean_rms = mean(rms_vals)
                mean_p95 = mean(p95_vals) if p95_vals else float("nan")
                mean_p99 = mean(p99_vals) if p99_vals else float("nan")
                mean_cv = mean(cv_vals) if cv_vals else float("nan")
                handle.write(f"- {display_label}: mean RMS jitter = {mean_rms:.2f} us, mean P95 abs jitter = {mean_p95:.2f} us, mean P99 abs jitter = {mean_p99:.2f} us, mean CV = {mean_cv * 100.0:.2f}%\n")
            else:
                handle.write(f"- {display_label}: jitter metrics not available.\n")
        handle.write("\n")
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

    # Normalize display names to use spaces for legend and map to colors
    color_map = {"PI FIFO": "tab:blue", "PI no FIFO": "tab:orange",
                 "Open loop FIFO": "tab:green", "Open loop no FIFO": "tab:red",
                 "PI_FIFO": "tab:blue", "PI_noFIFO": "tab:orange",
                 "OpenLoop_FIFO": "tab:green", "OpenLoop_NoFIFO": "tab:red"}

    figure, axis = plt.subplots(figsize=(10, 6))
    for name, (_, summary) in datasets.items():
        # Use condition metadata if present in summary rows; otherwise infer from dataset name
        if summary and summary[0].get("condition"):
            display_name = summary[0].get("condition")
        else:
            display_name = name
        display_label = display_name.replace("_", " ")
        pwm = [row["target_rpm"] if ("PI" in display_label or display_label.startswith("PI")) else row["pwm_percent"] for row in summary]
        rpm = [row["clean_mean_raw_rpm"] for row in summary]
        spread = [row["clean_std_raw_rpm"] for row in summary]
        axis.errorbar(pwm, rpm, yerr=spread, marker="o", capsize=3,
                      label=display_label, color=color_map.get(display_name, None))
    axis.set(title="RPM characterization and PI tracking", xlabel="PWM (%) or target RPM", ylabel="RPM")
    axis.grid(alpha=0.3)
    axis.legend()
    figure.tight_layout()
    figure.savefig(os.path.join(output_dir, "rpm_vs_pwm.png"), dpi=160)
    plt.close(figure)

    figure, axis = plt.subplots(figsize=(10, 6))
    for name, (_, summary) in datasets.items():
        if summary and summary[0].get("condition"):
            display_name = summary[0].get("condition")
        else:
            display_name = name
        display_label = display_name.replace("_", " ")
        pwm = [row["target_rpm"] if ("PI" in display_label or display_label.startswith("PI")) else row["pwm_percent"] for row in summary]
        rms = [row.get("rms_period_us", float("nan")) for row in summary]
        p95 = [row.get("p95_abs_jitter_us", float("nan")) for row in summary]
        p99 = [row.get("p99_abs_jitter_us", float("nan")) for row in summary]
        color = color_map.get(display_name, None)
        axis.plot(pwm, rms, marker="o", label=f"{display_label} RMS", color=color)
        axis.plot(pwm, p95, marker="^", linestyle="--", label=f"{display_label} P95", color=color, alpha=0.75)
        axis.plot(pwm, p99, marker="s", linestyle=":", label=f"{display_label} P99", color=color, alpha=0.75)
    axis.set(title="Loop-period jitter percentiles", xlabel="PWM (%) or target RPM", ylabel="Absolute period deviation (us)")
    axis.grid(alpha=0.3)
    axis.legend()
    figure.tight_layout()
    figure.savefig(os.path.join(output_dir, "jitter_vs_pwm.png"), dpi=160)
    plt.close(figure)

    figure, axis = plt.subplots(figsize=(10, 6))
    for name, (_, summary) in datasets.items():
        display_name = summary[0].get("condition") if summary and summary[0].get("condition") else name
        display_label = display_name.replace("_", " ")
        pwm = [row["pwm_percent"] for row in summary]
        axis.plot(pwm, [row["std_raw_rpm"] for row in summary], marker="o", label=f"{display_label} full")
        axis.plot(pwm, [row["clean_std_raw_rpm"] for row in summary], marker="^", linestyle="--", label=f"{display_label} clean")
    axis.set(title="Motor-speed variability", xlabel="PWM (%)", ylabel="RPM standard deviation")
    axis.grid(alpha=0.3)
    axis.legend()
    figure.tight_layout()
    figure.savefig(os.path.join(output_dir, "rpm_variability_vs_pwm.png"), dpi=160)
    plt.close(figure)

    figure, axis = plt.subplots(figsize=(10, 6))
    for name, (_, summary) in datasets.items():
        display_name = summary[0].get("condition") if summary and summary[0].get("condition") else name
        display_label = display_name.replace("_", " ")
        pwm = [row["pwm_percent"] for row in summary]
        axis.plot(pwm, [row["mean_period_us"] for row in summary], marker="o", label=display_label)
    axis.axhline(10000.0, color="black", linestyle="--", linewidth=1, label="10,000 us target")
    axis.set(title="Measured control-loop period", xlabel="PWM (%)", ylabel="Mean loop period (us)")
    axis.grid(alpha=0.3)
    axis.legend()
    figure.tight_layout()
    figure.savefig(os.path.join(output_dir, "loop_period_vs_pwm.png"), dpi=160)
    plt.close(figure)

    figure, axis = plt.subplots(figsize=(10, 6))
    for name, (_, summary) in datasets.items():
        display_name = summary[0].get("condition") if summary and summary[0].get("condition") else name
        display_label = display_name.replace("_", " ")
        pwm = [row["pwm_percent"] for row in summary]
        axis.plot(pwm, [row["p95_late_us"] for row in summary], marker="o", label=f"{display_label} P95")
        axis.plot(pwm, [row["max_late_us"] for row in summary], marker="s", linestyle=":", label=f"{display_label} maximum")
    axis.set(title="Control-loop lateness", xlabel="PWM (%)", ylabel="Lateness (us)")
    axis.grid(alpha=0.3)
    axis.legend()
    figure.tight_layout()
    figure.savefig(os.path.join(output_dir, "lateness_vs_pwm.png"), dpi=160)
    plt.close(figure)

    figure, axis = plt.subplots(figsize=(10, 6))
    for name, (_, summary) in datasets.items():
        display_name = summary[0].get("condition") if summary and summary[0].get("condition") else name
        display_label = display_name.replace("_", " ")
        pwm = [row["pwm_percent"] for row in summary]
        rates = [100.0 * row["timing_outliers"] / row["samples"] if row["samples"] else float("nan") for row in summary]
        axis.plot(pwm, rates, marker="o", label=display_label)
    axis.set(title="Timing-outlier rate", xlabel="PWM (%)", ylabel="Samples above lateness limit (%)")
    axis.grid(alpha=0.3)
    axis.legend()
    figure.tight_layout()
    figure.savefig(os.path.join(output_dir, "timing_outlier_rate_vs_pwm.png"), dpi=160)
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
            axis.plot(x_values, y_values, marker="o", label=name, color=color_map.get(name))
        axis.set_title(title)
        axis.grid(alpha=0.3)
    if any("PI" in name for name in datasets):
        axes[0, 0].legend()
    axes[1, 0].set_xlabel("Target RPM")
    axes[1, 1].set_xlabel("Target RPM")
    figure.suptitle("PI controller performance")
    figure.tight_layout()
    figure.savefig(os.path.join(output_dir, "pi_performance.png"), dpi=160)
    plt.close(figure)

    # Jitter comparison summary (per-dataset mean P95/P99 and RMS)
    jitter_summary_path = os.path.join(output_dir, "jitter_summary.csv")
    jitter_rows = []
    for name, (_, summary) in datasets.items():
        if not summary:
            continue
        display_name = summary[0].get("condition") if summary and summary[0].get("condition") else name
        mean_rms = mean([row.get("rms_period_us", float("nan")) for row in summary if math.isfinite(row.get("rms_period_us", float("nan")))])
        mean_p95 = mean([row.get("p95_abs_jitter_us", float("nan")) for row in summary if math.isfinite(row.get("p95_abs_jitter_us", float("nan")))])
        mean_p99 = mean([row.get("p99_abs_jitter_us", float("nan")) for row in summary if math.isfinite(row.get("p99_abs_jitter_us", float("nan")))])
        mean_cv = mean([row.get("cv_period", float("nan")) for row in summary if math.isfinite(row.get("cv_period", float("nan")))])
        jitter_rows.append({"condition": display_name, "mean_rms_us": mean_rms, "mean_p95_abs_us": mean_p95, "mean_p99_abs_us": mean_p99, "mean_cv": mean_cv})

    # write CSV summary
    if jitter_rows:
        with open(jitter_summary_path, "w", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=list(jitter_rows[0]))
            writer.writeheader()
            writer.writerows(jitter_rows)

        # create P95/P99 bar chart
        conditions = [r["condition"] for r in jitter_rows]
        p95_vals = [r["mean_p95_abs_us"] for r in jitter_rows]
        p99_vals = [r["mean_p99_abs_us"] for r in jitter_rows]
        x = list(range(len(conditions)))
        width = 0.35
        figure, ax = plt.subplots(figsize=(10, 6))
        ax.bar([i - width/2 for i in x], p95_vals, width, label='P95 abs jitter (us)')
        ax.bar([i + width/2 for i in x], p99_vals, width, label='P99 abs jitter (us)')
        ax.set_xticks(x)
        ax.set_xticklabels([c.replace("_", " ") for c in conditions], rotation=45, ha='right')
        ax.set_ylabel('Absolute jitter (us)')
        ax.set_title('P95 / P99 absolute jitter by condition')
        ax.grid(axis='y', alpha=0.3)
        ax.legend()
        figure.tight_layout()
        figure.savefig(os.path.join(output_dir, "jitter_comparison.png"), dpi=160)
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
        # Prefer condition label from CSV metadata when present, otherwise fall back to filename
        name = rows[0].get("condition") if rows and rows[0].get("condition") else label(path)
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
