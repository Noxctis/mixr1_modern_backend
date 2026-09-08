"""
analyze_sweep.py
Purpose: Automated Multi-Step Response Analyzer for MIXR-1 (Synchronous CET Data).
Extracts steady-state gain, system poles, bandwidth, and IMC PI tuning parameters.
"""
import glob
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy.optimize import curve_fit

def first_order_step(t, baseline, K, A):
    return baseline + K * (1 - np.exp(-A * t))

csv_files = glob.glob("*.csv")
if not csv_files:
    print("Error: No CSV files found in this directory.")
    sys.exit(1)

print("========================================")
print(" SELECT SWEEP DATASET TO ANALYZE ")
print("========================================")
for i, f in enumerate(csv_files):
    print(f"[{i + 1}] {f}")

try:
    choice = int(input("\nEnter file number: ")) - 1
    if choice < 0 or choice >= len(csv_files):
        raise ValueError
    selected_file = csv_files[choice]
except ValueError:
    print("Invalid selection.")
    sys.exit(1)

df = pd.read_csv(selected_file)
required_cols = {'elapsed_s', 'step_index', 'pwm_percent', 'raw_rpm', 'pwm'}
if not required_cols.issubset(df.columns):
    print(f"Error: CSV is missing required columns. Found: {df.columns.tolist()}")
    sys.exit(1)

out_dir = "step_response_plots"
os.makedirs(out_dir, exist_ok=True)

unique_steps = [s for s in df['step_index'].unique() if s > 0]
master_file = "tuning_table.csv"
file_exists = os.path.isfile(master_file)

valid_steps = []
kp_list = []
ki_list = []

print("\n" + "=" * 60)
print(f" ANALYZING SWEEP: {selected_file} ")
print("=" * 60)

with open(master_file, "a") as f:
    if not file_exists:
        f.write("Dataset,Step_Idx,Baseline_PWM_pct,Step_PWM_pct,Baseline_RPM,Pole_A,Rise_Time_s,Settling_Time_s,Bandwidth_Hz,Kp,Ki\n")

    for step_idx in unique_steps:
        prev_data = df[df['step_index'] == step_idx - 1]
        curr_data = df[df['step_index'] == step_idx]
        
        if prev_data.empty or curr_data.empty:
            continue

        t_prev_max = prev_data['elapsed_s'].max()
        baseline_data = prev_data[prev_data['elapsed_s'] >= (t_prev_max - 1.5)]
        baseline_rpm = baseline_data['raw_rpm'].mean() if not baseline_data.empty else 0.0

        base_pct = prev_data['pwm_percent'].iloc[-1]
        step_pct = curr_data['pwm_percent'].iloc[0]
        base_pwm_raw = prev_data['pwm'].iloc[-1]
        step_pwm_raw = curr_data['pwm'].iloc[0]

        t = curr_data['elapsed_s'].to_numpy(dtype=float)
        t = t - t[0]
        rpm = curr_data['raw_rpm'].to_numpy(dtype=float)

        p0 = [np.max(rpm) - baseline_rpm, 5.0]
        
        try:
            popt, _ = curve_fit(lambda t, K, A: first_order_step(t, baseline_rpm, K, A), t, rpm, p0=p0)
            K_fit = popt[0]
            A_fit = popt[1]
        except RuntimeError:
            print(f"[SKIP] Step {step_idx} ({base_pct}%->{step_pct}%): Curve fit failed to converge.")
            continue

        if A_fit <= 0:
            print(f"[SKIP] Step {step_idx}: Invalid system pole calculated (A = {A_fit:.2f}).")
            continue

        settling_time_2pct = 4.0 / A_fit
        rise_time_10_90 = 2.197 / A_fit
        bandwidth_hz = A_fit / (2 * np.pi)
        
        delta_pwm = step_pwm_raw - base_pwm_raw
        if delta_pwm == 0:
            continue
            
        K_plant = K_fit / delta_pwm
        Kp_calc = 1.0 / K_plant if np.isfinite(K_plant) and abs(K_plant) > 0 else np.nan
        Ki_calc = Kp_calc * A_fit if np.isfinite(Kp_calc) else np.nan

        print(f"Step {step_idx:2d} | {base_pct:3d}% -> {step_pct:3d}% PWM | Baseline: {baseline_rpm:7.2f} RPM")
        print(f"        -> Pole (A): {A_fit:6.3f} | BW: {bandwidth_hz:5.2f} Hz | Kp: {Kp_calc:7.4f} | Ki: {Ki_calc:7.4f}")

        f.write(f"{selected_file},{step_idx},{base_pct},{step_pct},{baseline_rpm:.2f},{A_fit:.4f},{rise_time_10_90:.4f},{settling_time_2pct:.4f},{bandwidth_hz:.4f},{Kp_calc:.4f},{Ki_calc:.4f}\n")

        valid_steps.append((baseline_rpm, Kp_calc, Ki_calc, base_pct, step_pct))
        kp_list.append(Kp_calc)
        ki_list.append(Ki_calc)

        # Plotting Side-by-Side
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 5))
        
        ax1.scatter(t, rpm, s=8, color="#58a6ff", alpha=0.6)
        ax1.set_title(f"Raw Data: {base_pct}% to {step_pct}% PWM")
        ax1.set_xlabel("Time (seconds)")
        ax1.set_ylabel("RPM")
        ax1.grid(True, linestyle=":", alpha=0.7)

        ax2.scatter(t, rpm, s=8, color="#58a6ff", alpha=0.3, label="Raw CET Data")
        ax2.plot(t, first_order_step(t, baseline_rpm, K_fit, A_fit), "r-", linewidth=2.5, label=f"Fitted G(s): K={K_fit:.1f}, A={A_fit:.2f}")
        ax2.axvline(x=rise_time_10_90, color="orange", linestyle="--", alpha=0.7, label=f"Rise Time ({rise_time_10_90:.2f}s)")
        ax2.axvline(x=settling_time_2pct, color="#3fb950", linestyle="--", alpha=0.7, label=f"Settling Time ({settling_time_2pct:.2f}s)")
        ax2.set_title(f"Fitted Model (BW: {bandwidth_hz:.2f} Hz)")
        ax2.set_xlabel("Time (seconds)")
        ax2.legend()
        ax2.grid(True, linestyle=":", alpha=0.7)

        plt.tight_layout()
        plot_filename = os.path.join(out_dir, f"plot_step_{step_idx}_{base_pct}_{step_pct}.png")
        plt.savefig(plot_filename, dpi=300)
        plt.close()

print("\n" + "=" * 60)
print(f" [1] SINGLE GLOBAL PI TUNING (MEDIAN) ")
print("=" * 60)
global_kp = np.median(kp_list) if kp_list else 0.0
global_ki = np.median(ki_list) if ki_list else 0.0
print(f"Kp = {global_kp:.4f}")
print(f"Ki = {global_ki:.4f}")

print("\n" + "=" * 60)
print(f" [2] GAIN SCHEDULING ARRAY (C++ config.hpp) ")
print("=" * 60)
cpp_schedule = f"    // --- COPY INTO config.hpp ---\n    constexpr std::array<GainTier, {len(valid_steps)}> PI_SCHEDULE = {{\n"
for i, (rpm, kp, ki, base, step) in enumerate(valid_steps):
    comma = "," if i < len(valid_steps) - 1 else ""
    cpp_schedule += f"        {{{rpm:7.2f}, {kp:7.4f}, {ki:7.4f}}}{comma} // Step {i+1}: {base}% to {step}% PWM\n"
cpp_schedule += "    }};\n"
print(cpp_schedule)

print("=" * 60)
print(f"[SUCCESS] Appended to {master_file}.")
print(f"[SUCCESS] {len(valid_steps)} step response plots saved to '{out_dir}/' directory.")