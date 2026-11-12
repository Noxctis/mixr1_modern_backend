#!/usr/bin/env python3
"""
orders_figs.py - the figures of Section 'Spectral Analysis of the Speed Signal' and 'Resolution Selection':
    fig_orders_spectra, fig_orders_map, fig_orders_amplitude   from cet_pwm_sweep.csv and dct_pwm_sweep.csv (open-loop PWM sweep, 100 Hz)
    fig_ppr_noise                                              from the values of Table ppr-dct (2 September sweeps)
    orders_cet.csv / orders_dct.csv                            the numbers behind Table orders-cet

    python orders_figs.py --data "<Research SeBoCa Test folder>" --out ../Figs
Method (as in the chapter): interior of each step, mean removed, Hann window, amplitude spectrum; line amplitude from the power in +-0.5 Hz
around h*f_rot (a Hann-windowed sinusoid spreads over three bins, so the band power is divided by 1.5); harmonics above 50 Hz alias to
|h f - k 100| and are used only if more than 4 Hz from zero; broadband noise = rms of the spectrum 1-48 Hz outside +-1 Hz of the first four harmonics.
"""
import argparse, os
import numpy as np, pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

FS, T0, T1 = 100.0, 1.0, 9.0       # sample rate; interior window of each hold, seconds after the start of the step
COL = {"CET": "#0072B2", "DCT": "#D55E00"}


def style():
    plt.rcParams.update({"font.family": "serif", "font.serif": ["Times New Roman", "Liberation Serif", "DejaVu Serif"], "mathtext.fontset": "stix",
                         "font.size": 9, "axes.labelsize": 9.5, "axes.titlesize": 9.5, "legend.fontsize": 7.8, "axes.grid": True, "grid.color": "#DDDDDD",
                         "grid.linewidth": 0.5, "axes.axisbelow": True, "axes.spines.top": False, "axes.spines.right": False, "savefig.dpi": 300, "pdf.fonttype": 42})


def alias(f):
    return abs(f - FS * round(f / FS))


def spectrum(x):
    x = x - x.mean(); w = np.hanning(len(x)); X = np.fft.rfft(x * w); f = np.fft.rfftfreq(len(x), 1 / FS)
    return f, 2 * np.abs(X) / w.sum()


def analyse(path):
    d = pd.read_csv(path); rows, spec = [], {}
    for k, g in d.groupby("step_index"):
        t = g.elapsed_s.to_numpy(); m = (t >= t[0] + T0) & (t <= t[0] + T1); r = g.raw_rpm.to_numpy()[100:900]
        n = r.mean()
        if n < 100: continue
        f, a = spectrum(r); fr = n / 60; line, ang = {}, {}
        used = np.zeros_like(f, bool)
        for h in (1, 2, 3):
            fa = alias(h * fr); ok = fa > 4.0 and fa < 49
            if ok:
                b = (f >= fa - 0.5) & (f <= fa + 0.5); line[h] = np.sqrt(np.sum(a[b] ** 2) / 1.5)
            else: line[h] = np.nan
        mask = (f >= 1) & (f <= 48)
        for h in (1, 2, 3, 4): mask &= ~(np.abs(f - alias(h * fr)) <= 1.0)
        noise = np.sqrt(np.mean(a[mask] ** 2) / 1.5) if mask.any() else np.nan
        var = np.var(r, ddof=0); share = np.nansum([line[h] ** 2 / 2 for h in (1, 2, 3)]) / var * 100
        sinc = lambda h: abs(np.sin(np.pi * h * fr * 0.01) / (np.pi * h * fr * 0.01))
        rel1 = 100 * line[1] / n if np.isfinite(line[1]) else np.nan
        eps1 = np.degrees(line[1] / n / (sinc(1))) if np.isfinite(line[1]) else np.nan
        rows.append(dict(speed=n, sd=r.std(ddof=1), A1=line[1], A2=line[2], A3=line[3], rel1_pct=rel1, eps1_deg=eps1, share_pct=share, noise=noise))
        spec[round(n)] = (f, a, fr, n)
    return pd.DataFrame(rows), spec


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--data", required=True); ap.add_argument("--out", required=True); a = ap.parse_args(); os.makedirs(a.out, exist_ok=True); style()
    TC, SC = analyse(os.path.join(a.data, "cet_pwm_sweep.csv")); TD, SD = analyse(os.path.join(a.data, "dct_pwm_sweep.csv"))
    TC.round(3).to_csv(os.path.join(a.out, "orders_cet.csv"), index=False); TD.round(3).to_csv(os.path.join(a.out, "orders_dct.csv"), index=False)
    print(TC.round(2).to_string())

    # ---- spectra at four speeds ----
    pick = [0, 2, 4, 7]; kc, kd = sorted(SC), sorted(SD)
    fig, ax = plt.subplots(2, 2, figsize=(7.2, 4.8), sharex=True)
    for A, i in zip(ax.ravel(), pick):
        for lab, S, ks in (("DCT", SD, kd), ("CET", SC, kc)):
            f, am, fr, n = S[ks[i]]; A.plot(f, am, color=COL[lab], lw=0.8, alpha=0.9, label=f"{lab}, {n:.0f} rpm" if lab == "CET" else f"{lab}, {n:.0f} rpm")
        _, _, fr, n = SC[kc[i]]
        for h in (1, 2, 3):
            A.axvline(alias(h * fr), color="k", ls=":", lw=0.8)
            A.text(alias(h * fr), 0.97, f"{h}$f_{{rot}}$", transform=A.get_xaxis_transform(), fontsize=7, ha="center", va="top", rotation=0, bbox=dict(fc="white", ec="none", pad=0.5))
        A.set_yscale("log"); A.set_ylim(1e-3, 30); A.set_xlim(0, 50); A.legend(frameon=False, loc="lower left", fontsize=7.5)
    for A in ax[1]: A.set_xlabel("Frequency (Hz)")
    for A in ax[:, 0]: A.set_ylabel("Amplitude (rpm)")
    fig.tight_layout(); fig.savefig(os.path.join(a.out, "fig_orders_spectra.pdf")); fig.savefig(os.path.join(a.out, "fig_orders_spectra.png"), dpi=170); plt.close(fig)

    # ---- order map (CET) ----
    ks = sorted(SC); F = SC[ks[0]][0]; M = np.array([SC[k][1] for k in ks]); sp = np.array([SC[k][3] for k in ks])
    fig, A = plt.subplots(figsize=(5.6, 4.1))
    im = A.pcolormesh(sp, F, np.log10(np.clip(M.T, 1e-3, None)), shading="nearest", cmap="viridis", vmin=-2, vmax=0.8)
    xs = np.linspace(sp.min(), sp.max(), 200)
    for h, ls in ((1, "-"), (2, "--"), (3, ":")): A.plot(xs, [alias(h * x / 60) for x in xs], color="white", ls=ls, lw=0.9, label=f"{h}$f_{{rot}}$" + (" (aliased above 50 Hz)" if False else ""))
    A.set_ylim(0, 50); A.set_xlabel("Speed (rpm)"); A.set_ylabel("Frequency (Hz)"); A.grid(False); A.legend(frameon=True, facecolor="#333", labelcolor="white", loc="upper left", fontsize=7.5)
    cb = fig.colorbar(im, ax=A); cb.set_label(r"$\log_{10}$ amplitude (rpm)")
    fig.tight_layout(); fig.savefig(os.path.join(a.out, "fig_orders_map.pdf")); fig.savefig(os.path.join(a.out, "fig_orders_map.png"), dpi=170); plt.close(fig)

    # ---- amplitude and equivalent angle ----
    fig, ax = plt.subplots(1, 2, figsize=(7.2, 3.1))
    for lab, T, mk in (("CET", TC, "o"), ("DCT", TD, "s")):
        ax[0].plot(T.speed, T.A1, mk + "-", color=COL[lab], ms=4, lw=1, mec="white", mew=0.4, label=f"{lab}, $1f_{{rot}}$")
        ax[0].plot(T.speed, T.A2, mk + "--", color=COL[lab], ms=4, lw=1, mfc="white", label=f"{lab}, $2f_{{rot}}$")
        ax[1].plot(T.speed, T.eps1_deg, mk + "-", color=COL[lab], ms=4, lw=1, mec="white", mew=0.4, label=lab)
    ax[1].axhline(0.25, color="k", ls=":", lw=1); ax[1].text(ax[1].get_xlim()[1] * 0.98, 0.265, "rated accuracy 0.25$^{\\circ}$", ha="right", fontsize=7.5)
    ax[0].set_xlabel("Speed (rpm)"); ax[0].set_ylabel("Line amplitude (rpm)"); ax[0].legend(frameon=False, ncol=2, fontsize=7); ax[0].set_title("(a) Lines at one and two times the rotation frequency")
    ax[1].set_xlabel("Speed (rpm)"); ax[1].set_ylabel(r"Equivalent angular error, $\varepsilon_1$ ($^{\circ}$)"); ax[1].legend(frameon=False); ax[1].set_title("(b) Equivalent encoder angle error")
    fig.tight_layout(); fig.savefig(os.path.join(a.out, "fig_orders_amplitude.pdf")); fig.savefig(os.path.join(a.out, "fig_orders_amplitude.png"), dpi=170); plt.close(fig)

    # ---- noise against PPR (values of Table ppr-dct) ----
    ppr = np.array([48, 96, 100, 125, 192, 200, 250, 256])
    n220 = {10: [9.2, 8.0, 6.6, 5.1, 5.7, 6.8, 5.6, 4.5], 20: [6.8, 5.2, 6.1, 6.0, 6.3, 6.2, 5.9, 4.7]}
    a1 = {10: [10.1, 11.6, 7.6, 7.5, 8.4, 10.4, 10.5, 10.0], 20: [7.7, 6.2, 5.4, 4.7, 5.7, 7.2, 7.3, 6.4]}
    fig, ax = plt.subplots(1, 2, figsize=(7.2, 3.0)); cc = {10: "#0072B2", 20: "#D55E00"}; xx = np.linspace(40, 270, 100)
    for w in (10, 20):
        ax[0].plot(ppr, n220[w], "o-", color=cc[w], mec="white", mew=0.4, ms=5, lw=1, label=f"{w} ms window")
        ax[0].plot(xx, 60 / (w / 1000 * 4 * xx) / np.sqrt(12), ":", color=cc[w], lw=1.1)
        ax[1].plot(ppr, a1[w], "o-", color=cc[w], mec="white", mew=0.4, ms=5, lw=1, label=f"{w} ms window")
    ax[0].plot([], [], "k:", label=r"quantization, $60/(T_wN)/\sqrt{12}$"); ax[0].set_xlabel("PPR"); ax[0].set_ylabel("Broadband noise at 220 rpm (rpm)"); ax[0].legend(frameon=False); ax[0].set_ylim(0, 11); ax[0].set_title("(a) Noise at the 10 % step")
    ax[1].set_xlabel("PPR"); ax[1].set_ylabel(r"Line at $1f_{rot}$, mean of 9 steps (rpm)"); ax[1].set_ylim(0, 13); ax[1].legend(frameon=False); ax[1].set_title("(b) Rotation-frequency line")
    fig.tight_layout(); fig.savefig(os.path.join(a.out, "fig_ppr_noise.pdf")); fig.savefig(os.path.join(a.out, "fig_ppr_noise.png"), dpi=170); plt.close(fig)


if __name__ == "__main__":
    main()
