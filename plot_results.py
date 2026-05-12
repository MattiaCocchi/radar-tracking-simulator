"""
plot_results_v2.py  —  Visualise the Tire-Road Friction Estimator v2.0 output
Usage: python3 plot_results_v2.py [friction_v2_results.csv]

CSV columns (13 total, written by TelemetryLogger in TelemetryRow order):
  t_ms, true_omega_rad_s, meas_omega_rad_s, ekf_omega_rad_s,
  true_mu, ekf_mu, ekf_mu_std, innovation_rad_s, mahalanobis,
  tau_Nm, confidence, mode, temp_c
"""
import sys
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib.gridspec as gridspec
from matplotlib.collections import LineCollection
from matplotlib.colors import Normalize

# ── Config ─────────────────────────────────────────────────────────────────────
CSV = sys.argv[1] if len(sys.argv) > 1 else "friction_v2_results.csv"

# ── Load CSV ──────────────────────────────────────────────────────────────────
df = pd.read_csv(CSV)

# ── Expected columns (must match TelemetryRow + CSV header exactly) ───────────
EXPECTED_COLS = [
    "t_ms", "true_omega_rad_s", "meas_omega_rad_s", "ekf_omega_rad_s",
    "true_mu", "ekf_mu", "ekf_mu_std",
    "innovation_rad_s", "mahalanobis",
    "tau_Nm",
    "confidence", "mode", "temp_c",
]
for col in EXPECTED_COLS:
    if col not in df.columns:
        df[col] = np.nan

# Sanity clamp
df["confidence"] = df["confidence"].clip(0.0, 1.0)
df["mode"]       = df["mode"].round()

print(f"Loaded {len(df)} rows from '{CSV}'")
print(f"Time span: {df.t_ms.min():.0f} – {df.t_ms.max():.0f} ms")
print(f"Confidence range: {df.confidence.min():.3f} – {df.confidence.max():.3f}")
print(f"EKF μ range:      {df.ekf_mu.min():.3f} – {df.ekf_mu.max():.3f}")

# ── Surface events (hardcoded to match main.cpp scenario) ─────────────────────
events = [
    (500,  "DRY→ICE\n(μ=0.10)", "#4A90D9", "ICE"),
    (3000, "ICE→WET\n(μ=0.60)", "#E67E22", "WET"),
    (6000, "WET→DRY\n(μ=1.00)", "#27AE60", "DRY"),
]

# ── Style ──────────────────────────────────────────────────────────────────────
plt.rcParams.update({
    "font.family":       "monospace",
    "axes.facecolor":    "#0F1117",
    "figure.facecolor":  "#0F1117",
    "axes.edgecolor":    "#2A2D3A",
    "axes.labelcolor":   "#C8D0E0",
    "xtick.color":       "#6B7280",
    "ytick.color":       "#6B7280",
    "grid.color":        "#1E2130",
    "grid.linewidth":    0.6,
    "text.color":        "#C8D0E0",
    "axes.titlecolor":   "#E8EDF5",
    "axes.titleweight":  "bold",
    "axes.titlesize":    11,
    "axes.labelsize":    9,
})

COLORS = {
    "true":       "#E8EDF5",
    "ekf":        "#FF4D6D",
    "meas":       "#4A90D9",
    "confidence": "#FFD166",
    "torque":     "#A78BFA",
    "temp":       "#F97316",
    "freeze_bg":  "#1A0A00",
    "freeze_ln":  "#FF6B00",
}

# ── Figure layout ──────────────────────────────────────────────────────────────
fig = plt.figure(figsize=(16, 11))
fig.suptitle(
    "EKF Virtual Friction Sensor  ·  Real-Time Tire-Road μ Estimator  v2.0",
    fontsize=14, fontweight="bold", color="#E8EDF5", y=0.98
)

gs = gridspec.GridSpec(
    3, 2,
    figure=fig,
    hspace=0.50, wspace=0.30,
    left=0.07, right=0.97, top=0.94, bottom=0.07
)

ax_omega = fig.add_subplot(gs[0, :])
ax_mu    = fig.add_subplot(gs[1, :])
ax_conf  = fig.add_subplot(gs[2, 0])
ax_env   = fig.add_subplot(gs[2, 1])


# ── Helper: shade FREEZE regions ──────────────────────────────────────────────
def shade_freeze(ax, df, alpha=0.18):
    if df["mode"].isna().all():
        return
    freeze_mask = df["mode"] == 1.0
    if not freeze_mask.any():
        return

    transitions = freeze_mask.astype(int).diff().fillna(0)
    starts = df.loc[transitions == 1,  "t_ms"].values
    ends   = df.loc[transitions == -1, "t_ms"].values

    if freeze_mask.iloc[0]:
        starts = np.concatenate([[df["t_ms"].iloc[0]], starts])
    if freeze_mask.iloc[-1]:
        ends = np.concatenate([ends, [df["t_ms"].iloc[-1]]])

    for s, e in zip(starts, ends):
        ax.axvspan(s, e, color=COLORS["freeze_bg"], alpha=alpha, zorder=0)
        ax.axvline(s, color=COLORS["freeze_ln"], lw=0.8, alpha=0.6, ls="--", zorder=1)

    patch = mpatches.Patch(color=COLORS["freeze_ln"], alpha=0.5, label="EKF FREEZE mode")
    handles, labels = ax.get_legend_handles_labels()
    if "EKF FREEZE mode" not in labels:
        handles.append(patch)
        ax.legend(handles=handles, loc="upper right",
                  fontsize=7.5, framealpha=0.2, facecolor="#0F1117")


# ── Helper: mark surface events ───────────────────────────────────────────────
def mark_events(ax, df):
    ymin, ymax = ax.get_ylim()
    for t_ms, label, color, _ in events:
        if df["t_ms"].min() <= t_ms <= df["t_ms"].max():
            ax.axvline(t_ms, color=color, lw=1.0, ls="--", alpha=0.7, zorder=2)
            ax.text(t_ms + (df["t_ms"].max() * 0.005), ymax * 0.92,
                    label, color=color, fontsize=7, va="top", zorder=3)


# ── Plot 1: Angular Velocity ───────────────────────────────────────────────────
ax = ax_omega
ax.set_facecolor("#0F1117")

ax.plot(df.t_ms, df.true_omega_rad_s,
        color=COLORS["true"], lw=1.8, label="True ω", zorder=4)
ax.scatter(df.t_ms[::3], df.meas_omega_rad_s[::3],
           s=1.2, color=COLORS["meas"], alpha=0.35, label="ABS meas (noisy)", zorder=3)
ax.plot(df.t_ms, df.ekf_omega_rad_s,
        color=COLORS["ekf"], lw=1.5, label="EKF ω̂", zorder=5)

ax2_twin = ax.twinx()
omega_err = (df.true_omega_rad_s - df.ekf_omega_rad_s).abs()
ax2_twin.fill_between(df.t_ms, omega_err, alpha=0.15, color=COLORS["ekf"], label="|error|")
ax2_twin.set_ylabel("|Δω| [rad/s]", color=COLORS["ekf"], fontsize=8)
ax2_twin.tick_params(axis='y', colors=COLORS["ekf"], labelsize=7)
ax2_twin.set_ylim(0, omega_err.quantile(0.99) * 4)
ax2_twin.set_facecolor("#0F1117")

shade_freeze(ax, df)
ax.set_ylabel("ω [rad/s]")
ax.set_title("① Wheel Angular Velocity — Truth · ABS Sensor · EKF Estimate")
ax.legend(loc="upper left", fontsize=7.5, framealpha=0.2, facecolor="#0F1117")
ax.grid(True)
ax.set_xlim(df.t_ms.min(), df.t_ms.max())
mark_events(ax, df)


# ── Plot 2: Friction Coefficient μ ────────────────────────────────────────────
ax = ax_mu
ax.set_facecolor("#0F1117")

# ±1σ band — now uses the real ekf_mu_std column
if not df["ekf_mu_std"].isna().all():
    ax.fill_between(
        df.t_ms,
        df.ekf_mu - df.ekf_mu_std,
        df.ekf_mu + df.ekf_mu_std,
        alpha=0.25, color=COLORS["ekf"], label="EKF ±1σ"
    )

ax.plot(df.t_ms, df.true_mu,
        color=COLORS["true"], lw=2.2, label="True μ (ground truth)", zorder=5)
ax.plot(df.t_ms, df.ekf_mu,
        color=COLORS["ekf"], lw=1.6, label="EKF μ̂ estimate", zorder=4)

# Colour the EKF line by confidence score (gradient)
if not df["confidence"].isna().all():
    points   = np.array([df.t_ms, df.ekf_mu]).T.reshape(-1, 1, 2)
    segments = np.concatenate([points[:-1], points[1:]], axis=1)
    norm = Normalize(vmin=0, vmax=1)
    lc = LineCollection(segments, cmap="RdYlGn", norm=norm, linewidth=2.0, zorder=6)
    lc.set_array(df["confidence"].values)
    ax.add_collection(lc)
    cbar = fig.colorbar(lc, ax=ax, pad=0.01, fraction=0.015)
    cbar.set_label("Confidence", fontsize=7, color="#C8D0E0")
    cbar.ax.yaxis.set_tick_params(color="#C8D0E0", labelsize=6)

shade_freeze(ax, df)
ax.set_ylabel("Friction coefficient μ [−]")
ax.set_title("② Virtual Sensor Output — EKF μ Estimate vs Ground Truth  (colour = confidence)")
ax.set_ylim(-0.05, 1.30)
ax.legend(loc="upper right", fontsize=7.5, framealpha=0.2, facecolor="#0F1117")
ax.grid(True)
ax.set_xlim(df.t_ms.min(), df.t_ms.max())
mark_events(ax, df)


# ── Plot 3: Confidence & Mode ──────────────────────────────────────────────────
ax = ax_conf
ax.set_facecolor("#0F1117")

if not df["confidence"].isna().all():
    ax.plot(df.t_ms, df["confidence"],
            color=COLORS["confidence"], lw=1.4, label="Confidence [0,1]")
    ax.axhline(0.7, color="#27AE60", lw=0.8, ls="--", label="High threshold (0.7)")
    ax.axhline(0.3, color=COLORS["ekf"],  lw=0.8, ls="--", label="Low threshold (0.3)")
    ax.fill_between(df.t_ms, 0.7, 1.0, alpha=0.07, color="#27AE60")
    ax.fill_between(df.t_ms, 0.0, 0.3, alpha=0.07, color=COLORS["ekf"])

shade_freeze(ax, df)
ax.set_ylim(0, 1.1)
ax.set_ylabel("Score [0-1]")
ax.set_xlabel("Time [ms]")
ax.set_title("③ Confidence Score & FREEZE Regions")
ax.legend(loc="upper right", fontsize=7, framealpha=0.2, facecolor="#0F1117")
ax.grid(True)
ax.set_xlim(df.t_ms.min(), df.t_ms.max())
mark_events(ax, df)


# ── Plot 4: Environment — Torque & Temperature ────────────────────────────────
ax = ax_env
ax.set_facecolor("#0F1117")

if not df["tau_Nm"].isna().all():
    ax.fill_between(df.t_ms, df["tau_Nm"], 0, alpha=0.4, color=COLORS["torque"])
    ax.plot(df.t_ms, df["tau_Nm"],
            color=COLORS["torque"], lw=1.2, label="Torque τ [N·m]")
    ax.set_ylabel("Torque τ [N·m]", color=COLORS["torque"])
    ax.tick_params(axis='y', colors=COLORS["torque"])

ax_temp = ax.twinx()
ax_temp.set_facecolor("#0F1117")
if not df["temp_c"].isna().all():
    ax_temp.plot(df.t_ms, df["temp_c"],
                 color=COLORS["temp"], lw=1.4, ls="-.", label="Tyre Temp [°C]")
    ax_temp.set_ylabel("Temperature [°C]", color=COLORS["temp"])
    ax_temp.tick_params(axis='y', colors=COLORS["temp"])

shade_freeze(ax, df)
ax.set_xlabel("Time [ms]")
ax.set_title("④ Applied Torque & Tyre Temperature Profile")
lines1, lab1 = ax.get_legend_handles_labels()
lines2, lab2 = ax_temp.get_legend_handles_labels()
ax.legend(lines1 + lines2, lab1 + lab2,
          loc="lower right", fontsize=7, framealpha=0.2, facecolor="#0F1117")
ax.grid(True)
ax.set_xlim(df.t_ms.min(), df.t_ms.max())


# ── Surface legend strip ───────────────────────────────────────────────────────
fig.text(0.50, 0.005,
         "  ".join(f"▶ {label.replace(chr(10),' ')}" for _, label, c, _ in events),
         ha="center", va="bottom", fontsize=8, color="#6B7280")

# ── Save ──────────────────────────────────────────────────────────────────────
OUT = "friction_v2_estimate.png"
plt.savefig(OUT, dpi=150, bbox_inches="tight", facecolor=fig.get_facecolor())
plt.show()
print(f"\nSaved: {OUT}")