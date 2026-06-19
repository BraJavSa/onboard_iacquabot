#!/usr/bin/env python3
"""
generate_pdf_report.py
======================
Generates a multi-page PDF report with one page per experiment (up to 10),
each showing measured vs. Stage 2 (Joint Opt) simulated velocities (u, v, r).
Also saves comparative RMS metrics (Stage 1 vs. Stage 2) to a JSON file.
"""

LANGUAGE = "en"   # "en" = English  |  "es" = Spanish

import argparse
import json
import os
import sys
import re
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.gridspec import GridSpec
from matplotlib.ticker import AutoMinorLocator
from matplotlib.backends.backend_pdf import PdfPages
import numpy as np
from scipy.io import loadmat
from scipy.signal import savgol_filter

# ═══════════════════════════════════════════════════════════════════════════
#  Integration and Helper functions
# ═══════════════════════════════════════════════════════════════════════════

def build_allocation_matrix(positions_yx, angles_deg):
    positions_yx = np.asarray(positions_yx, dtype=float)
    angles = np.deg2rad(np.asarray(angles_deg, dtype=float))
    y = positions_yx[:, 0]
    x = positions_yx[:, 1]
    c = np.cos(angles)
    s = np.sin(angles)
    return np.vstack([c, s, y * c + x * s])

def branch_thrust(cmd, params):
    return params["A"] + (params["K"] - params["A"]) / (
        params["C"] + np.exp(-params["B"] * (cmd - params["M"]))
    ) ** (1.0 / params["v"])

def cmd_to_thrust(cmd, motor):
    cmd = np.asarray(cmd, dtype=float)
    thrust = np.zeros_like(cmd)
    pos = cmd > 0.01
    neg = cmd < -0.01
    thrust[pos] = branch_thrust(cmd[pos], motor["T200"]["pos"])
    thrust[neg] = branch_thrust(cmd[neg], motor["T200"]["neg"])
    return np.clip(thrust, motor["max_rev"], motor["max_fwd"])

def pwm_to_cmd(pwm_us, inverted, motor):
    cmd = (np.asarray(pwm_us) - motor["pwm_mid"]) / (motor["pwm_max"] - motor["pwm_mid"])
    return -cmd if inverted else cmd

def experiment_to_tau(experiment, motor):
    cmd = [
        pwm_to_cmd(experiment["pwm_raw"][i], motor["motor_inverted"][i], motor)
        for i in range(4)
    ]
    thrust = [cmd_to_thrust(cmd[i], motor) for i in range(4)]
    tmat = np.vstack([
        thrust[3],
        thrust[1],
        thrust[2],
        thrust[0],
    ])
    b_alloc = build_allocation_matrix(motor["positions_yx"], motor["angles_deg"])
    tau = b_alloc @ tmat
    return tau[0], tau[2]

def rk4_integrate(t, u0, v0, r0, Tu, Tr, dynamic):
    N = len(t)
    u_sim = np.zeros(N)
    v_sim = np.zeros(N)
    r_sim = np.zeros(N)
    u_sim[0], v_sim[0], r_sim[0] = u0, v0, r0
    m11 = dynamic["m11"]
    m22 = dynamic["m22"]
    m33 = dynamic["m33"]
    Xu = dynamic["Xu"]
    Xuu = dynamic["Xuu"]
    Yv = dynamic["Yv"]
    Yvv = dynamic["Yvv"]
    Nr = dynamic["Nr"]
    Nrr = dynamic["Nrr"]
    for k in range(N - 1):
        dt = t[k+1] - t[k]
        uk, vk, rk = u_sim[k], v_sim[k], r_sim[k]
        tu1, tr1 = Tu[k], Tr[k]
        du1 = (tu1 + m22 * vk * rk - Xu * uk - Xuu * abs(uk) * uk) / m11
        dv1 = (-m11 * uk * rk - Yv * vk - Yvv * abs(vk) * vk) / m22
        dr1 = (tr1 - (m22 - m11) * uk * vk - Nr * rk - Nrr * abs(rk) * rk) / m33
        
        u2 = uk + 0.5 * dt * du1
        v2 = vk + 0.5 * dt * dv1
        r2 = rk + 0.5 * dt * dr1
        tu2 = 0.5 * (Tu[k] + Tu[k+1])
        tr2 = 0.5 * (Tr[k] + Tr[k+1])
        du2 = (tu2 + m22 * v2 * r2 - Xu * u2 - Xuu * abs(u2) * u2) / m11
        dv2 = (-m11 * u2 * r2 - Yv * v2 - Yvv * abs(v2) * v2) / m22
        dr2 = (tr2 - (m22 - m11) * u2 * v2 - Nr * r2 - Nrr * abs(r2) * r2) / m33
        
        u3 = uk + 0.5 * dt * du2
        v3 = vk + 0.5 * dt * dv2
        r3 = rk + 0.5 * dt * dr2
        du3 = (tu2 + m22 * v3 * r3 - Xu * u3 - Xuu * abs(u3) * u3) / m11
        dv3 = (-m11 * v3 * r3 - Yv * v3 - Yvv * abs(v3) * v3) / m22
        dr3 = (tr2 - (m22 - m11) * u3 * v3 - Nr * r3 - Nrr * abs(r3) * r3) / m33
        
        u4 = uk + dt * du3
        v4 = vk + dt * dv3
        r4 = rk + dt * dr3
        tu4, tr4 = Tu[k+1], Tr[k+1]
        du4 = (tu4 + m22 * v4 * r4 - Xu * u4 - Xuu * abs(u4) * u4) / m11
        dv4 = (-m11 * u4 * r4 - Yv * v4 - Yvv * abs(v4) * v4) / m22
        dr4 = (tr4 - (m22 - m11) * u4 * v4 - Nr * r4 - Nrr * abs(r4) * r4) / m33
        
        u_sim[k+1] = uk + (dt / 6.0) * (du1 + 2.0*du2 + 2.0*du3 + du4)
        v_sim[k+1] = vk + (dt / 6.0) * (dv1 + 2.0*dv2 + 2.0*dv3 + dv4)
        r_sim[k+1] = rk + (dt / 6.0) * (dr1 + 2.0*dr2 + 2.0*dr3 + dr4)
        
        if not (np.isfinite(u_sim[k+1]) and np.isfinite(v_sim[k+1]) and np.isfinite(r_sim[k+1])):
            raise RuntimeError("Integration diverged")
    return u_sim, v_sim, r_sim

def load_experiment(path, dt=1.0/30.0):
    data = loadmat(path)
    pwm_raw = np.array([data[f"pwm{i+1}"].squeeze() for i in range(4)])
    u_raw = data["vx"].squeeze()
    u = savgol_filter(u_raw, 25, 2)
    v = savgol_filter(data["vy"].squeeze(), 25, 2)
    r = savgol_filter(data["wz"].squeeze(), 25, 2)
    return {
        "name": os.path.basename(path),
        "t": data["t"].squeeze() if "t" in data else np.arange(len(u_raw)) * dt,
        "u": u,
        "v": v,
        "r": r,
        "u_raw": u_raw,
        "v_raw": data["vy"].squeeze(),
        "r_raw": data["wz"].squeeze(),
        "acc_u": np.gradient(u, dt),
        "acc_v": np.gradient(v, dt),
        "acc_r": np.gradient(r, dt),
        "pwm_raw": pwm_raw,
    }

def rms(y_meas, y_sim):
    return float(np.sqrt(np.mean((np.asarray(y_meas) - np.asarray(y_sim)) ** 2)))

def mae(y_meas, y_sim):
    return float(np.mean(np.abs(np.asarray(y_meas) - np.asarray(y_sim))))

def r2(y_meas, y_sim):
    y_meas = np.asarray(y_meas)
    y_sim  = np.asarray(y_sim)
    ss_res = np.sum((y_meas - y_sim) ** 2)
    ss_tot = np.sum((y_meas - np.mean(y_meas)) ** 2)
    if ss_tot == 0.0:
        return 1.0 if ss_res == 0.0 else 0.0
    return float(1.0 - ss_res / ss_tot)

# ═══════════════════════════════════════════════════════════════════════════
#  Localization
# ═══════════════════════════════════════════════════════════════════════════

STRINGS = {
    "en": {
        "measured": "Measured",
        "model": "Modeled",
        "error_region": "Error Region",
        "time_label": "Time (s)",
        "ylabels": {
            "u": "Surge velocity\n$u$  [m/s]",
            "v": "Sway velocity\n$v$  [m/s]",
            "r": "Yaw rate\n$r$  [rad/s]",
        },
    },
}

COLOR_MEASURED = "#000000"
COLOR_MODEL    = "#1a56a0" # Premium steel blue
COLOR_FILL     = "#d0e1f9" # Light matching blue for shaded error region

YLIMS = {
    "u": (-1.0,  1.6),
    "v": (-1.0,  1.0),
    "r": (-1.0,  1.0),
}

def apply_article_style():
    plt.rcParams.update({
        "figure.facecolor":    "white",
        "axes.facecolor":      "white",
        "axes.edgecolor":      "black",
        "axes.linewidth":      0.8,
        "xtick.direction":     "in",
        "ytick.direction":     "in",
        "xtick.major.size":    4.0,
        "ytick.major.size":    4.0,
        "xtick.minor.size":    2.5,
        "ytick.minor.size":    2.5,
        "xtick.major.width":   0.7,
        "ytick.major.width":   0.7,
        "xtick.color":         "black",
        "ytick.color":         "black",
        "xtick.labelsize":     8,
        "ytick.labelsize":     8,
        "axes.grid":           True,
        "grid.color":          "#d0d0d0",
        "grid.linewidth":      0.45,
        "grid.linestyle":      "--",
        "font.family":         "serif",
        "font.serif":          ["Times New Roman", "DejaVu Serif", "serif"],
        "mathtext.fontset":    "dejavuserif",
        "axes.labelsize":      9,
        "axes.labelcolor":     "black",
        "text.color":          "black",
        "legend.frameon":      True,
        "legend.framealpha":   1.0,
        "legend.edgecolor":    "black",
        "legend.facecolor":    "white",
        "legend.fontsize":     8,
        "legend.handlelength": 2.2,
        "lines.linewidth":     1.2,
    })

def make_page(ds, sim_s2, exp_name: str, lang: str, exp_metrics) -> plt.Figure:
    S = STRINGS[lang]
    t = ds["t"]
    u_s2, v_s2, r_s2 = sim_s2
    
    signals = [
        ("u", ds["u_raw"], u_s2),
        ("v", ds["v_raw"], v_s2),
        ("r", ds["r_raw"], r_s2),
    ]

    fig = plt.figure(figsize=(7.5, 9.5))
    fig.patch.set_facecolor("white")

    # Parse digit from experiment_xx
    match = re.search(r'\d+', exp_name)
    num_str = match.group(0) if match else exp_name
    title_str = f"Experiment {num_str}"

    # Title
    fig.text(
        0.5, 0.985,
        f"{title_str}",
        ha="center", va="top",
        fontsize=12, fontweight="bold",
        fontfamily="serif",
    )

    gs = GridSpec(
        3, 1,
        figure=fig,
        left=0.12, right=0.96,
        top=0.94, bottom=0.06,
        hspace=0.34,
    )

    for row, (key, y_meas, y_sim) in enumerate(signals):
        ax = fig.add_subplot(gs[row])

        rms_val = exp_metrics[key]["rms"]
        mae_val = exp_metrics[key]["mae"]

        # Plot measured
        ax.plot(t, y_meas, color=COLOR_MEASURED, linewidth=1.2, linestyle="-", label=S["measured"], zorder=4)

        # Plot Stage 2 model
        ax.plot(t, y_sim, color=COLOR_MODEL, linewidth=1.1, linestyle="--", label=S["model"], zorder=5)

        # Error shading in light blue matching the model color
        ax.fill_between(
            t, y_meas, y_sim,
            color=COLOR_FILL, alpha=0.55,
            linewidth=0, zorder=2,
        )

        # Axis styling
        ax.set_xlim(t[0], t[-1])
        ax.set_ylim(YLIMS[key])
        ax.set_ylabel(S["ylabels"][key], labelpad=6, multialignment="center")
        ax.yaxis.set_minor_locator(AutoMinorLocator(4))
        ax.xaxis.set_minor_locator(AutoMinorLocator(4))
        ax.tick_params(which="both", top=True, right=True)

        if row < 2:
            ax.set_xticklabels([])
        else:
            ax.set_xlabel(S["time_label"], labelpad=3)

        err_patch = mpatches.Patch(
            facecolor=COLOR_FILL, alpha=0.6,
            edgecolor="none", label=S["error_region"],
        )
        metrics_handle = mpatches.Patch(
            visible=False,
            label=f"RMS={rms_val:.4f}  MAE={mae_val:.4f}",
        )
        
        handles, labels = ax.get_legend_handles_labels()
        ax.legend(
            handles + [err_patch, metrics_handle],
            labels  + [S["error_region"],
                       f"RMS={rms_val:.4f}  MAE={mae_val:.4f}"],
            loc="upper right",
            fontsize=7.5,
            borderpad=0.4,
            labelspacing=0.25,
        )

    return fig

def main():
    parser = argparse.ArgumentParser(description="Generate PDF report for Stage 2 validation.")
    parser.add_argument("--data-dir", default=os.path.dirname(os.path.abspath(__file__)))
    args = parser.parse_args()
    
    lang = LANGUAGE
    data_dir = args.data_dir
    
    dyn_s2_path = os.path.join(data_dir, "identified_dynamics.json")
    motor_s2_path = os.path.join(data_dir, "identified_motors.json")
    metrics_s2_path = os.path.join(data_dir, "identified_dynamics_metrics.json")
    output_pdf = os.path.join(data_dir, "experiment_validation.pdf")
    
    for p, desc in [(dyn_s2_path, "Definitive dynamics"), (motor_s2_path, "Definitive motors"), (metrics_s2_path, "Definitive metrics")]:
        if not os.path.exists(p):
            print(f"[ERROR] Required JSON file not found: {p} ({desc})")
            sys.exit(1)
            
    dyn_s2 = json.load(open(dyn_s2_path, "r"))
    motor_s2 = json.load(open(motor_s2_path, "r"))
    metrics_s2 = json.load(open(metrics_s2_path, "r"))
    
    # Load validation experiments (1 to 10)
    exp_paths = []
    for i in range(1, 11):
        p = os.path.join(data_dir, f"experiment_{i:02d}.mat")
        if os.path.exists(p):
            exp_paths.append(p)
            
    if not exp_paths:
        print(f"[ERROR] No experiment mat files found in {data_dir}")
        sys.exit(1)
        
    print(f"Generating validation report for Stage 2. Language: {lang}")
    apply_article_style()
    
    pages_to_write = []
    
    for exp_path in exp_paths:
        exp_name = os.path.splitext(os.path.basename(exp_path))[0]
        print(f"  Simulating {exp_name}...", end=" ", flush=True)
        try:
            ds = load_experiment(exp_path)
            
            # Simulate Stage 2
            Tu_s2, Tr_s2 = experiment_to_tau(ds, motor_s2)
            u_s2, v_s2, r_s2 = rk4_integrate(ds["t"], ds["u_raw"][0], ds["v_raw"][0], ds["r_raw"][0], Tu_s2, Tr_s2, dyn_s2)
            
            # Prepare plotting figure for this page (Plotting ONLY Measured and Stage 2 simulation)
            fig_page = make_page(ds, (u_s2, v_s2, r_s2), exp_name, lang, metrics_s2[exp_name + ".mat"])
            pages_to_write.append(fig_page)
            print("done")
        except Exception as e:
            print(f"FAILED ({e})")
            
    if not pages_to_write:
        print("[ERROR] No experiments were simulated successfully.")
        sys.exit(1)
        
    with PdfPages(output_pdf) as pdf:
        for fig_page in pages_to_write:
            pdf.savefig(fig_page, dpi=300, bbox_inches="tight")
            plt.close(fig_page)
            
    print(f"Comparative PDF report generated successfully at: {output_pdf}")

if __name__ == "__main__":
    main()