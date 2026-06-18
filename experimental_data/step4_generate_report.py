#!/usr/bin/env python3
"""
Step 4: Generate PDF validation report.
This script runs simulations on all 10 experiments using the final asymmetric parameters 
and generates a multi-page PDF report plotting measured vs. simulated dynamics (u, v, r) 
with error shading and statistical metrics (RMS, R^2, MAE) for each experiment.
"""
import argparse
import json
import os
import sys
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.gridspec import GridSpec
from matplotlib.ticker import AutoMinorLocator
from matplotlib.backends.backend_pdf import PdfPages
import numpy as np
from scipy.io import loadmat
from scipy.integrate import solve_ivp
from scipy.interpolate import interp1d

MOTOR_TEMPLATE = {
    "order": ["FR", "FL", "BR", "BL"],
    "T200": {
        "pos": {"A": 0.000001, "K": 40.0209, "B": 2.6249, "v": 0.1615, "C": 0.9432, "M": 0.00001},
        "neg": {"A": -31.4990, "K": -0.00001, "B": 3.6986, "v": 0.3264, "C": 0.9713, "M": -1.0000},
    },
    "max_fwd": 36.3827,
    "max_rev": -28.4393,
    "pwm_mid": 1500,
    "pwm_max": 1900,
    "motor_inverted": [True, True, False, False],
    "positions_yx": [
        [-0.35, 0.40],
        [0.35, 0.40],
        [-0.35, -0.50],
        [0.35, -0.50],
    ],
    "angles_deg": [0.0, 0.0, 0.0, 0.0],
}

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

def load_experiment(path):
    data = loadmat(path)
    pwm_raw = np.array([data[f"pwm{i+1}"].squeeze() for i in range(4)])
    return {
        "name": os.path.basename(path),
        "t": data["t"].squeeze(),
        "u": data["vx"].squeeze(),
        "v": data["vy"].squeeze(),
        "r": data["wz"].squeeze(),
        "pwm_raw": pwm_raw,
    }

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
    return tau[0], tau[1], tau[2]

def rk4_integrate(t, u0, v0, r0, Tu, Tr, dynamic):
    N = len(t)
    u_sim = np.zeros(N)
    v_sim = np.zeros(N)
    r_sim = np.zeros(N)
    u_sim[0] = u0
    v_sim[0] = v0
    r_sim[0] = r0
    m11 = dynamic["m11"]
    m22 = dynamic["m22"]
    m33 = dynamic["m33"]
    Xu_pos = dynamic["Xu_pos"]
    Xu_neg = dynamic["Xu_neg"]
    Xuu_pos = dynamic["Xuu_pos"]
    Xuu_neg = dynamic["Xuu_neg"]
    Yv = dynamic["Yv"]
    Yvv = dynamic["Yvv"]
    Yvr = dynamic["Yvr"]
    Yr = dynamic["Yr"]
    Yrv = dynamic["Yrv"]
    Yrr = dynamic["Yrr"]
    Yur = dynamic["Yur"]
    Nv = dynamic["Nv"]
    Nvv = dynamic["Nvv"]
    Nvr = dynamic["Nvr"]
    Nr = dynamic["Nr"]
    Nrv = dynamic["Nrv"]
    Nrr = dynamic["Nrr"]
    for k in range(N - 1):
        dt = t[k+1] - t[k]
        uk = u_sim[k]
        vk = v_sim[k]
        rk = r_sim[k]
        tu1 = Tu[k]
        tr1 = Tr[k]
        s1 = 0.5 * (1.0 + np.tanh(20.0 * uk))
        Xu_k1 = Xu_neg + (Xu_pos - Xu_neg) * s1
        Xuu_k1 = Xuu_neg + (Xuu_pos - Xuu_neg) * s1
        du1 = (tu1 + m22 * vk * rk - Xu_k1 * uk - Xuu_k1 * abs(uk) * uk) / m11
        dv1 = (-m11 * uk * rk - Yv * vk - Yvv * abs(vk) * vk - Yvr * abs(rk) * vk - Yr * rk - Yrv * abs(vk) * rk - Yrr * abs(rk) * rk - Yur * uk * rk) / m22
        dr1 = (tr1 - (m22 - m11) * uk * vk - Nv * vk - Nvv * abs(vk) * vk - Nvr * abs(rk) * vk - Nr * rk - Nrv * abs(vk) * rk - Nrr * abs(rk) * rk) / m33
        
        u2 = uk + 0.5 * dt * du1
        v2 = vk + 0.5 * dt * dv1
        r2 = rk + 0.5 * dt * dr1
        tu2 = 0.5 * (Tu[k] + Tu[k+1])
        tr2 = 0.5 * (Tr[k] + Tr[k+1])
        s2 = 0.5 * (1.0 + np.tanh(20.0 * u2))
        Xu_k2 = Xu_neg + (Xu_pos - Xu_neg) * s2
        Xuu_k2 = Xuu_neg + (Xuu_pos - Xuu_neg) * s2
        du2 = (tu2 + m22 * v2 * r2 - Xu_k2 * u2 - Xuu_k2 * abs(u2) * u2) / m11
        dv2 = (-m11 * u2 * r2 - Yv * v2 - Yvv * abs(v2) * v2 - Yvr * abs(r2) * v2 - Yr * r2 - Yrv * abs(v2) * r2 - Yrr * abs(r2) * r2 - Yur * u2 * r2) / m22
        dr2 = (tr2 - (m22 - m11) * u2 * v2 - Nv * v2 - Nvv * abs(v2) * v2 - Nvr * abs(r2) * v2 - Nr * r2 - Nrv * abs(v2) * r2 - Nrr * abs(r2) * r2) / m33
        
        u3 = uk + 0.5 * dt * du2
        v3 = vk + 0.5 * dt * dv2
        r3 = rk + 0.5 * dt * dr2
        s3 = 0.5 * (1.0 + np.tanh(20.0 * u3))
        Xu_k3 = Xu_neg + (Xu_pos - Xu_neg) * s3
        Xuu_k3 = Xuu_neg + (Xuu_pos - Xuu_neg) * s3
        du3 = (tu2 + m22 * v3 * r3 - Xu_k3 * u3 - Xuu_k3 * abs(u3) * u3) / m11
        dv3 = (-m11 * u3 * r3 - Yv * v3 - Yvv * abs(v3) * v3 - Yvr * abs(r3) * v3 - Yr * r3 - Yrv * abs(v3) * r3 - Yrr * abs(r3) * r3 - Yur * u3 * r3) / m22
        dr3 = (tr2 - (m22 - m11) * u3 * v3 - Nv * v3 - Nvv * abs(v3) * v3 - Nvr * abs(r3) * v3 - Nr * r3 - Nrv * abs(v3) * r3 - Nrr * abs(r3) * r3) / m33
        
        u4 = uk + dt * du3
        v4 = vk + dt * dv3
        r4 = rk + dt * dr3
        tu4 = Tu[k+1]
        tr4 = Tr[k+1]
        s4 = 0.5 * (1.0 + np.tanh(20.0 * u4))
        Xu_k4 = Xu_neg + (Xu_pos - Xu_neg) * s4
        Xuu_k4 = Xuu_neg + (Xuu_pos - Xuu_neg) * s4
        du4 = (tu4 + m22 * v4 * r4 - Xu_k4 * u4 - Xuu_k4 * abs(u4) * u4) / m11
        dv4 = (-m11 * u4 * r4 - Yv * v4 - Yvv * abs(v4) * v4 - Yvr * abs(r4) * v4 - Yr * r4 - Yrv * abs(v4) * r4 - Yrr * abs(r4) * r4 - Yur * u4 * r4) / m22
        dr4 = (tr4 - (m22 - m11) * u4 * v4 - Nv * v4 - Nvv * abs(v4) * v4 - Nvr * abs(r4) * v4 - Nr * r4 - Nrv * abs(v4) * r4 - Nrr * abs(r4) * r4) / m33
        
        u_sim[k+1] = uk + (dt / 6.0) * (du1 + 2.0*du2 + 2.0*du3 + du4)
        v_sim[k+1] = vk + (dt / 6.0) * (dv1 + 2.0*dv2 + 2.0*dv3 + dv4)
        r_sim[k+1] = rk + (dt / 6.0) * (dr1 + 2.0*dr2 + 2.0*dr3 + dr4)
    return u_sim, v_sim, r_sim

def simulate_asymmetric_experiment(experiment, dynamic, motor, fast=False):
    t = experiment["t"]
    u = experiment["u"]
    v = experiment["v"]
    r = experiment["r"]
    tau_u, _, tau_r = experiment_to_tau(experiment, motor)
    if fast:
        return rk4_integrate(t, u[0], v[0], r[0], tau_u, tau_r, dynamic)
    tau_u_fn = interp1d(t, tau_u, bounds_error=False, fill_value="extrapolate")
    tau_r_fn = interp1d(t, tau_r, bounds_error=False, fill_value="extrapolate")
    m11 = dynamic["m11"]
    m22 = dynamic["m22"]
    m33 = dynamic["m33"]
    Xu_pos = dynamic["Xu_pos"]
    Xu_neg = dynamic["Xu_neg"]
    Xuu_pos = dynamic["Xuu_pos"]
    Xuu_neg = dynamic["Xuu_neg"]
    Yv = dynamic["Yv"]
    Yvv = dynamic["Yvv"]
    Yvr = dynamic["Yvr"]
    Yr = dynamic["Yr"]
    Yrv = dynamic["Yrv"]
    Yrr = dynamic["Yrr"]
    Yur = dynamic["Yur"]
    Nv = dynamic["Nv"]
    Nvv = dynamic["Nvv"]
    Nvr = dynamic["Nvr"]
    Nr = dynamic["Nr"]
    Nrv = dynamic["Nrv"]
    Nrr = dynamic["Nrr"]
    def ode(tt, x):
        u_, v_, r_ = x
        tauu = float(tau_u_fn(tt))
        taur = float(tau_r_fn(tt))
        s_val = 0.5 * (1.0 + np.tanh(20.0 * u_))
        Xu_val = Xu_neg + (Xu_pos - Xu_neg) * s_val
        Xuu_val = Xuu_neg + (Xuu_pos - Xuu_neg) * s_val
        du = (tauu + m22 * v_ * r_ - Xu_val * u_ - Xuu_val * abs(u_) * u_) / m11
        dv = (-m11 * u_ * r_ - Yv * v_ - Yvv * abs(v_) * v_ - Yvr * abs(r_) * v_ - Yr * r_ - Yrv * abs(v_) * r_ - Yrr * abs(r_) * r_ - Yur * u_ * r_) / m22
        dr = (taur - (m22 - m11) * u_ * v_ - Nv * v_ - Nvv * abs(v_) * v_ - Nvr * abs(r_) * v_ - Nr * r_ - Nrv * abs(v_) * r_ - Nrr * abs(r_) * r_) / m33
        return [du, dv, dr]
    try:
        sol = solve_ivp(
            ode,
            [t[0], t[-1]],
            [u[0], v[0], r[0]],
            t_eval=t,
            method="RK45",
            rtol=1e-5,
            atol=1e-5
        )
        if not sol.success or sol.y.shape[1] != len(t):
            raise RuntimeError("solve_ivp failed")
        return sol.y[0], sol.y[1], sol.y[2]
    except Exception:
        return rk4_integrate(t, u[0], v[0], r[0], tau_u, tau_r, dynamic)

def mae(y_meas: np.ndarray, y_sim: np.ndarray) -> float:
    return float(np.mean(np.abs(np.asarray(y_meas) - np.asarray(y_sim))))

def r2(y_meas: np.ndarray, y_sim: np.ndarray) -> float:
    y_meas = np.asarray(y_meas)
    y_sim  = np.asarray(y_sim)
    ss_res = np.sum((y_meas - y_sim) ** 2)
    ss_tot = np.sum((y_meas - np.mean(y_meas)) ** 2)
    if ss_tot == 0.0:
        return 1.0 if ss_res == 0.0 else 0.0
    return float(1.0 - ss_res / ss_tot)

def rms(y_meas: np.ndarray, y_sim: np.ndarray) -> float:
    return float(np.sqrt(np.mean((y_meas - y_sim) ** 2)))

STRINGS = {
    "page_title":   "Asymmetric Model Validation",
    "time_label":   "Time (s)",
    "measured":     "Measured",
    "model":        "Model",
    "error_region": "Error region",
    "ylabels": {
        "u": "Surge velocity\n$u$  [m/s]",
        "v": "Sway velocity\n$v$  [m/s]",
        "r": "Yaw rate\n$r$  [rad/s]",
    },
}

COLOR_MEASURED = "#000000"
COLOR_MODEL    = "#1a56a0"
COLOR_FILL     = "#aec6e8"

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
        "legend.fontsize":     7,
        "legend.handlelength": 2.2,
        "lines.linewidth":     1.2,
    })

def make_page(ds, u_sim, v_sim, r_sim, exp_name: str) -> plt.Figure:
    t = ds["t"]
    signals = [
        ("u", ds["u"], u_sim),
        ("v", ds["v"], v_sim),
        ("r", ds["r"], r_sim),
    ]
    fig = plt.figure(figsize=(6.5, 7.5))
    fig.patch.set_facecolor("white")
    fig.text(
        0.5, 0.995,
        f"{STRINGS['page_title']} — {exp_name}",
        ha="center", va="top",
        fontsize=9, fontweight="bold",
        fontfamily="serif",
    )
    gs = GridSpec(
        3, 1,
        figure=fig,
        left=0.14, right=0.97,
        top=0.955, bottom=0.07,
        hspace=0.38,
    )
    for row, (key, y_meas, y_sim) in enumerate(signals):
        ax = fig.add_subplot(gs[row])
        rms_val = rms(y_meas, y_sim)
        mae_val = mae(y_meas, y_sim)
        r2_val  = r2(y_meas, y_sim)
        ax.fill_between(
            t, y_meas, y_sim,
            color=COLOR_FILL, alpha=0.55,
            linewidth=0, zorder=2,
        )
        ax.plot(t, y_meas, color=COLOR_MEASURED, linewidth=1.2, linestyle="-", label=STRINGS["measured"], zorder=4)
        ax.plot(t, y_sim, color=COLOR_MODEL, linewidth=1.1, linestyle="--", label=STRINGS["model"], zorder=5)
        ax.set_xlim(t[0], t[-1])
        ax.set_ylim(YLIMS[key])
        ax.set_ylabel(STRINGS["ylabels"][key], labelpad=6, multialignment="center")
        ax.yaxis.set_minor_locator(AutoMinorLocator(4))
        ax.xaxis.set_minor_locator(AutoMinorLocator(4))
        ax.tick_params(which="both", top=True, right=True)
        if row < 2:
            ax.set_xticklabels([])
        else:
            ax.set_xlabel(STRINGS["time_label"], labelpad=3)
        err_patch = mpatches.Patch(facecolor=COLOR_FILL, alpha=0.6, edgecolor="none", label=STRINGS["error_region"])
        metrics_handle = mpatches.Patch(visible=False, label=f"RMS={rms_val:.4f}  MAE={mae_val:.4f}  R²={r2_val:.4f}")
        handles, labels = ax.get_legend_handles_labels()
        ax.legend(
            handles + [err_patch, metrics_handle],
            labels  + [STRINGS["error_region"], f"RMS={rms_val:.4f}  MAE={mae_val:.4f}  R²={r2_val:.4f}"],
            loc="upper right",
            fontsize=7,
            handlelength=2.0,
            borderpad=0.5,
            labelspacing=0.3,
            handletextpad=0.4,
        )
    return fig

def build_metrics_entry(exp_name: str, ds: dict, u_sim: np.ndarray, v_sim: np.ndarray, r_sim: np.ndarray) -> dict:
    channels = {
        "u": (ds["u"], u_sim),
        "v": (ds["v"], v_sim),
        "r": (ds["r"], r_sim),
    }
    entry = {"experiment": exp_name}
    rms_vals, mae_vals, r2_vals = [], [], []
    for ch, (y_meas, y_sim) in channels.items():
        rms_v = rms(y_meas, y_sim)
        mae_v = mae(y_meas, y_sim)
        r2_v  = r2(y_meas, y_sim)
        entry[ch] = {
            "RMS": round(rms_v, 6),
            "MAE": round(mae_v, 6),
            "R2":  round(r2_v,  6),
        }
        rms_vals.append(rms_v)
        mae_vals.append(mae_v)
        r2_vals.append(r2_v)
    entry["mean"] = {
        "RMS": round(float(np.mean(rms_vals)), 6),
        "MAE": round(float(np.mean(mae_vals)), 6),
        "R2":  round(float(np.mean(r2_vals)),  6),
    }
    return entry

def save_metrics_json(metrics: list[dict], json_path: str) -> None:
    payload = {
        "experiments": metrics,
        "global_mean": {
            "RMS": round(float(np.mean([e["mean"]["RMS"] for e in metrics])), 6),
            "MAE": round(float(np.mean([e["mean"]["MAE"] for e in metrics])), 6),
            "R2":  round(float(np.mean([e["mean"]["R2"]  for e in metrics])), 6),
        },
    }
    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2, ensure_ascii=False)
    print(f"Metrics saved to: {json_path}")

def parse_args():
    parser = argparse.ArgumentParser(description="Generate PDF Validation Report for Asymmetric Model.")
    parser.add_argument("--data-dir", default=os.path.dirname(os.path.abspath(__file__)))
    parser.add_argument("--dynamic-json", default="identified_dynamics_asymmetric.json")
    parser.add_argument("--output", default=None, help="Output PDF path.")
    parser.add_argument("--metrics-json", default=None, help="Output metrics JSON path.")
    parser.add_argument("--fast", action="store_true", help="Use RK4 solver.")
    return parser.parse_args()

def main():
    args = parse_args()
    dyn_path = os.path.join(args.data_dir, args.dynamic_json)
    if not os.path.exists(dyn_path):
        print(f"[ERROR] Asymmetric dynamics file not found: {dyn_path}")
        sys.exit(1)
    with open(dyn_path, "r", encoding="utf-8") as f:
        dynamic = json.load(f)
    motor = dict(MOTOR_TEMPLATE)
    exp_paths = []
    for i in range(1, 11):
        p = os.path.join(args.data_dir, f"experiment_{i:02d}.mat")
        if os.path.exists(p):
            exp_paths.append(p)
    if not exp_paths:
        print("[ERROR] No experiment .mat files found in:", args.data_dir)
        sys.exit(1)
    print(f"Found {len(exp_paths)} experiments.")
    if args.output:
        pdf_path = args.output
    else:
        pdf_path = os.path.join(args.data_dir, "experiment_asymmetric_validation.pdf")
    if args.metrics_json:
        json_path = args.metrics_json
    else:
        json_path = os.path.splitext(pdf_path)[0] + "_metrics.json"
    apply_article_style()
    all_metrics = []
    with PdfPages(pdf_path) as pdf:
        for exp_path in exp_paths:
            exp_name = os.path.splitext(os.path.basename(exp_path))[0]
            print(f"  Processing {exp_name} ...", end=" ", flush=True)
            try:
                ds = load_experiment(exp_path)
                u_sim, v_sim, r_sim = simulate_asymmetric_experiment(ds, dynamic, motor, fast=args.fast)
                entry = build_metrics_entry(exp_name, ds, u_sim, v_sim, r_sim)
                all_metrics.append(entry)
                fig = make_page(ds, u_sim, v_sim, r_sim, exp_name)
                pdf.savefig(fig, dpi=300, bbox_inches="tight", facecolor="white")
                plt.close(fig)
                print(f"done [u RMS={entry['u']['RMS']:.4f}  v RMS={entry['v']['RMS']:.4f}  r RMS={entry['r']['RMS']:.4f}]")
            except Exception as e:
                print(f"FAILED ({e})")
        d = pdf.infodict()
        d["Title"]   = "Asymmetric Vessel Model Validation Report"
        d["Author"]  = "step4_generate_report.py"
        d["Subject"] = "Vessel system identification - output-error method with asymmetric surge drag"
    print(f"\nReport saved to: {pdf_path}")
    if all_metrics:
        save_metrics_json(all_metrics, json_path)

if __name__ == "__main__":
    main()
