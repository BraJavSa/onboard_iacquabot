#!/usr/bin/env python3
"""
Step 5: Compare Identification Stages.
This script compares the three identification stages (Bounded Least Squares, 
Powell Symmetric, and Powell Asymmetric) by simulating them on all 10 experiments. 
It generates a PDF report comparing the model responses to the measured data 
and displays a bar chart of the RMS error of each stage for every channel.
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

def ode_symmetric(tt, state, Tu_fn, Tr_fn, dynamic):
    u_, v_, r_ = state
    tau_u = float(Tu_fn(tt))
    tau_r = float(Tr_fn(tt))
    m11 = dynamic["m11"]
    m22 = dynamic["m22"]
    m33 = dynamic["m33"]
    Xu = dynamic["Xu"]
    Xuu = dynamic["Xuu"]
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
    du = (tau_u + m22 * v_ * r_ - Xu * u_ - Xuu * abs(u_) * u_) / m11
    dv = (-m11 * u_ * r_ - Yv * v_ - Yvv * abs(v_) * v_ - Yvr * abs(r_) * v_ - Yr * r_ - Yrv * abs(v_) * r_ - Yrr * abs(r_) * r_ - Yur * u_ * r_) / m22
    dr = (tau_r - (m22 - m11) * u_ * v_ - Nv * v_ - Nvv * abs(v_) * v_ - Nvr * abs(r_) * v_ - Nr * r_ - Nrv * abs(v_) * r_ - Nrr * abs(r_) * r_) / m33
    return [du, dv, dr]

def rk4_symmetric(t, u0, v0, r0, Tu, Tr, dynamic):
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
        uk, vk, rk = u_sim[k], v_sim[k], r_sim[k]
        tu1 = Tu[k]
        tr1 = Tr[k]
        du1 = (tu1 + m22 * vk * rk - Xu * uk - Xuu * abs(uk) * uk) / m11
        dv1 = (-m11 * uk * rk - Yv * vk - Yvv * abs(vk) * vk - Yvr * abs(rk) * vk - Yr * rk - Yrv * abs(vk) * rk - Yrr * abs(rk) * rk - Yur * uk * rk) / m22
        dr1 = (tr1 - (m22 - m11) * uk * vk - Nv * vk - Nvv * abs(vk) * vk - Nvr * abs(rk) * vk - Nr * rk - Nrv * abs(vk) * rk - Nrr * abs(rk) * rk) / m33
        
        u2 = uk + 0.5 * dt * du1
        v2 = vk + 0.5 * dt * dv1
        r2 = rk + 0.5 * dt * dr1
        tu2 = 0.5 * (Tu[k] + Tu[k+1])
        tr2 = 0.5 * (Tr[k] + Tr[k+1])
        du2 = (tu2 + m22 * v2 * r2 - Xu * u2 - Xuu * abs(u2) * u2) / m11
        dv2 = (-m11 * u2 * r2 - Yv * v2 - Yvv * abs(v2) * v2 - Yvr * abs(r2) * v2 - Yr * r2 - Yrv * abs(v2) * r2 - Yrr * abs(r2) * r2 - Yur * u2 * r2) / m22
        dr2 = (tr2 - (m22 - m11) * u2 * v2 - Nv * v2 - Nvv * abs(v2) * v2 - Nvr * abs(r2) * v2 - Nr * r2 - Nrv * abs(v2) * r2 - Nrr * abs(r2) * r2) / m33
        
        u3 = uk + 0.5 * dt * du2
        v3 = vk + 0.5 * dt * dv2
        r3 = rk + 0.5 * dt * dr2
        du3 = (tu2 + m22 * v3 * r3 - Xu * u3 - Xuu * abs(u3) * u3) / m11
        dv3 = (-m11 * u3 * r3 - Yv * v3 - Yvv * abs(v3) * v3 - Yvr * abs(r3) * v3 - Yr * r3 - Yrv * abs(v3) * r3 - Yrr * abs(r3) * r3 - Yur * u3 * r3) / m22
        dr3 = (tr2 - (m22 - m11) * u3 * v3 - Nv * v3 - Nvv * abs(v3) * v3 - Nvr * abs(r3) * v3 - Nr * r3 - Nrv * abs(v3) * r3 - Nrr * abs(r3) * r3) / m33
        
        u4 = uk + dt * du3
        v4 = vk + dt * dv3
        r4 = rk + dt * dr3
        tu4 = Tu[k+1]
        tr4 = Tr[k+1]
        du4 = (tu4 + m22 * v4 * r4 - Xu * u4 - Xuu * abs(u4) * u4) / m11
        dv4 = (-m11 * u4 * r4 - Yv * v4 - Yvv * abs(v4) * v4 - Yvr * abs(r4) * v4 - Yr * r4 - Yrv * abs(v4) * r4 - Yrr * abs(r4) * r4 - Yur * u4 * r4) / m22
        dr4 = (tr4 - (m22 - m11) * u4 * v4 - Nv * v4 - Nvv * abs(v4) * v4 - Nvr * abs(r4) * v4 - Nr * r4 - Nrv * abs(v4) * r4 - Nrr * abs(r4) * r4) / m33
        
        u_sim[k+1] = uk + (dt / 6.0) * (du1 + 2.0*du2 + 2.0*du3 + du4)
        v_sim[k+1] = vk + (dt / 6.0) * (dv1 + 2.0*dv2 + 2.0*dv3 + dv4)
        r_sim[k+1] = rk + (dt / 6.0) * (dr1 + 2.0*dr2 + 2.0*dr3 + dr4)
    return u_sim, v_sim, r_sim

def simulate_symmetric(experiment, dynamic, motor, fast=False):
    t = experiment["t"]
    u = experiment["u"]
    v = experiment["v"]
    r = experiment["r"]
    Tu, _, Tr = experiment_to_tau(experiment, motor)
    if fast:
        return rk4_symmetric(t, u[0], v[0], r[0], Tu, Tr, dynamic)
    Tu_fn = interp1d(t, Tu, bounds_error=False, fill_value="extrapolate")
    Tr_fn = interp1d(t, Tr, bounds_error=False, fill_value="extrapolate")
    try:
        sol = solve_ivp(
            ode_symmetric,
            [t[0], t[-1]],
            [u[0], v[0], r[0]],
            args=(Tu_fn, Tr_fn, dynamic),
            t_eval=t,
            method="RK45",
            rtol=1e-5,
            atol=1e-5
        )
        if not sol.success or sol.y.shape[1] != len(t):
            raise RuntimeError("solve_ivp failed")
        return sol.y[0], sol.y[1], sol.y[2]
    except Exception:
        return rk4_symmetric(t, u[0], v[0], r[0], Tu, Tr, dynamic)

def ode_asymmetric(tt, state, Tu_fn, Tr_fn, dynamic):
    u_, v_, r_ = state
    tau_u = float(Tu_fn(tt))
    tau_r = float(Tr_fn(tt))
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
    s_val = 0.5 * (1.0 + np.tanh(20.0 * u_))
    Xu_val = Xu_neg + (Xu_pos - Xu_neg) * s_val
    Xuu_val = Xuu_neg + (Xuu_pos - Xuu_neg) * s_val
    du = (tau_u + m22 * v_ * r_ - Xu_val * u_ - Xuu_val * abs(u_) * u_) / m11
    dv = (-m11 * u_ * r_ - Yv * v_ - Yvv * abs(v_) * v_ - Yvr * abs(r_) * v_ - Yr * r_ - Yrv * abs(v_) * r_ - Yrr * abs(r_) * r_ - Yur * u_ * r_) / m22
    dr = (tau_r - (m22 - m11) * u_ * v_ - Nv * v_ - Nvv * abs(v_) * v_ - Nvr * abs(r_) * v_ - Nr * r_ - Nrv * abs(v_) * r_ - Nrr * abs(r_) * r_) / m33
    return [du, dv, dr]

def rk4_asymmetric(t, u0, v0, r0, Tu, Tr, dynamic):
    N = len(t)
    u_sim = np.zeros(N)
    v_sim = np.zeros(N)
    r_sim = np.zeros(N)
    u_sim[0], v_sim[0], r_sim[0] = u0, v0, r0
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
        uk, vk, rk = u_sim[k], v_sim[k], r_sim[k]
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

def simulate_asymmetric(experiment, dynamic, motor, fast=False):
    t = experiment["t"]
    u = experiment["u"]
    v = experiment["v"]
    r = experiment["r"]
    Tu, _, Tr = experiment_to_tau(experiment, motor)
    if fast:
        return rk4_asymmetric(t, u[0], v[0], r[0], Tu, Tr, dynamic)
    Tu_fn = interp1d(t, Tu, bounds_error=False, fill_value="extrapolate")
    Tr_fn = interp1d(t, Tr, bounds_error=False, fill_value="extrapolate")
    try:
        sol = solve_ivp(
            ode_asymmetric,
            [t[0], t[-1]],
            [u[0], v[0], r[0]],
            args=(Tu_fn, Tr_fn, dynamic),
            t_eval=t,
            method="RK45",
            rtol=1e-5,
            atol=1e-5
        )
        if not sol.success or sol.y.shape[1] != len(t):
            raise RuntimeError("solve_ivp failed")
        return sol.y[0], sol.y[1], sol.y[2]
    except Exception:
        return rk4_asymmetric(t, u[0], v[0], r[0], Tu, Tr, dynamic)

def rms(y_meas: np.ndarray, y_sim: np.ndarray) -> float:
    return float(np.sqrt(np.mean((y_meas - y_sim) ** 2)))

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

def make_page(ds, ls_sim, powell_sim, asym_sim, exp_name: str) -> plt.Figure:
    t = ds["t"]
    fig = plt.figure(figsize=(8.5, 7.5))
    fig.patch.set_facecolor("white")
    fig.text(
        0.5, 0.99,
        f"Stage Identification Comparison — {exp_name}",
        ha="center", va="top",
        fontsize=10, fontweight="bold",
        fontfamily="serif",
    )
    gs = GridSpec(
        3, 2,
        figure=fig,
        left=0.10, right=0.96,
        top=0.94, bottom=0.08,
        hspace=0.36, wspace=0.25,
        width_ratios=[1.7, 1.0]
    )
    
    signals = [
        ("u", ds["u"], ls_sim[0], powell_sim[0], asym_sim[0], "Surge velocity\n$u$  [m/s]", (-1.0, 1.6)),
        ("v", ds["v"], ls_sim[1], powell_sim[1], asym_sim[1], "Sway velocity\n$v$  [m/s]", (-1.0, 1.0)),
        ("r", ds["r"], ls_sim[2], powell_sim[2], asym_sim[2], "Yaw rate\n$r$  [rad/s]", (-1.0, 1.0)),
    ]

    for row, (key, y_meas, y_ls, y_powell, y_asym, ylabel, ylim) in enumerate(signals):
        ax_plot = fig.add_subplot(gs[row, 0])
        ax_plot.plot(t, y_meas, color="#000000", linewidth=1.2, linestyle="-", label="Measured", zorder=4)
        ax_plot.plot(t, y_ls, color="#4b7bec", linewidth=1.1, linestyle="--", label="Stage 1: Least Squares", zorder=5)
        ax_plot.plot(t, y_powell, color="#26de81", linewidth=1.1, linestyle="-.", label="Stage 2: Powell Symmetric", zorder=6)
        ax_plot.plot(t, y_asym, color="#eb3b5a", linewidth=1.1, linestyle=":", label="Stage 3: Powell Asymmetric", zorder=7)
        ax_plot.set_xlim(t[0], t[-1])
        ax_plot.set_ylim(ylim)
        ax_plot.set_ylabel(ylabel, labelpad=6, multialignment="center")
        ax_plot.yaxis.set_minor_locator(AutoMinorLocator(4))
        ax_plot.xaxis.set_minor_locator(AutoMinorLocator(4))
        ax_plot.tick_params(which="both", top=True, right=True)
        if row < 2:
            ax_plot.set_xticklabels([])
        else:
            ax_plot.set_xlabel("Time (s)", labelpad=3)
        if row == 0:
            ax_plot.legend(loc="upper right", fontsize=6, framealpha=0.95)

        ax_bar = fig.add_subplot(gs[row, 1])
        rms_ls = rms(y_meas, y_ls)
        rms_powell = rms(y_meas, y_powell)
        rms_asym = rms(y_meas, y_asym)
        
        stages = ["Step 1", "Step 2", "Step 3"]
        errors = [rms_ls, rms_powell, rms_asym]
        colors = ["#4b7bec", "#26de81", "#eb3b5a"]
        
        bars = ax_bar.bar(stages, errors, color=colors, edgecolor="black", linewidth=0.6, width=0.5)
        ax_bar.set_title(f"{key.upper()} RMS Error", fontsize=8)
        ax_bar.set_ylabel("RMS Error", fontsize=7)
        ax_bar.tick_params(labelsize=7)
        ax_bar.grid(True, linestyle="--", linewidth=0.4, alpha=0.7)
        
        max_err = max(errors)
        if max_err > 0.0:
            ax_bar.set_ylim(0, max_err * 1.3)
        
        for bar in bars:
            height = bar.get_height()
            ax_bar.text(
                bar.get_x() + bar.get_width() / 2.0,
                height + 0.02 * (max_err if max_err > 0 else 1.0),
                f"{height:.4f}",
                ha="center", va="bottom", fontsize=6, fontweight="bold"
            )

    return fig

def main():
    parser = argparse.ArgumentParser(description="Compare identification stages.")
    parser.add_argument("--data-dir", default=os.path.dirname(os.path.abspath(__file__)))
    parser.add_argument("--output", default="stage_comparison_report.pdf", help="Output PDF report path.")
    parser.add_argument("--fast", action="store_true", help="Use fast RK4 integration.")
    args = parser.parse_args()

    dyn_ls_path = os.path.join(args.data_dir, "identified_dynamics.json")
    dyn_powell_path = os.path.join(args.data_dir, "identified_dynamics_powell.json")
    dyn_asym_path = os.path.join(args.data_dir, "identified_dynamics_asymmetric.json")

    for p in [dyn_ls_path, dyn_powell_path, dyn_asym_path]:
        if not os.path.exists(p):
            print(f"[ERROR] Required parameters file not found: {p}")
            sys.exit(1)

    with open(dyn_ls_path, "r") as f:
        dyn_ls = json.load(f)
    with open(dyn_powell_path, "r") as f:
        dyn_powell = json.load(f)
    with open(dyn_asym_path, "r") as f:
        dyn_asym = json.load(f)

    motor = dict(MOTOR_TEMPLATE)
    exp_paths = []
    for i in range(1, 11):
        p = os.path.join(args.data_dir, f"experiment_{i:02d}.mat")
        if os.path.exists(p):
            exp_paths.append(p)
    if not exp_paths:
        print("[ERROR] No experiment .mat files found in:", args.data_dir)
        sys.exit(1)
    
    print(f"Comparing stages on {len(exp_paths)} experiments...")
    apply_article_style()
    pdf_path = os.path.join(args.data_dir, args.output)

    with PdfPages(pdf_path) as pdf:
        for exp_path in exp_paths:
            exp_name = os.path.splitext(os.path.basename(exp_path))[0]
            print(f"  Processing {exp_name} ...", end=" ", flush=True)
            try:
                ds = load_experiment(exp_path)
                ls_sim = simulate_symmetric(ds, dyn_ls, motor, fast=args.fast)
                powell_sim = simulate_symmetric(ds, dyn_powell, motor, fast=args.fast)
                asym_sim = simulate_asymmetric(ds, dyn_asym, motor, fast=args.fast)
                
                fig = make_page(ds, ls_sim, powell_sim, asym_sim, exp_name)
                pdf.savefig(fig, dpi=300, bbox_inches="tight", facecolor="white")
                plt.close(fig)
                print("done")
            except Exception as e:
                print(f"FAILED ({e})")
                
        d = pdf.infodict()
        d["Title"]   = "USV Model Identification Stage Comparison Report"
        d["Author"]  = "step5_compare_stages.py"
        d["Subject"] = "Comparison of Least Squares, Powell Symmetric, and Powell Asymmetric Models"

    print(f"\nStage comparison report successfully saved to: {pdf_path}")

if __name__ == "__main__":
    main()
