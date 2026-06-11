import json
import os

import numpy as np
from scipy.io import loadmat
from scipy.integrate import solve_ivp
from scipy.interpolate import interp1d


def read_json(path):
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def write_json(path, data):
    with open(path, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2)


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


def simulate_experiment(experiment, dynamic, motor, fast=False):
    t = experiment["t"]
    u = experiment["u"]
    v = experiment["v"]
    r = experiment["r"]
    tau_u, _, tau_r = experiment_to_tau(experiment, motor)
    tau_u_fn = interp1d(t, tau_u, bounds_error=False, fill_value="extrapolate")
    tau_r_fn = interp1d(t, tau_r, bounds_error=False, fill_value="extrapolate")

    def ode(tt, x):
        u_, v_, r_ = x
        tauu = float(tau_u_fn(tt))
        taur = float(tau_r_fn(tt))
        du = (
            tauu
            + dynamic["m22"] * v_ * r_
            - dynamic["Xu"] * u_
            - dynamic["Xuu"] * abs(u_) * u_
        ) / dynamic["m11"]
        dv = (
            -dynamic["m11"] * u_ * r_
            - dynamic["Yv"] * v_
            - dynamic["Yvv"] * abs(v_) * v_
            - dynamic.get("Yr", 0.0) * r_
            - dynamic.get("Yrr", 0.0) * abs(r_) * r_
            - dynamic.get("Yur", 0.0) * u_ * r_
        ) / dynamic["m22"]
        dr = (
            taur
            + (dynamic["m11"] - dynamic["m22"]) * u_ * v_
            - dynamic["Nr"] * r_
            - dynamic["Nrr"] * abs(r_) * r_
        ) / dynamic["m33"]
        return [du, dv, dr]

    method = "RK23" if fast else "RK45"
    tol = 1e-3 if fast else 1e-5
    sol = solve_ivp(
        ode,
        [t[0], t[-1]],
        [u[0], v[0], r[0]],
        t_eval=t,
        method=method,
        rtol=tol,
        atol=tol,
    )
    if not sol.success or sol.y.shape[1] != len(t):
        raise RuntimeError(f"Model integration failed for {experiment['name']}")
    return sol.y


def r2(y, y_hat):
    return 1.0 - np.sum((y - y_hat) ** 2) / (np.sum((y - np.mean(y)) ** 2) + 1e-8)


def rms(y, y_hat):
    return float(np.sqrt(np.mean((y - y_hat) ** 2)))


def default_json_paths(base_dir):
    return (
        os.path.join(base_dir, "identified_dynamics.json"),
        os.path.join(base_dir, "identified_motors.json"),
    )
