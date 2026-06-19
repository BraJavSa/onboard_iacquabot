#!/usr/bin/env python3
"""
Step 1: Bounded Least Squares Identification (9-parameter model).
Estimates the 9 hydrodynamic parameters of the diagonal Fossen 3-DOF model for the iacquabot USV.
"""
import os
import json
import numpy as np
from scipy.io import loadmat
from scipy.signal import savgol_filter
from scipy.optimize import lsq_linear

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
        [-0.3, 0.40],
        [0.3, 0.40],
        [-0.3, -0.50],
        [0.3, -0.50],
    ],
    "angles_deg": [0.0, 0.0, 0.0, 0.0],
}

y_coords = [
    MOTOR_TEMPLATE["positions_yx"][3][0],
    MOTOR_TEMPLATE["positions_yx"][1][0],
    MOTOR_TEMPLATE["positions_yx"][2][0],
    MOTOR_TEMPLATE["positions_yx"][0][0],
]

PARAM_NAMES = ['m11', 'm22', 'm33', 'Xu', 'Xuu', 'Yv', 'Yvv', 'Nr', 'Nrr']

def pwm_to_thrust_forces(pwm1, pwm2, pwm3, pwm4):
    pwms = [pwm1, pwm2, pwm3, pwm4]
    T = np.zeros(4)
    params = MOTOR_TEMPLATE["T200"]
    for i in range(4):
        s_i = -1.0 if MOTOR_TEMPLATE["motor_inverted"][i] else 1.0
        cmd = s_i * (pwms[i] - MOTOR_TEMPLATE["pwm_mid"]) / (MOTOR_TEMPLATE["pwm_max"] - MOTOR_TEMPLATE["pwm_mid"])
        if abs(cmd) <= 0.01:
            T[i] = 0.0
        elif cmd > 0.01:
            p = params["pos"]
            T[i] = p["A"] + (p["K"] - p["A"]) / ((p["C"] + np.exp(-p["B"] * (cmd - p["M"])))**(1.0 / p["v"]))
        else:
            p = params["neg"]
            T[i] = p["A"] + (p["K"] - p["A"]) / ((p["C"] + np.exp(-p["B"] * (cmd - p["M"])))**(1.0 / p["v"]))
        T[i] = np.clip(T[i], MOTOR_TEMPLATE["max_rev"], MOTOR_TEMPLATE["max_fwd"])
    Tu = np.sum(T)
    Tr = np.sum([y * t for y, t in zip(y_coords, T)])
    return Tu, Tr

def build_global_phi_tau(acc_u, acc_v, acc_r, u, v, r, Tu, Tr):
    N = len(u)
    Phi_list = []
    Tau_list = []
    for k in range(N):
        row_u = np.zeros(9)
        row_u[0] = acc_u[k]       # m11
        row_u[1] = -v[k] * r[k]   # m22
        row_u[3] = u[k]           # Xu
        row_u[4] = abs(u[k]) * u[k] # Xuu
        Phi_list.append(row_u)
        Tau_list.append(Tu[k])

        row_v = np.zeros(9)
        row_v[0] = u[k] * r[k]    # m11
        row_v[1] = acc_v[k]       # m22
        row_v[5] = v[k]           # Yv
        row_v[6] = abs(v[k]) * v[k] # Yvv
        Phi_list.append(row_v)
        Tau_list.append(0.0)

        row_r = np.zeros(9)
        row_r[0] = -u[k] * v[k]   # m11
        row_r[1] = u[k] * v[k]    # m22
        row_r[2] = acc_r[k]       # m33
        row_r[7] = r[k]           # Nr
        row_r[8] = abs(r[k]) * r[k] # Nrr
        Phi_list.append(row_r)
        Tau_list.append(Tr[k])
    return np.array(Phi_list), np.array(Tau_list)

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
        dv3 = (-m11 * u3 * r3 - Yv * v3 - Yvv * abs(v3) * v3) / m22
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

def compute_metrics(y_real, y_sim):
    rms_val = float(np.sqrt(np.mean((y_real - y_sim)**2)))
    mae_val = float(np.mean(np.abs(y_real - y_sim)))
    return rms_val, mae_val

def main():
    FS = 30.0
    dt = 1.0 / FS
    valid_mats = sorted([f for f in os.listdir(".") if f.startswith("experiment_") and f.endswith(".mat")])
    print(f"Loaded .mat files for joint regression: {valid_mats}")
    if not valid_mats:
        print("No experiment .mat files found.")
        return
    Phi_global = []
    Tau_global = []
    experiments_data = []
    for mat_file in valid_mats:
        try:
            mat_data = loadmat(mat_file)
            u_raw = mat_data["vx"].flatten()
            v_raw = mat_data["vy"].flatten()
            r_raw = mat_data["wz"].flatten()
            u = savgol_filter(u_raw, 25, 2)
            v = savgol_filter(v_raw, 25, 2)
            r = savgol_filter(r_raw, 25, 2)
            acc_u = np.gradient(u, dt)
            acc_v = np.gradient(v, dt)
            acc_r = np.gradient(r, dt)
            pwm1 = mat_data["pwm1"].flatten()
            pwm2 = mat_data["pwm2"].flatten()
            pwm3 = mat_data["pwm3"].flatten()
            pwm4 = mat_data["pwm4"].flatten()
            N_samples = len(u)
            Tu_arr = np.zeros(N_samples)
            Tr_arr = np.zeros(N_samples)
            for i in range(N_samples):
                Tu_arr[i], Tr_arr[i] = pwm_to_thrust_forces(pwm1[i], pwm2[i], pwm3[i], pwm4[i])
            Phi_exp, Tau_exp = build_global_phi_tau(acc_u, acc_v, acc_r, u, v, r, Tu_arr, Tr_arr)
            if mat_file not in ["experiment_01.mat", "experiment_02.mat"]:
                Phi_global.append(Phi_exp)
                Tau_global.append(Tau_exp)
            t_arr = mat_data["t"].flatten() if "t" in mat_data else np.arange(N_samples) * dt
            experiments_data.append({
                "name": mat_file,
                "t": t_arr,
                "u_raw": u_raw,
                "v_raw": v_raw,
                "r_raw": r_raw,
                "u_filt": u,
                "v_filt": v,
                "r_filt": r,
                "Tu": Tu_arr,
                "Tr": Tr_arr
            })
        except Exception as e:
            print(f"Error processing file {mat_file}: {e}")
            continue
    Phi_global = np.vstack(Phi_global)
    Tau_global = np.concatenate(Tau_global)
    print(f"\nGlobal Regression Matrix Phi shape (excluding exp_01 and exp_02): {Phi_global.shape}")
    print("--- Running Ordinary Least Squares (OLS) ---")
    theta_ols, _, _, _ = np.linalg.lstsq(Phi_global, Tau_global, rcond=None)
    print("\n================ IDENTIFIED PARAMETERS (OLS UNCONSTRAINED 9-PARAM) ================")
    for i, name in enumerate(PARAM_NAMES):
        print(f" {name:<8}: {theta_ols[i]:.6f}")
    print("===================================================================================")
    dynamic_ols = {name: float(theta_ols[i]) for i, name in enumerate(PARAM_NAMES)}
    dynamic_ols["model"] = "Fossen 3DOF 9-parameter OLS (Unconstrained)"
    
    print("\n--- Running Bounded Least Squares (lsq_linear) ---")
    lb = [20.0, 30.0, 5.0, 1.0, 1.0, 50.0, 5.0, 1.0, 1.0]
    ub = [150.0, 150.0, 50.0, 150.0, 150.0, 300.0, 150.0, 150.0, 150.0]
    res = lsq_linear(Phi_global, Tau_global, bounds=(lb, ub))
    theta = res.x
    print("\n================ FINAL IDENTIFIED PARAMETERS (BOUNDED LSTSQ 9-PARAM) ================")
    for i, name in enumerate(PARAM_NAMES):
         print(f" {name:<8}: {theta[i]:.6f}")
    print("=====================================================================================")
    dynamic_bounded = {name: float(theta[i]) for i, name in enumerate(PARAM_NAMES)}
    dynamic_bounded["model"] = "Fossen 3DOF 9-parameter (Bounded Least Squares)"
    with open("identified_dynamics_step1.json", "w") as f:
        json.dump(dynamic_bounded, f, indent=4)
    print("Bounded LSTSQ parameters saved to 'identified_dynamics_step1.json'!")
    
    print("\n--- EVALUATING VALIDATION METRICS ---")
    print(f"{'Experiment':<18} | {'Surge (u)':<17} | {'Sway (v)':<17} | {'Yaw (r)':<17}")
    print(f"{'':<18} | {'RMS':<7} {'MAE':<8} | {'RMS':<7} {'MAE':<8} | {'RMS':<7} {'MAE':<8}")
    print("-" * 78)
    metrics_dict = {}
    for idx, exp in enumerate(experiments_data):
        try:
            t_exp = exp["t"]
            u_real = exp["u_raw"]
            v_real = exp["v_raw"]
            r_real = exp["r_raw"]
            theta_dict = {name: float(theta[i]) for i, name in enumerate(PARAM_NAMES)}
            u_sim, v_sim, r_sim = rk4_integrate(t_exp, u_real[0], v_real[0], r_real[0], exp["Tu"], exp["Tr"], theta_dict)
            rms_u, mae_u = compute_metrics(u_real, u_sim)
            rms_v, mae_v = compute_metrics(v_real, v_sim)
            rms_r, mae_r = compute_metrics(r_real, r_sim)
            print(f"{exp['name']:<18} | {rms_u:.4f}  {mae_u:>7.4f} | {rms_v:.4f}  {mae_v:>7.4f} | {rms_r:.4f}  {mae_r:>7.4f}")
            metrics_dict[exp["name"]] = {
                "u": {"rms": rms_u, "mae": mae_u},
                "v": {"rms": rms_v, "mae": mae_v},
                "r": {"rms": rms_r, "mae": mae_r}
            }
        except Exception as e:
            print(f"Error in simulation of {exp['name']}: {e}")
            continue
    with open("identified_dynamics_metrics_step1.json", "w") as f:
        json.dump(metrics_dict, f, indent=4)

if __name__ == "__main__":
    main()
