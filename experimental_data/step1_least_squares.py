#!/usr/bin/env python3
"""
Step 1: Bounded Least Squares Identification.
This script estimates the initial 18 hydrodynamic parameters of the Fossen 3-DOF model 
for the iacquabot USV using joint linear regression (ordinary and bounded least squares) 
across multiple experimental datasets.
"""
import os
import json
import numpy as np
from scipy.io import loadmat
from scipy.signal import savgol_filter
from scipy.integrate import solve_ivp
from scipy.interpolate import interp1d
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
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
        [-0.35, 0.40],
        [0.35, 0.40],
        [-0.35, -0.50],
        [0.35, -0.50],
    ],
    "angles_deg": [0.0, 0.0, 0.0, 0.0],
}

y_coords = [
    MOTOR_TEMPLATE["positions_yx"][3][0],
    MOTOR_TEMPLATE["positions_yx"][1][0],
    MOTOR_TEMPLATE["positions_yx"][2][0],
    MOTOR_TEMPLATE["positions_yx"][0][0],
]

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
        row_u = np.zeros(18)
        row_u[0] = acc_u[k]
        row_u[1] = -v[k] * r[k]
        row_u[3] = u[k]
        row_u[4] = abs(u[k]) * u[k]
        Phi_list.append(row_u)
        Tau_list.append(Tu[k])

        row_v = np.zeros(18)
        row_v[0] = u[k] * r[k]
        row_v[1] = acc_v[k]
        row_v[5] = v[k]
        row_v[6] = abs(v[k]) * v[k]
        row_v[7] = abs(r[k]) * v[k]
        row_v[8] = r[k]
        row_v[9] = abs(v[k]) * r[k]
        row_v[10] = abs(r[k]) * r[k]
        row_v[11] = u[k] * r[k]
        Phi_list.append(row_v)
        Tau_list.append(0.0)

        row_r = np.zeros(18)
        row_r[0] = -u[k] * v[k]
        row_r[1] = u[k] * v[k]
        row_r[2] = acc_r[k]
        row_r[12] = v[k]
        row_r[13] = abs(v[k]) * v[k]
        row_r[14] = abs(r[k]) * v[k]
        row_r[15] = r[k]
        row_r[16] = abs(v[k]) * r[k]
        row_r[17] = abs(r[k]) * r[k]
        Phi_list.append(row_r)
        Tau_list.append(Tr[k])
    return np.array(Phi_list), np.array(Tau_list)

def simulate_vessel(t, u, v, r, Tu, Tr, theta):
    m11, m22, m33, Xu, Xuu, Yv, Yvv, Yvr, Yr, Yrv, Yrr, Yur, Nv, Nvv, Nvr, Nr, Nrv, Nrr = theta
    Tu_fn = interp1d(t, Tu, bounds_error=False, fill_value="extrapolate")
    Tr_fn = interp1d(t, Tr, bounds_error=False, fill_value="extrapolate")
    def ode_system(tt, state):
        u_, v_, r_ = state
        tau_u = float(Tu_fn(tt))
        tau_r = float(Tr_fn(tt))
        du = (tau_u + m22 * v_ * r_ - Xu * u_ - Xuu * abs(u_) * u_) / m11
        dv = (-m11 * u_ * r_ - Yv * v_ - Yvv * abs(v_) * v_ - Yvr * abs(r_) * v_ - Yr * r_ - Yrv * abs(v_) * r_ - Yrr * abs(r_) * r_ - Yur * u_ * r_) / m22
        dr = (tau_r - (m22 - m11) * u_ * v_ - Nv * v_ - Nvv * abs(v_) * v_ - Nvr * abs(r_) * v_ - Nr * r_ - Nrv * abs(v_) * r_ - Nrr * abs(r_) * r_) / m33
        return [du, dv, dr]
    sol = solve_ivp(
        ode_system,
        [t[0], t[-1]],
        [u[0], v[0], r[0]],
        t_eval=t,
        method="RK45",
        rtol=1e-5,
        atol=1e-5
    )
    return sol.y[0], sol.y[1], sol.y[2]

def compute_metrics(y_real, y_sim):
    rms_val = float(np.sqrt(np.mean((y_real - y_sim)**2)))
    r2_val = float(1.0 - np.sum((y_real - y_sim)**2) / (np.sum((y_real - np.mean(y_real))**2) + 1e-8))
    return rms_val, r2_val

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
    print("\n--- ANALYZING NATIVE DYNAMIC RANGE (FLU) ---")
    print(f"{'Experiment':<18} | {'Max |u|':<10} | {'Max |v|':<10} | {'Max |r|':<10} | {'Max |Tr|':<10}")
    print("-" * 65)
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
            print(f"{mat_file:<18} | {np.max(np.abs(u)):.4f}     | {np.max(np.abs(v)):.4f}     | {np.max(np.abs(r)):.4f}     | {np.max(np.abs(Tr_arr)):.4f}")
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
    param_names = [
        'm11', 'm22', 'm33',
        'Xu', 'Xuu',
        'Yv', 'Yvv', 'Yvr', 'Yr', 'Yrv', 'Yrr', 'Yur',
        'Nv', 'Nvv', 'Nvr', 'Nr', 'Nrv', 'Nrr'
    ]
    print("\n================ IDENTIFIED PARAMETERS (OLS UNCONSTRAINED) ================")
    for i, name in enumerate(param_names):
        print(f" {name:<8}: {theta_ols[i]:.6f}")
    print("==============================================================================")
    dynamic_ols = {name: float(theta_ols[i]) for i, name in enumerate(param_names)}
    dynamic_ols["model"] = "Fossen 3DOF 18-parameter OLS (Unconstrained)"
    with open("identified_dynamics_ols.json", "w") as f:
        json.dump(dynamic_ols, f, indent=4)
    print("OLS parameters saved to 'identified_dynamics_ols.json'!")
    print("\n--- Running Bounded Least Squares (lsq_linear) ---")
    lb = [20.0, 30.0, 5.0, 1.0, 0.0, 50.0, 0.0, -150.0, -150.0, -150.0, -150.0, -500.0, -150.0, -150.0, -150.0, 1.0, -150.0, 0.0]
    ub = [150.0, 150.0, 50.0, 150.0, 150.0, 300.0, 150.0, 150.0, 150.0, 150.0, 150.0, 500.0, 150.0, 150.0, 150.0, 150.0, 150.0, 150.0]
    res = lsq_linear(Phi_global, Tau_global, bounds=(lb, ub))
    theta = res.x
    print("\n================ FINAL IDENTIFIED PARAMETERS (BOUNDED LSTSQ) ================")
    for i, name in enumerate(param_names):
        print(f" {name:<8}: {theta[i]:.6f}")
    print("==================================================================================")
    output_py_file = "iacquabot_identified_params.py"
    with open(output_py_file, "w") as f:
        f.write("# Globally identified Fossen parameters with stability, velocity and coupling bounds in sway\n")
        f.write("import numpy as np\n")
        for i, name in enumerate(param_names):
            clean_name = name.replace("-", "")
            f.write(f"{clean_name} = {theta[i]:.10f};\n")
        f.write(f"\ntheta = {list(theta)}\n")
    print(f"\nParameters successfully saved to '{output_py_file}'!")
    dynamic_bounded = {name: float(theta[i]) for i, name in enumerate(param_names)}
    dynamic_bounded["model"] = "Fossen 3DOF 18-parameter (Bounded Least Squares)"
    with open("identified_dynamics.json", "w") as f:
        json.dump(dynamic_bounded, f, indent=4)
    print("Bounded LSTSQ parameters saved to 'identified_dynamics.json'!")
    with open("identified_motors.json", "w") as f:
        json.dump(MOTOR_TEMPLATE, f, indent=4)
    print("Motor parameters saved to 'identified_motors.json'!")
    print("\n--- RUNNING SIMULATION AND ERROR METRICS EVALUATION (BOUNDED LSTSQ) ---")
    print(f"{'Experiment':<18} | {'Surge (u)':<17} | {'Sway (v)':<17} | {'Yaw (r)':<17}")
    print(f"{'':<18} | {'RMS':<7} {'R2':<8} | {'RMS':<7} {'R2':<8} | {'RMS':<7} {'R2':<8}")
    print("-" * 78)
    num_exps = len(experiments_data)
    fig, axes = plt.subplots(num_exps, 3, figsize=(15, 2.5 * num_exps), squeeze=False)
    metrics_dict = {}
    for idx, exp in enumerate(experiments_data):
        try:
            t_exp = exp["t"]
            u_real = exp["u_raw"]
            v_real = exp["v_raw"]
            r_real = exp["r_raw"]
            u_sim, v_sim, r_sim = simulate_vessel(t_exp, u_real, v_real, r_real, exp["Tu"], exp["Tr"], theta)
            rms_u, r2_u = compute_metrics(u_real, u_sim)
            rms_v, r2_v = compute_metrics(v_real, v_sim)
            rms_r, r2_r = compute_metrics(r_real, r_sim)
            print(f"{exp['name']:<18} | {rms_u:.4f}  {r2_u:>7.3f} | {rms_v:.4f}  {r2_v:>7.3f} | {rms_r:.4f}  {r2_r:>7.3f}")
            metrics_dict[exp["name"]] = {
                "u": {"rms": rms_u, "r2": r2_u},
                "v": {"rms": rms_v, "r2": r2_v},
                "r": {"rms": rms_r, "r2": r2_r}
            }
            axes[idx, 0].plot(t_exp, u_real, 'k-', alpha=0.5, label='Measured' if idx == 0 else "")
            axes[idx, 0].plot(t_exp, u_sim, 'b--', label='Model' if idx == 0 else "")
            if idx == 0:
                axes[idx, 0].legend(loc='upper right')
            axes[idx, 0].set_ylabel(f"{exp['name']}\nu [m/s]")
            axes[idx, 0].grid(True)
            axes[idx, 0].set_title(f"u (RMS: {rms_u:.3f}, R2: {r2_u:.2f})", fontsize=9)
            axes[idx, 1].plot(t_exp, v_real, 'k-', alpha=0.5, label='Measured' if idx == 0 else "")
            axes[idx, 1].plot(t_exp, v_sim, 'g--', label='Model' if idx == 0 else "")
            if idx == 0:
                axes[idx, 1].legend(loc='upper right')
            axes[idx, 1].grid(True)
            axes[idx, 1].set_title(f"v (RMS: {rms_v:.3f}, R2: {r2_v:.2f})", fontsize=9)
            axes[idx, 2].plot(t_exp, r_real, 'k-', alpha=0.5, label='Measured' if idx == 0 else "")
            axes[idx, 2].plot(t_exp, r_sim, 'r--', label='Model' if idx == 0 else "")
            if idx == 0:
                axes[idx, 2].legend(loc='upper right')
            axes[idx, 2].grid(True)
            axes[idx, 2].set_title(f"r (RMS: {rms_r:.3f}, R2: {r2_r:.2f})", fontsize=9)
            if idx == num_exps - 1:
                axes[idx, 0].set_xlabel('Time [s]')
                axes[idx, 1].set_xlabel('Time [s]')
                axes[idx, 2].set_xlabel('Time [s]')
        except Exception as e:
            print(f"Error in simulation of {exp['name']}: {e}")
            continue
    plt.tight_layout()
    unified_plot_path = "validation_all_experiments.png"
    plt.savefig(unified_plot_path, dpi=150)
    plt.close(fig)
    print(f"\nUnified validation plot saved to '{unified_plot_path}'!")
    with open("identified_dynamics_metrics.json", "w") as f:
        json.dump(metrics_dict, f, indent=4)
    print("Metrics successfully saved to 'identified_dynamics_metrics.json'!")

if __name__ == "__main__":
    main()
