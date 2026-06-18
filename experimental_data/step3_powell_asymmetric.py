#!/usr/bin/env python3
"""
Step 3: Powell Asymmetric Identification.
This script performs Powell optimization to identify asymmetric surge damping parameters 
(Xu_pos, Xu_neg, Xuu_pos, Xuu_neg) to model the USV's behavior differently in forward 
and reverse directions, using a smooth tanh transition.
"""
import os
import json
import argparse
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.io import loadmat
from scipy.optimize import minimize
from scipy.integrate import solve_ivp
from scipy.interpolate import interp1d
import multiprocessing

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

DYNAMIC_START = {
    "m11": 135.731576,
    "m22": 75.337575,
    "m33": 23.713832,
    "Xu": 3.791877,
    "Xuu": 98.192691,
    "Yv": 50.000000,
    "Yvv": 0.000000,
    "Yvr": -81.788472,
    "Yr": -22.523586,
    "Yrv": 37.755515,
    "Yrr": 10.338075,
    "Yur": -89.394162,
    "Nv": 10.241031,
    "Nvv": 49.001575,
    "Nvr": -25.774476,
    "Nr": 34.324285,
    "Nrv": 12.738439,
    "Nrr": 15.439135,
}

global_train_datasets = []
global_pool = None
opt_history = {}

def dynamic_from_vector(p):
    dyn = dict(DYNAMIC_START)
    dyn["Xu_pos"] = float(p[0])
    dyn["Xu_neg"] = float(p[1])
    dyn["Xuu_pos"] = float(p[2])
    dyn["Xuu_neg"] = float(p[3])
    dyn["model"] = "Fossen 3DOF asymmetric output-error (Fixed m11)"
    return dyn

def experiment_to_tau_powell(experiment, motor):
    pwm1 = experiment["pwm_raw"][0]
    pwm2 = experiment["pwm_raw"][1]
    pwm3 = experiment["pwm_raw"][2]
    pwm4 = experiment["pwm_raw"][3]
    N = len(pwm1)
    pwms = [pwm1, pwm2, pwm3, pwm4]
    T = np.zeros((4, N))
    params = motor["T200"]
    for i in range(4):
        s_i = -1.0 if motor["motor_inverted"][i] else 1.0
        cmd = s_i * (pwms[i] - motor["pwm_mid"]) / (motor["pwm_max"] - motor["pwm_mid"])
        mask_zero = np.abs(cmd) <= 0.01
        mask_pos = cmd > 0.01
        mask_neg = cmd < -0.01
        T[i, mask_zero] = 0.0
        if np.any(mask_pos):
            p = params["pos"]
            T[i, mask_pos] = p["A"] + (p["K"] - p["A"]) / ((p["C"] + np.exp(-p["B"] * (cmd[mask_pos] - p["M"])))**(1.0 / p["v"]))
        if np.any(mask_neg):
            p = params["neg"]
            T[i, mask_neg] = p["A"] + (p["K"] - p["A"]) / ((p["C"] + np.exp(-p["B"] * (cmd[mask_neg] - p["M"])))**(1.0 / p["v"]))
        T[i] = np.clip(T[i], motor["max_rev"], motor["max_fwd"])
    Tu = np.sum(T, axis=0)
    y_coords = [
        motor["positions_yx"][3][0],
        motor["positions_yx"][1][0],
        motor["positions_yx"][2][0],
        motor["positions_yx"][0][0],
    ]
    Tr = np.sum([y * T[i] for i, y in enumerate(y_coords)], axis=0)
    return Tu, Tr

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
    Xu_pos = dynamic.get("Xu_pos", DYNAMIC_START["Xu"])
    Xu_neg = dynamic.get("Xu_neg", DYNAMIC_START["Xu"])
    Xuu_pos = dynamic.get("Xuu_pos", DYNAMIC_START["Xuu"])
    Xuu_neg = dynamic.get("Xuu_neg", DYNAMIC_START["Xuu"])
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
        if (abs(u_sim[k+1]) > 8.0 or abs(v_sim[k+1]) > 8.0 or abs(r_sim[k+1]) > 10.0 or not (np.isfinite(u_sim[k+1]) and np.isfinite(v_sim[k+1]) and np.isfinite(r_sim[k+1]))):
            raise RuntimeError("Numerical integration diverged.")
    return u_sim, v_sim, r_sim

def simulate_experiment_powell(experiment, dynamic, motor, fast=False):
    t = experiment["t"]
    u = experiment["u"]
    v = experiment["v"]
    r = experiment["r"]
    if "Tu" in experiment and "Tr" in experiment:
        Tu, Tr = experiment["Tu"], experiment["Tr"]
    else:
        Tu, Tr = experiment_to_tau_powell(experiment, motor)
    if fast:
        return rk4_integrate(t, u[0], v[0], r[0], Tu, Tr, dynamic)
    Tu_fn = interp1d(t, Tu, bounds_error=False, fill_value="extrapolate")
    Tr_fn = interp1d(t, Tr, bounds_error=False, fill_value="extrapolate")
    m11 = dynamic["m11"]
    m22 = dynamic["m22"]
    m33 = dynamic["m33"]
    Xu_pos = dynamic.get("Xu_pos", DYNAMIC_START["Xu"])
    Xu_neg = dynamic.get("Xu_neg", DYNAMIC_START["Xu"])
    Xuu_pos = dynamic.get("Xuu_pos", DYNAMIC_START["Xuu"])
    Xuu_neg = dynamic.get("Xuu_neg", DYNAMIC_START["Xuu"])
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
    def ode_system(tt, state):
        u_, v_, r_ = state
        tau_u = float(Tu_fn(tt))
        tau_r = float(Tr_fn(tt))
        s_val = 0.5 * (1.0 + np.tanh(20.0 * u_))
        Xu_val = Xu_neg + (Xu_pos - Xu_neg) * s_val
        Xuu_val = Xuu_neg + (Xuu_pos - Xuu_neg) * s_val
        du = (tau_u + m22 * v_ * r_ - Xu_val * u_ - Xuu_val * abs(u_) * u_) / m11
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
    if not sol.success or sol.y.shape[1] != len(t):
        raise RuntimeError(f"Integration failed for {experiment['name']}")
    return sol.y[0], sol.y[1], sol.y[2]

def normalized_mse(y, y_hat, mask=None):
    if mask is not None:
        if np.count_nonzero(mask) < 5:
            return 0.0
        y = y[mask]
        y_hat = y_hat[mask]
    val = np.mean((y - y_hat) ** 2) / (np.var(y) + 1e-6)
    if not np.isfinite(val):
        return 1e6
    return val

def r2(y, y_hat):
    return 1.0 - np.sum((y - y_hat) ** 2) / (np.sum((y - np.mean(y)) ** 2) + 1e-8)

def rms(y, y_hat):
    return float(np.sqrt(np.mean((y - y_hat) ** 2)))

def simulate_and_evaluate_dynamic_worker(args):
    ds, dynamic = args
    try:
        u_hat, v_hat, r_hat = rk4_integrate(ds['t'], ds['u'][0], ds['v'][0], ds['r'][0], ds['Tu'], ds['Tr'], dynamic)
    except Exception:
        return 1e6
    cost = normalized_mse(ds["u"], u_hat)
    return cost

def dynamic_cost(p, datasets, pool):
    dynamic = dynamic_from_vector(p)
    if (dynamic["Xu_pos"] < 1.0 or dynamic["Xu_neg"] < 1.0 or dynamic["Xuu_pos"] < 0.0 or dynamic["Xuu_neg"] < 0.0):
        return 1e6
    tasks = [(ds, dynamic) for ds in datasets]
    costs = pool.map(simulate_and_evaluate_dynamic_worker, tasks)
    return sum(costs) / len(datasets)

def load_experiment(path, dt=1.0/30.0):
    data = loadmat(path)
    pwm_raw = np.array([data[f"pwm{i+1}"].squeeze() for i in range(4)])
    u = data["vx"].squeeze()
    t = data["t"].squeeze() if "t" in data else np.arange(len(u)) * dt
    return {
        "name": os.path.basename(path),
        "t": t,
        "u": u,
        "v": data["vy"].squeeze(),
        "r": data["wz"].squeeze(),
        "pwm_raw": pwm_raw,
    }

def main():
    global global_train_datasets, global_pool, opt_history
    parser = argparse.ArgumentParser(description="Powell asymmetric Identification.")
    parser.add_argument("--data-dir", default=os.path.dirname(os.path.abspath(__file__)))
    parser.add_argument("--dynamic-maxiter", type=int, default=150)
    args = parser.parse_args()

    paths = sorted([os.path.join(args.data_dir, f) for f in os.listdir(args.data_dir) if f.startswith("experiment_") and f.endswith(".mat")])
    datasets = [load_experiment(p) for p in paths if os.path.exists(p)]
    if not datasets:
        raise RuntimeError("No experiment .mat files found.")
    print(f"Loaded {len(datasets)} experiments.")
    train_datasets = [ds for ds in datasets if ds["name"] not in ["experiment_01.mat", "experiment_02.mat"]]
    print(f"Using {len(train_datasets)} training experiments (excluding exp01 and exp02).")
    global_train_datasets = train_datasets
    motor = dict(MOTOR_TEMPLATE)
    print("\nPrecalculating forces and moments for all experiments...")
    for ds in datasets:
        ds["Tu"], ds["Tr"] = experiment_to_tau_powell(ds, motor)

    num_cores = min(multiprocessing.cpu_count(), 8)
    pool = multiprocessing.Pool(processes=num_cores)
    global_pool = pool

    opt_history = {
        "dynamic_iterations": [],
        "dynamic_costs": [],
        "dynamic_params": []
    }

    print(f"\n--- OPTIMIZING ASYMMETRIC DYNAMIC PARAMETERS ({num_cores} parallel cores) ---")
    p0_dynamic = np.array([
        DYNAMIC_START["Xu"],
        DYNAMIC_START["Xu"],
        DYNAMIC_START["Xuu"],
        DYNAMIC_START["Xuu"],
    ])
    bounds_dynamic = [
        (1.0, 150.0),
        (1.0, 150.0),
        (0.0, 200.0),
        (0.0, 250.0),
    ]

    init_cost_dyn = dynamic_cost(p0_dynamic, train_datasets, pool)
    opt_history["dynamic_iterations"].append(0)
    opt_history["dynamic_costs"].append(init_cost_dyn)
    opt_history["dynamic_params"].append(list(p0_dynamic))
    print(f"[Dynamics Opt] Initial | Cost: {init_cost_dyn:.6f}")

    def dynamic_callback(xk):
        current_cost = dynamic_cost(xk, global_train_datasets, global_pool)
        it = len(opt_history["dynamic_iterations"])
        opt_history["dynamic_iterations"].append(it)
        opt_history["dynamic_costs"].append(current_cost)
        opt_history["dynamic_params"].append(list(xk))
        print(f"[Dynamics Opt] Iteration {it:03d} | Cost: {current_cost:.6f}")

    res_dynamic = minimize(
        dynamic_cost,
        p0_dynamic,
        args=(train_datasets, pool),
        method="Powell",
        bounds=bounds_dynamic,
        callback=dynamic_callback,
        tol=1e-9,
        options={"maxiter": args.dynamic_maxiter, "disp": True, "ftol": 1e-9, "xtol": 1e-9}
    )
    dynamic = dynamic_from_vector(res_dynamic.x)
    pool.close()
    pool.join()

    with open("optimization_history_asymmetric.json", "w") as f:
        json.dump(opt_history, f, indent=4)
    print("\nOptimization history saved to 'optimization_history_asymmetric.json'!")

    with open("identified_dynamics_asymmetric.json", "w") as f:
        json.dump(dynamic, f, indent=4)
    print("Final parameters saved to 'identified_dynamics_asymmetric.json'!")

    with open("identified_motors_asymmetric.json", "w") as f:
        json.dump(motor, f, indent=4)
    print("Motor parameters saved to 'identified_motors_asymmetric.json'!")

    with open("iacquabot_asymmetric_identified_params.py", "w") as f:
        f.write("# Optimal dynamic parameters identified with Powell method (Asymmetric)\n")
        f.write("import numpy as np\n\n")
        f.write("# --- DYNAMIC PARAMETERS ---\n")
        f.write(f"m11 = {dynamic['m11']:.10f}\n")
        f.write(f"m22 = {dynamic['m22']:.10f}\n")
        f.write(f"m33 = {dynamic['m33']:.10f}\n")
        f.write(f"Xu_pos = {dynamic['Xu_pos']:.10f}\n")
        f.write(f"Xu_neg = {dynamic['Xu_neg']:.10f}\n")
        f.write(f"Xuu_pos = {dynamic['Xuu_pos']:.10f}\n")
        f.write(f"Xuu_neg = {dynamic['Xuu_neg']:.10f}\n")
        fixed_names = ['Yv', 'Yvv', 'Yvr', 'Yr', 'Yrv', 'Yrr', 'Yur', 'Nv', 'Nvv', 'Nvr', 'Nr', 'Nrv', 'Nrr']
        for name in fixed_names:
            f.write(f"{name} = {dynamic[name]:.10f}\n")
        f.write("\n# For compatibility, average values for Xu and Xuu\n")
        f.write(f"Xu = 0.5 * (Xu_pos + Xu_neg)\n")
        f.write(f"Xuu = 0.5 * (Xuu_pos + Xuu_neg)\n")
        f.write("theta = [m11, m22, m33, Xu, Xuu, Yv, Yvv, Yvr, Yr, Yrv, Yrr, Yur, Nv, Nvv, Nvr, Nr, Nrv, Nrr]\n\n")
        f.write("# --- MOTOR PARAMETERS ---\n")
        f.write(f"motor_neg_A = {motor['T200']['neg']['A']:.10f}\n")
        f.write(f"motor_neg_B = {motor['T200']['neg']['B']:.10f}\n")
        f.write(f"motor_neg_v = {motor['T200']['neg']['v']:.10f}\n")
        f.write(f"motor_neg_C = {motor['T200']['neg']['C']:.10f}\n")
        f.write(f"motor_neg_M = {motor['T200']['neg']['M']:.10f}\n")
        f.write(f"max_rev = {motor['max_rev']:.10f}\n\n")
        f.write(f"motor_pos_K = {motor['T200']['pos']['K']:.10f}\n")
        f.write(f"motor_pos_B = {motor['T200']['pos']['B']:.10f}\n")
        f.write(f"motor_pos_v = {motor['T200']['pos']['v']:.10f}\n")
        f.write(f"motor_pos_C = {motor['T200']['pos']['C']:.10f}\n")
        f.write(f"motor_pos_M = {motor['T200']['pos']['M']:.10f}\n")
        f.write(f"max_fwd = {motor['max_fwd']:.10f}\n")
    print("File 'iacquabot_asymmetric_identified_params.py' successfully generated!")

    fig_evol, ax = plt.subplots(figsize=(8, 4.5))
    ax.plot(opt_history["dynamic_iterations"], opt_history["dynamic_costs"], "b-o", label="Asymmetric Surge Cost (Powell)")
    ax.set_title("Optimization Cost Evolution")
    ax.set_xlabel("Iteration")
    ax.set_ylabel("Simulation Error Cost")
    ax.grid(True)
    ax.legend()
    plt.tight_layout()
    evol_plot_path = "optimization_evolution_asymmetric.png"
    plt.savefig(evol_plot_path, dpi=150)
    plt.close(fig_evol)
    print(f"Optimization evolution plot saved to '{evol_plot_path}'!")

    print("\n--- EVALUATING METRICS FOR ALL 10 EXPERIMENTS (ASYMMETRIC) ---")
    print(f"{'Experiment':<18} | {'Surge (u)':<17} | {'Sway (v)':<17} | {'Yaw (r)':<17}")
    print(f"{'':<18} | {'RMS':<7} {'R2':<8} | {'RMS':<7} {'R2':<8} | {'RMS':<7} {'R2':<8}")
    print("-" * 78)
    num_exps = len(datasets)
    fig, axes = plt.subplots(num_exps, 3, figsize=(15, 2.5 * num_exps), squeeze=False)
    metrics_dict = {}
    for idx, exp in enumerate(datasets):
        try:
            t_exp = exp["t"]
            u_real = exp["u"]
            v_real = exp["v"]
            r_real = exp["r"]
            try:
                u_sim, v_sim, r_sim = simulate_experiment_powell(exp, dynamic, motor, fast=False)
            except Exception:
                u_sim, v_sim, r_sim = simulate_experiment_powell(exp, dynamic, motor, fast=True)
            rms_u, r2_u = rms(u_real, u_sim), r2(u_real, u_sim)
            rms_v, r2_v = rms(v_real, v_sim), r2(v_real, v_sim)
            rms_r, r2_r = rms(r_real, r_sim), r2(r_real, r_sim)
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
    unified_plot_path = "validation_asymmetric_experiments.png"
    plt.savefig(unified_plot_path, dpi=150)
    plt.close(fig)
    print(f"\nAsymmetric validation plot successfully saved to '{unified_plot_path}'!")
    with open("identified_dynamics_asymmetric_metrics.json", "w") as f:
        json.dump(metrics_dict, f, indent=4)
    print("Metrics successfully saved to 'identified_dynamics_asymmetric_metrics.json'!")

if __name__ == "__main__":
    main()
