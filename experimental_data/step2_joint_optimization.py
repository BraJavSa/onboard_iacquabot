#!/usr/bin/env python3
"""
Step 2: Joint Motor and Hydrodynamic Parameter Optimization (9-parameter model).
"""
import os
import json
import argparse
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.io import loadmat
from scipy.signal import savgol_filter
from scipy.optimize import minimize, lsq_linear
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
        [-0.3, 0.40],
        [0.3, 0.40],
        [-0.3, -0.50],
        [0.3, -0.50],
    ],
    "angles_deg": [0.0, 0.0, 0.0, 0.0],
}

PARAM_NAMES = ['m11', 'm22', 'm33', 'Xu', 'Xuu', 'Yv', 'Yvv', 'Nr', 'Nrr']

LB_DYN = [20.0, 30.0, 5.0, 1.0, 1.0, 50.0, 1.0, 1.0, 1.0]
UB_DYN = [150.0, 150.0, 50.0, 150.0, 150.0, 300.0, 150.0, 150.0, 150.0]

global_train_datasets = []
global_pool = None
global_phi_train = None

def build_allocation_matrix(positions_yx, angles_deg):
    positions_yx = np.asarray(positions_yx, dtype=float)
    angles = np.deg2rad(np.asarray(angles_deg, dtype=float))
    y = positions_yx[:, 0]
    x = positions_yx[:, 1]
    c = np.cos(angles)
    s = np.sin(angles)
    return np.vstack([c, s, y * c + x * s])

def motor_from_vector(p):
    motor = dict(MOTOR_TEMPLATE)
    motor["T200"] = {
        "pos": {
            "A": float(p[0]), "K": float(p[1]), "B": float(p[2]), "v": float(p[3]), "C": float(p[4]), "M": float(p[5])
        },
        "neg": {
            "A": float(p[6]), "K": float(p[7]), "B": float(p[8]), "v": float(p[9]), "C": float(p[10]), "M": float(p[11])
        }
    }
    motor["max_fwd"] = min(abs(float(p[1])) * 1.1, 36.3827)
    motor["max_rev"] = max(-abs(float(p[6])) * 1.1, -28.4393)
    return motor

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

def build_phi_matrix(acc_u, acc_v, acc_r, u, v, r):
    N = len(u)
    Phi_list = []
    for k in range(N):
        row_u = np.zeros(9)
        row_u[0] = acc_u[k]       # m11
        row_u[1] = -v[k] * r[k]   # m22
        row_u[3] = u[k]           # Xu
        row_u[4] = abs(u[k]) * u[k] # Xuu
        Phi_list.append(row_u)

        row_v = np.zeros(9)
        row_v[0] = u[k] * r[k]    # m11
        row_v[1] = acc_v[k]       # m22
        row_v[5] = v[k]           # Yv
        row_v[6] = abs(v[k]) * v[k] # Yvv
        Phi_list.append(row_v)

        row_r = np.zeros(9)
        row_r[0] = -u[k] * v[k]   # m11
        row_r[1] = u[k] * v[k]    # m22
        row_r[2] = acc_r[k]       # m33
        row_r[7] = r[k]           # Nr
        row_r[8] = abs(r[k]) * r[k] # Nrr
        Phi_list.append(row_r)
    return np.array(Phi_list)

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

def normalized_mse(y, y_hat):
    val = np.mean((y - y_hat) ** 2) / (np.var(y) + 1e-6)
    return val if np.isfinite(val) else 1e6

def rms(y, y_hat):
    return float(np.sqrt(np.mean((y - y_hat) ** 2)))

def r2(y, y_hat):
    return 1.0 - np.sum((y - y_hat) ** 2) / (np.sum((y - np.mean(y)) ** 2) + 1e-8)

def mae(y, y_hat):
    return float(np.mean(np.abs(y - y_hat)))

def evaluate_joint_worker(args):
    ds, motor, theta_dict = args
    try:
        Tu, Tr = experiment_to_tau(ds, motor)
        u_hat, v_hat, r_hat = rk4_integrate(ds["t"], ds["u_raw"][0], ds["v_raw"][0], ds["r_raw"][0], Tu, Tr, theta_dict)
        cost = normalized_mse(ds["u_raw"], u_hat) + normalized_mse(ds["v_raw"], v_hat) + normalized_mse(ds["r_raw"], r_hat)
        return cost
    except Exception as e:
        return 1e6

def joint_cost(p, datasets, pool):
    motor = motor_from_vector(p)
    tau_global_list = []
    for ds in datasets:
        Tu, Tr = experiment_to_tau(ds, motor)
        N = len(Tu)
        for k in range(N):
            tau_global_list.append(Tu[k])
            tau_global_list.append(0.0)
            tau_global_list.append(Tr[k])
    tau_global = np.array(tau_global_list)

    res_lsq = lsq_linear(global_phi_train, tau_global, bounds=(LB_DYN, UB_DYN))
    theta = res_lsq.x
    theta_dict = {name: float(theta[i]) for i, name in enumerate(PARAM_NAMES)}

    tasks = [(ds, motor, theta_dict) for ds in datasets]
    costs = pool.map(evaluate_joint_worker, tasks)
    return sum(costs) / len(datasets)

def main():
    global global_train_datasets, global_pool, global_phi_train
    parser = argparse.ArgumentParser(description="Step 2: Joint dynamic and motor parameters optimization (9-param).")
    parser.add_argument("--data-dir", default=os.path.dirname(os.path.abspath(__file__)))
    parser.add_argument("--maxiter", type=int, default=50)
    args = parser.parse_args()

    paths = sorted([os.path.join(args.data_dir, f) for f in os.listdir(args.data_dir) if f.startswith("experiment_") and f.endswith(".mat")])
    datasets = [load_experiment(p) for p in paths]
    train_datasets = [ds for ds in datasets if ds["name"] not in ["experiment_01.mat", "experiment_02.mat"]]
    print(f"Loaded {len(datasets)} experiments. Using {len(train_datasets)} training sets.")

    global_train_datasets = train_datasets
    num_cores = min(multiprocessing.cpu_count(), 8)
    pool = multiprocessing.Pool(processes=num_cores)
    global_pool = pool

    phi_list = []
    for ds in train_datasets:
        phi_exp = build_phi_matrix(ds["acc_u"], ds["acc_v"], ds["acc_r"], ds["u"], ds["v"], ds["r"])
        phi_list.append(phi_exp)
    global_phi_train = np.vstack(phi_list)

    pos_init = MOTOR_TEMPLATE["T200"]["pos"]
    neg_init = MOTOR_TEMPLATE["T200"]["neg"]
    p0 = np.array([
        pos_init["A"], pos_init["K"], pos_init["B"], pos_init["v"], pos_init["C"], pos_init["M"],
        neg_init["A"], neg_init["K"], neg_init["B"], neg_init["v"], neg_init["C"], neg_init["M"],
    ])

    bounds = [
        (-0.1, 0.1),       # pos A
        (10.0, 36.3827),   # pos K
        (0.1, 10.0),       # pos B
        (0.01, 2.0),       # pos v
        (0.1, 5.0),        # pos C
        (-0.5, 0.5),       # pos M
        (-28.4393, -10.0), # neg A
        (-0.1, 0.1),       # neg K
        (0.1, 10.0),       # neg B
        (0.01, 2.0),       # neg v
        (0.1, 5.0),        # neg C
        (-2.0, 0.0),       # neg M
    ]

    print("\n--- RUNNING JOINT POWELL OPTIMIZATION (CO-IDENTIFICATION 9-PARAM) ---")
    initial_cost = joint_cost(p0, train_datasets, pool)
    print(f"Initial Cost: {initial_cost:.6f}")

    history_costs = []
    def callback(xk):
        c = joint_cost(xk, global_train_datasets, global_pool)
        history_costs.append(c)
        print(f"Iteration {len(history_costs):03d} | Cost: {c:.6f}")

    res = minimize(
        joint_cost,
        p0,
        args=(train_datasets, pool),
        method="Powell",
        bounds=bounds,
        callback=callback,
        options={"maxiter": args.maxiter, "disp": True}
    )

    opt_motor = motor_from_vector(res.x)
    tau_global_list = []
    for ds in train_datasets:
        Tu, Tr = experiment_to_tau(ds, opt_motor)
        N = len(Tu)
        for k in range(N):
            tau_global_list.append(Tu[k])
            tau_global_list.append(0.0)
            tau_global_list.append(Tr[k])
    tau_global = np.array(tau_global_list)

    res_lsq = lsq_linear(global_phi_train, tau_global, bounds=(LB_DYN, UB_DYN))
    theta_opt = res_lsq.x
    opt_dynamics = {name: float(theta_opt[i]) for i, name in enumerate(PARAM_NAMES)}
    opt_dynamics["model"] = "Fossen 3DOF 9-parameter (Joint Co-Identified)"

    pool.close()
    pool.join()

    print("\n--- JOINT OPTIMIZATION COMPLETE (9-PARAM) ---")
    save_dyn_path = os.path.join(args.data_dir, "identified_dynamics.json")
    with open(save_dyn_path, "w") as f:
        json.dump(opt_dynamics, f, indent=4)
    print(f"Joint dynamics saved to {save_dyn_path}")

    save_motor_path = os.path.join(args.data_dir, "identified_motors.json")
    with open(save_motor_path, "w") as f:
        json.dump(opt_motor, f, indent=4)
    print(f"Joint motors saved to {save_motor_path}")

    dyn_s1_path = os.path.join(args.data_dir, "identified_dynamics_step1.json")
    if os.path.exists(dyn_s1_path):
        with open(dyn_s1_path, "r") as f:
            dyn_s1 = json.load(f)
    else:
        dyn_s1 = opt_dynamics

    print("\n--- EVALUATING VALIDATION METRICS (STAGE 2 / JOINT OPT) ---")
    print(f"{'Experiment':<18} | {'Surge (u)':<17} | {'Sway (v)':<17} | {'Yaw (r)':<17}")
    print(f"{'':<18} | {'RMS':<7} {'MAE':<8} | {'RMS':<7} {'MAE':<8} | {'RMS':<7} {'MAE':<8}")
    print("-" * 78)

    metrics_dict = {}
    for idx, exp in enumerate(datasets):
        try:
            Tu_opt, Tr_opt = experiment_to_tau(exp, opt_motor)
            u_opt, v_opt, r_opt = rk4_integrate(exp["t"], exp["u_raw"][0], exp["v_raw"][0], exp["r_raw"][0], Tu_opt, Tr_opt, opt_dynamics)
            
            rms_u = rms(exp["u_raw"], u_opt)
            mae_u = mae(exp["u_raw"], u_opt)
            rms_v = rms(exp["v_raw"], v_opt)
            mae_v = mae(exp["v_raw"], v_opt)
            rms_r = rms(exp["r_raw"], r_opt)
            mae_r = mae(exp["r_raw"], r_opt)
            
            print(f"{exp['name']:<18} | {rms_u:.4f}  {mae_u:>7.4f} | {rms_v:.4f}  {mae_v:>7.4f} | {rms_r:.4f}  {mae_r:>7.4f}")
            
            metrics_dict[exp["name"]] = {
                "u": {"rms": rms_u, "mae": mae_u},
                "v": {"rms": rms_v, "mae": mae_v},
                "r": {"rms": rms_r, "mae": mae_r}
            }
        except Exception as e:
            print(f"Error evaluating {exp['name']}: {e}")

    save_metrics_path = os.path.join(args.data_dir, "identified_dynamics_metrics.json")
    with open(save_metrics_path, "w") as f:
        json.dump(metrics_dict, f, indent=4)
    print(f"Metrics saved to {save_metrics_path}")

    print("\n--- GENERATING PROPULSION CURVES PDF COMPARISON ---")
    cmd_range = np.linspace(-1.0, 1.0, 500)
    original_thrust = cmd_to_thrust(cmd_range, MOTOR_TEMPLATE)
    optimized_thrust = cmd_to_thrust(cmd_range, opt_motor)

    fig, ax = plt.subplots(figsize=(8, 6))
    ax.plot(cmd_range, original_thrust, 'b-', label='Original Curve (Template)', linewidth=2)
    ax.plot(cmd_range, optimized_thrust, 'r--', label='Optimized Curve (Joint Opt Bounded)', linewidth=2)
    ax.set_title('T200 Thruster Command vs. Thrust Force (9-param model)')
    ax.set_xlabel('Normalized Command [-1, 1]')
    ax.set_ylabel('Thrust Force [N]')
    ax.grid(True, linestyle='--', alpha=0.7)
    ax.legend()
    
    pdf_path = os.path.join(args.data_dir, "thruster_curves_comparison.pdf")
    plt.savefig(pdf_path, format='pdf', bbox_inches='tight')
    plt.close()
    print(f"Propulsion curves comparison report saved to: {pdf_path}")

if __name__ == "__main__":
    main()
