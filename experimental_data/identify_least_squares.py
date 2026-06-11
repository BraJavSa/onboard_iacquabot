import argparse
import json
import os

import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import savgol_filter
from scipy.integrate import solve_ivp
from scipy.interpolate import interp1d

from model_validation_utils import (
    load_experiment,
    experiment_to_tau,
    r2,
    rms
)

def simulate_experiment_custom(experiment, dynamic, motor, fast=False):
    t = experiment["t"]
    u = experiment["u"]
    v = experiment["v"]
    r = experiment["r"]
    tau_u, tau_v, tau_r = experiment_to_tau(experiment, motor)
    tau_u_fn = interp1d(t, tau_u, bounds_error=False, fill_value="extrapolate")
    tau_v_fn = interp1d(t, tau_v, bounds_error=False, fill_value="extrapolate")
    tau_r_fn = interp1d(t, tau_r, bounds_error=False, fill_value="extrapolate")

    def ode(tt, x):
        u_, v_, r_ = x
        tauu = float(tau_u_fn(tt))
        tauv = float(tau_v_fn(tt))
        taur = float(tau_r_fn(tt))
        
        du = (
            tauu
            + dynamic["m22"] * v_ * r_
            - dynamic["Xu"] * u_
            - dynamic["Xuu"] * abs(u_) * u_
        ) / dynamic["m11"]
        
        dv = (
            tauv
            - dynamic["m11"] * u_ * r_
            - dynamic["Yv"] * v_
            - dynamic["Yvv"] * abs(v_) * v_
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

def compute_derivatives(t, x, window=11, poly=2):
    # Smooth the signal slightly and compute derivative
    # If the signal is too short, fall back to simple gradient
    if len(x) < window:
        return np.gradient(x, t)
    
    dt = np.mean(np.diff(t))
    # savgol_filter(x, window_length, polyorder, deriv, delta)
    dx = savgol_filter(x, window_length=window, polyorder=poly, deriv=1, delta=dt)
    return dx

def main():
    parser = argparse.ArgumentParser(description="Identify vessel model using Least Squares.")
    parser.add_argument("--data-dir", default=os.path.dirname(os.path.abspath(__file__)))
    args = parser.parse_args()

    motor_path = os.path.join(args.data_dir, "identified_motors.json")
    print(f"Loading identified motor parameters from {motor_path}...")
    with open(motor_path, "r", encoding="utf-8") as f:
        motor = json.load(f)

    paths = [os.path.join(args.data_dir, f"experiment_{i:02d}.mat") for i in range(1, 11)]
    datasets = [load_experiment(path) for path in paths if os.path.exists(path)]
    if not datasets:
        raise RuntimeError("No experiment files were found.")

    print(f"Loaded {len(datasets)} experiments.")

    # We will build global Y matrix and tau vector
    # theta = [m11, m22, m33, m23, m32, Xu, Xuu, Yv, Yvv, Nr, Nrr]
    Y_all = []
    tau_all = []

    for ds in datasets:
        t = ds["t"]
        u = ds["u"]
        v = ds["v"]
        r = ds["r"]
        
        # Calculate accelerations
        du = compute_derivatives(t, u)
        dv = compute_derivatives(t, v)
        dr = compute_derivatives(t, r)
        
        # Get tau from experiments
        tau_u, tau_v, tau_r = experiment_to_tau(ds, motor)
        
        for i in range(len(t)):
            # Row for Surge (tau_u)
            Y_u = [
                du[i],        # m11
                -v[i]*r[i],   # m22
                0.0,          # m33
                u[i],         # Xu
                abs(u[i])*u[i], # Xuu
                0.0,          # Yv
                0.0,          # Yvv
                0.0,          # Nr
                0.0           # Nrr
            ]
            
            # Row for Sway (tau_v)
            Y_v = [
                u[i]*r[i],    # m11
                dv[i],        # m22
                0.0,          # m33
                0.0,          # Xu
                0.0,          # Xuu
                v[i],         # Yv
                abs(v[i])*v[i], # Yvv
                0.0,          # Nr
                0.0           # Nrr
            ]
            
            # Row for Yaw (tau_r)
            Y_r = [
                -u[i]*v[i],   # m11
                u[i]*v[i],    # m22
                dr[i],        # m33
                0.0,          # Xu
                0.0,          # Xuu
                0.0,          # Yv
                0.0,          # Yvv
                r[i],         # Nr
                abs(r[i])*r[i]  # Nrr
            ]
            
            Y_all.extend([Y_u, Y_v, Y_r])
            tau_all.extend([tau_u[i], tau_v[i], tau_r[i]])

    Y_all = np.array(Y_all)
    tau_all = np.array(tau_all)

    print("Solving Least Squares...")
    # theta = (Y^T Y)^-1 Y^T tau
    # We use np.linalg.lstsq
    theta, residuals, rank, s = np.linalg.lstsq(Y_all, tau_all, rcond=None)

    param_names = ["m11", "m22", "m33", "Xu", "Xuu", "Yv", "Yvv", "Nr", "Nrr"]
    
    print("\nIdentified Parameters (Least Squares):")
    dynamic_ls = {"model": "Fossen 3DOF Least Squares"}
    for name, val in zip(param_names, theta):
        print(f"  {name}: {val:.6f}")
        dynamic_ls[name] = val

    print("\nEcuaciones del Modelo Identificado (Forma Matricial):")
    print("-" * 50)
    print("M =")
    print(f"| {dynamic_ls['m11']:8.3f}         0.0         0.0 |")
    print(f"|      0.0  {dynamic_ls['m22']:8.3f}         0.0 |")
    print(f"|      0.0         0.0  {dynamic_ls['m33']:8.3f} |")
    
    print("\nC(v) =")
    print(f"|           0.0            0.0      -m22*v |")
    print(f"|           0.0            0.0       m11*u |")
    print(f"|         m22*v         -m11*u         0.0 |")
    
    print("\nD(v) =")
    print(f"| Xu + Xuu|u|          0.0          0.0 |")
    print(f"|         0.0   Yv + Yvv|v|         0.0 |")
    print(f"|         0.0          0.0  Nr + Nrr|r| |")
    print("-" * 50)

    dynamic_orig_path = os.path.join(args.data_dir, "identified_dynamics.json")
    print(f"\nLoading original dynamics from {dynamic_orig_path}...")
    with open(dynamic_orig_path, "r", encoding="utf-8") as f:
        dynamic_orig = json.load(f)

    print("\n" + "="*80)
    print("COMPARING ORIGINAL MODEL VS LEAST SQUARES MODEL")
    print("="*80)
    print(f"{'Exp':<10} | {'R2_u (Orig)':<12} {'R2_u (LS)':<12} | {'R2_v (Orig)':<12} {'R2_v (LS)':<12} | {'R2_r (Orig)':<12} {'R2_r (LS)':<12}")
    print("-" * 80)
    
    print("\nPlotting experiments (Velocities)...")
    fig, axes = plt.subplots(10, 3, figsize=(15, 20))
    fig.suptitle("Experiment Results (Original vs Least Squares)", fontsize=16)
    
    for i, ds in enumerate(datasets):
        if i >= 10: break
        t = ds['t']
        
        try:
            u_hat_orig, v_hat_orig, r_hat_orig = simulate_experiment_custom(ds, dynamic_orig, motor, fast=False)
            r2_u_orig = r2(ds['u'], u_hat_orig)
            r2_v_orig = r2(ds['v'], v_hat_orig)
            r2_r_orig = r2(ds['r'], r_hat_orig)
        except RuntimeError:
            u_hat_orig, v_hat_orig, r_hat_orig = np.zeros_like(t), np.zeros_like(t), np.zeros_like(t)
            r2_u_orig, r2_v_orig, r2_r_orig = -99.9, -99.9, -99.9
            
        try:
            u_hat_ls, v_hat_ls, r_hat_ls = simulate_experiment_custom(ds, dynamic_ls, motor, fast=False)
            r2_u_ls = r2(ds['u'], u_hat_ls)
            r2_v_ls = r2(ds['v'], v_hat_ls)
            r2_r_ls = r2(ds['r'], r_hat_ls)
        except RuntimeError:
            u_hat_ls, v_hat_ls, r_hat_ls = np.zeros_like(t), np.zeros_like(t), np.zeros_like(t)
            r2_u_ls, r2_v_ls, r2_r_ls = -99.9, -99.9, -99.9

        print(f"{ds['name']:<10} | {r2_u_orig:>10.3f}   {r2_u_ls:>10.3f} | {r2_v_orig:>10.3f}   {r2_v_ls:>10.3f} | {r2_r_orig:>10.3f}   {r2_r_ls:>10.3f}")

        # Plot Surge (u)
        axes[i, 0].plot(t, ds['u'], 'k--', label='Measured u')
        axes[i, 0].plot(t, u_hat_orig, 'orange', alpha=0.7, label='Original')
        axes[i, 0].plot(t, u_hat_ls, 'b-', label='Least Squares')
        axes[i, 0].set_ylabel(f'Exp {i+1}\nu (m/s)')
        axes[i, 0].grid(True)
        if i == 0: 
            axes[i, 0].set_title('Surge Velocity')
            axes[i, 0].legend()
        
        # Plot Sway (v)
        axes[i, 1].plot(t, ds['v'], 'k--', label='Measured v')
        axes[i, 1].plot(t, v_hat_orig, 'orange', alpha=0.7, label='Original')
        axes[i, 1].plot(t, v_hat_ls, 'r-', label='Least Squares')
        axes[i, 1].set_ylabel('v (m/s)')
        axes[i, 1].grid(True)
        if i == 0: 
            axes[i, 1].set_title('Sway Velocity')
            axes[i, 1].legend()
        
        # Plot Yaw Rate (r)
        axes[i, 2].plot(t, ds['r'], 'k--', label='Measured r')
        axes[i, 2].plot(t, r_hat_orig, 'orange', alpha=0.7, label='Original')
        axes[i, 2].plot(t, r_hat_ls, 'g-', label='Least Squares')
        axes[i, 2].set_ylabel('r (rad/s)')
        axes[i, 2].grid(True)
        if i == 0: 
            axes[i, 2].set_title('Yaw Rate')
            axes[i, 2].legend()
        
        if i == 9:
            axes[i, 0].set_xlabel('Time (s)')
            axes[i, 1].set_xlabel('Time (s)')
            axes[i, 2].set_xlabel('Time (s)')
            
    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    plt.savefig('least_squares_velocities_fit.png')
    print("\nVelocities fit plot saved to least_squares_velocities_fit.png")

if __name__ == "__main__":
    main()
