import argparse
import os

import numpy as np
from scipy.optimize import differential_evolution

from model_validation_utils import (
    default_json_paths,
    load_experiment,
    r2,
    rms,
    simulate_experiment,
    write_json,
)

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
        [-0.5, 0.40],
        [0.5, 0.40],
        [-0.5, -0.50],
        [0.5, -0.50],
    ],
    "angles_deg": [0.0, 0.0, 0.0, 0.0],
}

DYNAMIC_START = {
    "m11": 132.556203,
    "m22": 159.117429,
    "m33": 18.018928,
    "Xu":  18.187662,
    "Xuu": 68.719290,
    "Yv":  325.907865,
    "Yvv": -0.006465,
    "Nr":  60.393403,
    "Nrr": 0.853737,
}

BOUNDS_DYNAMIC = [
    (20.0,  400.0),   # m11
    (20.0,  500.0),   # m22
    (1.0,   120.0),   # m33
    (1.0,   300.0),   # Xu
    (0.0,   500.0),   # Xuu
    (1.0,   700.0),   # Yv
    (-20.0, 300.0),   # Yvv
    (1.0,   250.0),   # Nr
    (0.0,   120.0),   # Nrr
]


def normalized_mse(y, y_hat, mask=None):
    if mask is not None:
        if np.count_nonzero(mask) < 5:
            return 0.0
        y = y[mask]
        y_hat = y_hat[mask]
    return np.mean((y - y_hat) ** 2) / (np.var(y) + 1e-6)


def dynamic_from_vector(p):
    return {
        "model": "Fossen 3DOF output-error",
        "m11": float(p[0]),
        "m22": float(p[1]),
        "m33": float(p[2]),
        "Xu":  float(p[3]),
        "Xuu": float(p[4]),
        "Yv":  float(p[5]),
        "Yvv": float(p[6]),
        "Nr":  float(p[7]),
        "Nrr": float(p[8]),
    }


def preprocess_datasets(datasets):
    for ds in datasets:
        u  = ds["u"]
        dt = ds.get("dt", 0.1)
        du = np.gradient(u, dt)

        ds["_fwd"]   = u >  0.05
        ds["_rev"]   = u < -0.05
        ds["_accel"] = (du >  0.02) & (u > 0.05)
        ds["_decel"] = (du < -0.02) & (u > 0.05)

    return datasets


def dynamic_cost(p, datasets):
    m11, m22, m33, Xu, Xuu, Yv, Yvv, Nr, Nrr = p

    if m11 < 1.0 or m22 < 1.0 or m33 < 1.0 or Xu < 1.0 or Yv < 1.0 or Nr < 1.0:
        return 1e6
    if Xuu < 0.0 or Nrr < 0.0 or Yvv < -20.0:
        return 1e6

    penalty = 0.0
    for m, d in ((m11, Xu), (m22, Yv), (m33, Nr)):
        tau = m / d
        if tau < 0.5:
            penalty += (0.5 - tau) ** 2 * 10.0
        elif tau > 30.0:
            penalty += (tau - 30.0) ** 2 * 0.1

    dynamic = dynamic_from_vector(p)

    total = penalty
    for ds in datasets:
        try:
            u_hat, v_hat, r_hat = simulate_experiment(ds, dynamic, MOTOR_TEMPLATE, fast=True)
        except RuntimeError:
            return 1e6

        u, v, r = ds["u"], ds["v"], ds["r"]

        total += (
            2.5 * normalized_mse(u, u_hat, ds["_rev"])
            + 1.5 * normalized_mse(u, u_hat, ds["_accel"])
            + 1.5 * normalized_mse(u, u_hat, ds["_decel"])
            + 0.8 * normalized_mse(u, u_hat, ds["_fwd"])
            + 0.5 * normalized_mse(u, u_hat)
            + 2.5 * normalized_mse(v, v_hat)
            + 1.2 * normalized_mse(r, r_hat)
        )

    return total / len(datasets)


def evaluate(datasets, dynamic):
    print()
    print("Internal validation")
    print("experiment       R2_u    RMS_u    R2_v    RMS_v    R2_r    RMS_r")
    for ds in datasets:
        u_hat, v_hat, r_hat = simulate_experiment(ds, dynamic, MOTOR_TEMPLATE, fast=False)
        print(
            f"{ds['name']:<14} "
            f"{r2(ds['u'], u_hat):>7.3f} {rms(ds['u'], u_hat):>8.4f} "
            f"{r2(ds['v'], v_hat):>7.3f} {rms(ds['v'], v_hat):>8.4f} "
            f"{r2(ds['r'], r_hat):>7.3f} {rms(ds['r'], r_hat):>8.4f}"
        )


def parse_args():
    parser = argparse.ArgumentParser(
        description="Identify the vessel dynamic model using a fixed motor template."
    )
    parser.add_argument("--data-dir",        default=os.path.dirname(os.path.abspath(__file__)))
    parser.add_argument("--dynamic-json",    default=None)
    parser.add_argument("--dynamic-popsize", type=int, default=8,
                        help="Population size multiplier for DE.")
    return parser.parse_args()


def main():
    args = parse_args()
    default_dynamic_json, _ = default_json_paths(args.data_dir)
    base, ext = os.path.splitext(args.dynamic_json or default_dynamic_json)
    dynamic_json = f"{base}_de{ext}"

    paths    = [os.path.join(args.data_dir, f"experiment_{i:02d}.mat") for i in range(1, 11)]
    datasets = [load_experiment(path) for path in paths if os.path.exists(path)]
    if not datasets:
        raise RuntimeError("No experiment files were found.")

    datasets = preprocess_datasets(datasets)

    print(f"Loaded {len(datasets)} experiments.")
    print("Motor model: fixed MOTOR_TEMPLATE (no identification).")
    print("Running differential_evolution (max 30 steps, parallel)...")

    de = differential_evolution(
        dynamic_cost,
        BOUNDS_DYNAMIC,
        args=(datasets,),
        strategy="best1bin",
        maxiter=40,
        popsize=args.dynamic_popsize,
        tol=1e-4,
        mutation=(0.5, 1.5),
        recombination=0.9,
        seed=42,
        disp=True,
        polish=False,
        workers=-1,
        updating="deferred",
    )

    print(f"\nFinal dynamic cost: {de.fun:.6f}")
    dynamic = dynamic_from_vector(de.x)

    write_json(dynamic_json, dynamic)
    print(f"Dynamic parameters saved to {dynamic_json}")
    evaluate(datasets, dynamic)


if __name__ == "__main__":
    main()