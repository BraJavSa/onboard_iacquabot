import argparse
import os

import numpy as np
from scipy.optimize import differential_evolution, minimize

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
    "Xu": 18.187662,
    "Xuu": 68.719290,
    "Yv": 325.907865,
    "Yvv": -0.006465,
    "Nr": 60.393403,
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


def motor_from_vector(p):
    motor = {
        **MOTOR_TEMPLATE,
        "T200": {
            "pos": dict(MOTOR_TEMPLATE["T200"]["pos"]),
            "neg": {
                "A": float(p[0]),
                "K": -0.00001,
                "B": float(p[1]),
                "v": float(p[2]),
                "C": float(p[3]),
                "M": float(p[4]),
            },
        },
        "max_rev": float(p[5]),
    }
    return motor


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


# ---------------------------------------------------------------------------
# Motor cost (unchanged – motor identification left as-is)
# ---------------------------------------------------------------------------

def motor_cost(p, datasets):
    motor = motor_from_vector(p)
    dynamic = dict(DYNAMIC_START)

    if motor["max_rev"] > -3.0 or motor["T200"]["neg"]["A"] > -3.0:
        return 1e6
    if motor["max_rev"] < motor["T200"]["neg"]["A"]:
        return 1e5 + (motor["T200"]["neg"]["A"] - motor["max_rev"]) ** 2

    total = 0.0
    for ds in datasets:
        try:
            u_hat, v_hat, r_hat = simulate_experiment(ds, dynamic, motor, fast=True)
        except RuntimeError:
            return 1e6

        reverse = ds["u"] < -0.05
        forward = ds["u"] > 0.05
        total += (
            3.0 * normalized_mse(ds["u"], u_hat, reverse)
            + 0.7 * normalized_mse(ds["u"], u_hat, forward)
            + 0.5 * normalized_mse(ds["u"], u_hat)
            + 0.8 * normalized_mse(ds["r"], r_hat)
            + 0.5 * normalized_mse(ds["v"], v_hat)
        )

    return total / len(datasets)


# ---------------------------------------------------------------------------
# Dynamic cost – improved version
# ---------------------------------------------------------------------------

def dynamic_cost(p, datasets, motor):
    """
    Weighted normalized-MSE cost over all DOFs.

    Improvements vs. original:
    - Separate surge terms for acceleration/deceleration transients, steady
      forward cruise, and reverse motion, each with distinct weights.
    - Stronger weight on sway to prevent it from being under-fit.
    - Soft physical-consistency penalty relating added-mass to damping scales.
    - Early exit (penalty) on any hard-bound violation before simulation.
    """
    dynamic = dynamic_from_vector(p)

    # ---- Hard physical constraints ----------------------------------------
    if any(dynamic[k] < 1.0 for k in ["m11", "m22", "m33", "Xu", "Yv", "Nr"]):
        return 1e6
    if dynamic["Xuu"] < 0.0 or dynamic["Nrr"] < 0.0 or dynamic["Yvv"] < -20.0:
        return 1e6

    # ---- Soft consistency penalty -----------------------------------------
    # Linear damping should be meaningful relative to inertia:
    #   tau_u = m11 / Xu  and  tau_v = m22 / Yv  should both be in [0.5, 30] s
    penalty = 0.0
    tau_u = dynamic["m11"] / (dynamic["Xu"] + 1e-6)
    tau_v = dynamic["m22"] / (dynamic["Yv"] + 1e-6)
    tau_r = dynamic["m33"] / (dynamic["Nr"] + 1e-6)
    for tau in (tau_u, tau_v, tau_r):
        if tau < 0.5:
            penalty += (0.5 - tau) ** 2 * 10.0
        elif tau > 30.0:
            penalty += (tau - 30.0) ** 2 * 0.1

    total = penalty
    for ds in datasets:
        try:
            u_hat, v_hat, r_hat = simulate_experiment(ds, dynamic, motor, fast=True)
        except RuntimeError:
            return 1e6

        u, v, r = ds["u"], ds["v"], ds["r"]

        # Masks for kinematically distinct phases
        fwd       = u >  0.05
        rev       = u < -0.05
        du        = np.gradient(u, ds.get("dt", 0.1))
        accel     = (du >  0.02) & fwd
        decel     = (du < -0.02) & fwd

        # Surge: weight transient phases more than cruise
        cost_u = (
            2.5 * normalized_mse(u, u_hat, rev)      # reverse motion (hardest)
            + 1.5 * normalized_mse(u, u_hat, accel)  # acceleration transient
            + 1.5 * normalized_mse(u, u_hat, decel)  # deceleration transient
            + 0.8 * normalized_mse(u, u_hat, fwd)    # forward cruise
            + 0.5 * normalized_mse(u, u_hat)         # global
        )

        # Sway: weighted higher than original to prevent under-fitting
        cost_v = 2.5 * normalized_mse(v, v_hat)

        # Yaw
        cost_r = 1.2 * normalized_mse(r, r_hat)

        total += cost_u + cost_v + cost_r

    return total / len(datasets)


# ---------------------------------------------------------------------------
# Two-phase + multi-start dynamic identification
# ---------------------------------------------------------------------------

def identify_dynamic(datasets, motor, args):
    """
    Stage 1 (optional): differential_evolution for global search.
    Stage 2:            Powell refinement from best candidate.
    Stage 3:            N random restarts with perturbation around best.
    Returns best dynamic dict found.
    """
    powell_opts = {"maxiter": args.dynamic_maxiter, "disp": True, "ftol": 1e-7}

    def run_powell(p0):
        res = minimize(
            dynamic_cost,
            p0,
            args=(datasets, motor),
            method="Powell",
            bounds=BOUNDS_DYNAMIC,
            options=powell_opts,
        )
        return res

    best_p = np.array(list(DYNAMIC_START.values()))
    best_cost = dynamic_cost(best_p, datasets, motor)

    # ------------------------------------------------------------------
    # Stage 1 – global search via differential evolution
    # ------------------------------------------------------------------
    if not args.no_global_search:
        print("Stage 1: differential_evolution global search...")
        de_result = differential_evolution(
            dynamic_cost,
            BOUNDS_DYNAMIC,
            args=(datasets, motor),
            strategy="best1bin",
            maxiter=max(30, args.dynamic_maxiter // 3),
            popsize=args.dynamic_popsize,
            tol=1e-5,
            mutation=(0.5, 1.5),
            recombination=0.9,
            seed=42,
            disp=True,
            polish=False,     # we polish ourselves with Powell
            workers=1,
        )
        if de_result.fun < best_cost:
            best_p    = de_result.x
            best_cost = de_result.fun
            print(f"  DE best cost: {best_cost:.6f}")

    # ------------------------------------------------------------------
    # Stage 2 – Powell refinement from best candidate so far
    # ------------------------------------------------------------------
    print("Stage 2: Powell refinement from best candidate...")
    res = run_powell(best_p)
    if res.fun < best_cost:
        best_p    = res.x
        best_cost = res.fun
    print(f"  Powell best cost: {best_cost:.6f}")

    # ------------------------------------------------------------------
    # Stage 3 – multi-start perturbation restarts
    # ------------------------------------------------------------------
    if args.dynamic_restarts > 0:
        print(f"Stage 3: {args.dynamic_restarts} random restarts...")
        rng = np.random.default_rng(seed=0)
        lo  = np.array([b[0] for b in BOUNDS_DYNAMIC])
        hi  = np.array([b[1] for b in BOUNDS_DYNAMIC])

        for i in range(args.dynamic_restarts):
            # Gaussian perturbation ±15 % of parameter range, clipped to bounds
            scale = 0.15 * (hi - lo)
            p_try = np.clip(best_p + rng.normal(0.0, scale), lo, hi)
            res_i = run_powell(p_try)
            if res_i.fun < best_cost:
                best_p    = res_i.x
                best_cost = res_i.fun
                print(f"  Restart {i+1}: new best cost {best_cost:.6f}")
            else:
                print(f"  Restart {i+1}: {res_i.fun:.6f}  (no improvement)")

    print(f"\nFinal dynamic cost: {best_cost:.6f}")
    return dynamic_from_vector(best_p)


# ---------------------------------------------------------------------------
# Evaluation
# ---------------------------------------------------------------------------

def evaluate(datasets, dynamic, motor):
    print()
    print("Internal validation")
    print("experiment       R2_u    RMS_u    R2_v    RMS_v    R2_r    RMS_r")
    for ds in datasets:
        u_hat, v_hat, r_hat = simulate_experiment(ds, dynamic, motor, fast=False)
        print(
            f"{ds['name']:<14} "
            f"{r2(ds['u'], u_hat):>7.3f} {rms(ds['u'], u_hat):>8.4f} "
            f"{r2(ds['v'], v_hat):>7.3f} {rms(ds['v'], v_hat):>8.4f} "
            f"{r2(ds['r'], r_hat):>7.3f} {rms(ds['r'], r_hat):>8.4f}"
        )


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def parse_args():
    parser = argparse.ArgumentParser(
        description="Identify the vessel dynamic model and save JSON parameter files."
    )
    parser.add_argument("--data-dir",    default=os.path.dirname(os.path.abspath(__file__)))
    parser.add_argument("--dynamic-json", default=None)
    parser.add_argument("--motor-json",   default=None)

    # Motor (unchanged)
    parser.add_argument("--motor-maxiter",              type=int,  default=25)
    parser.add_argument("--skip-motor-identification",  action="store_true")

    # Dynamic – new options
    parser.add_argument("--dynamic-maxiter",            type=int,  default=150,
                        help="Max iterations for each Powell run.")
    parser.add_argument("--dynamic-restarts",           type=int,  default=5,
                        help="Number of random-perturbation restarts after main Powell (Stage 3).")
    parser.add_argument("--dynamic-popsize",            type=int,  default=12,
                        help="Population size multiplier for differential_evolution (Stage 1).")
    parser.add_argument("--no-global-search",           action="store_true",
                        help="Skip differential_evolution and go straight to Powell.")
    parser.add_argument("--skip-dynamic-identification", action="store_true")

    return parser.parse_args()


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    args = parse_args()
    default_dynamic_json, default_motor_json = default_json_paths(args.data_dir)
    dynamic_json = args.dynamic_json or default_dynamic_json
    motor_json   = args.motor_json   or default_motor_json

    paths    = [os.path.join(args.data_dir, f"experiment_{i:02d}.mat") for i in range(1, 11)]
    datasets = [load_experiment(path) for path in paths if os.path.exists(path)]
    if not datasets:
        raise RuntimeError("No experiment files were found.")

    reverse_datasets = [ds for ds in datasets if np.count_nonzero(ds["u"] < -0.05) >= 5]
    if not reverse_datasets:
        raise RuntimeError("No reverse-motion samples were found.")

    print(f"Loaded {len(datasets)} experiments.")
    print(f"Using {len(reverse_datasets)} experiments with reverse-motion samples for motor identification.")

    # ------------------------------------------------------------------
    # Motor identification (left unchanged)
    # ------------------------------------------------------------------
    if args.skip_motor_identification:
        motor = dict(MOTOR_TEMPLATE)
    else:
        p0_motor     = np.array([
            MOTOR_TEMPLATE["T200"]["neg"]["A"],
            MOTOR_TEMPLATE["T200"]["neg"]["B"],
            MOTOR_TEMPLATE["T200"]["neg"]["v"],
            MOTOR_TEMPLATE["T200"]["neg"]["C"],
            MOTOR_TEMPLATE["T200"]["neg"]["M"],
            MOTOR_TEMPLATE["max_rev"],
        ])
        bounds_motor = [
            (-45.0, -5.0),
            (0.2,   10.0),
            (0.05,   1.5),
            (0.3,    2.0),
            (-1.8,   0.2),
            (-40.0, -5.0),
        ]
        print("Identifying reverse thruster branch...")
        res_motor = minimize(
            motor_cost,
            p0_motor,
            args=(reverse_datasets,),
            method="Powell",
            bounds=bounds_motor,
            options={"maxiter": args.motor_maxiter, "disp": True},
        )
        motor = motor_from_vector(res_motor.x)

    # ------------------------------------------------------------------
    # Dynamic identification (improved)
    # ------------------------------------------------------------------
    if args.skip_dynamic_identification:
        dynamic = dynamic_from_vector(np.array(list(DYNAMIC_START.values())))
    else:
        print("Identifying dynamic parameters...")
        dynamic = identify_dynamic(datasets, motor, args)

    write_json(motor_json,   motor)
    write_json(dynamic_json, dynamic)

    print(f"Motor parameters saved to   {motor_json}")
    print(f"Dynamic parameters saved to {dynamic_json}")
    evaluate(datasets, dynamic, motor)


if __name__ == "__main__":
    main()