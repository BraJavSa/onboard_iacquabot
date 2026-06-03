import argparse
import os

import numpy as np
from scipy.optimize import minimize

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
        [-0.35, 0.40],
        [0.35, 0.40],
        [-0.35, -0.50],
        [0.35, -0.50],
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
        "Xu": float(p[3]),
        "Xuu": float(p[4]),
        "Yv": float(p[5]),
        "Yvv": float(p[6]),
        "Nr": float(p[7]),
        "Nrr": float(p[8]),
    }


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


def dynamic_cost(p, datasets, motor):
    dynamic = dynamic_from_vector(p)

    if any(dynamic[k] < 1.0 for k in ["m11", "m22", "m33", "Xu", "Yv", "Nr"]):
        return 1e6
    if dynamic["Xuu"] < 0.0 or dynamic["Nrr"] < 0.0 or dynamic["Yvv"] < -20.0:
        return 1e6

    total = 0.0
    for ds in datasets:
        try:
            u_hat, v_hat, r_hat = simulate_experiment(ds, dynamic, motor, fast=True)
        except RuntimeError:
            return 1e6

        total += (
            normalized_mse(ds["u"], u_hat)
            + 2.0 * normalized_mse(ds["v"], v_hat)
            + normalized_mse(ds["r"], r_hat)
        )

    return total / len(datasets)


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


def parse_args():
    parser = argparse.ArgumentParser(description="Identify the final vessel model and save JSON parameter files.")
    parser.add_argument("--data-dir", default=os.path.dirname(os.path.abspath(__file__)))
    parser.add_argument("--dynamic-json", default=None)
    parser.add_argument("--motor-json", default=None)
    parser.add_argument("--motor-maxiter", type=int, default=25)
    parser.add_argument("--dynamic-maxiter", type=int, default=150)
    parser.add_argument("--skip-motor-identification", action="store_true")
    parser.add_argument("--skip-dynamic-identification", action="store_true")
    return parser.parse_args()


def main():
    args = parse_args()
    default_dynamic_json, default_motor_json = default_json_paths(args.data_dir)
    dynamic_json = args.dynamic_json or default_dynamic_json
    motor_json = args.motor_json or default_motor_json

    paths = [os.path.join(args.data_dir, f"experiment_{i:02d}.mat") for i in range(1, 11)]
    datasets = [load_experiment(path) for path in paths if os.path.exists(path)]
    if not datasets:
        raise RuntimeError("No experiment files were found.")

    reverse_datasets = [ds for ds in datasets if np.count_nonzero(ds["u"] < -0.05) >= 5]
    if not reverse_datasets:
        raise RuntimeError("No reverse-motion samples were found.")

    print(f"Loaded {len(datasets)} experiments.")
    print(f"Using {len(reverse_datasets)} experiments with reverse-motion samples for motor identification.")

    if args.skip_motor_identification:
        motor = dict(MOTOR_TEMPLATE)
    else:
        p0_motor = np.array([
            MOTOR_TEMPLATE["T200"]["neg"]["A"],
            MOTOR_TEMPLATE["T200"]["neg"]["B"],
            MOTOR_TEMPLATE["T200"]["neg"]["v"],
            MOTOR_TEMPLATE["T200"]["neg"]["C"],
            MOTOR_TEMPLATE["T200"]["neg"]["M"],
            MOTOR_TEMPLATE["max_rev"],
        ])
        bounds_motor = [
            (-45.0, -5.0),
            (0.2, 10.0),
            (0.05, 1.5),
            (0.3, 2.0),
            (-1.8, 0.2),
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

    if args.skip_dynamic_identification:
        dynamic = dynamic_from_vector(np.array(list(DYNAMIC_START.values())))
    else:
        p0_dynamic = np.array([
            DYNAMIC_START["m11"],
            DYNAMIC_START["m22"],
            DYNAMIC_START["m33"],
            DYNAMIC_START["Xu"],
            DYNAMIC_START["Xuu"],
            DYNAMIC_START["Yv"],
            DYNAMIC_START["Yvv"],
            DYNAMIC_START["Nr"],
            DYNAMIC_START["Nrr"],
        ])
        bounds_dynamic = [
            (20.0, 400.0),
            (20.0, 500.0),
            (1.0, 120.0),
            (1.0, 300.0),
            (0.0, 500.0),
            (1.0, 700.0),
            (-20.0, 300.0),
            (1.0, 250.0),
            (0.0, 120.0),
        ]
        print("Identifying dynamic parameters...")
        res_dynamic = minimize(
            dynamic_cost,
            p0_dynamic,
            args=(datasets, motor),
            method="Powell",
            bounds=bounds_dynamic,
            options={"maxiter": args.dynamic_maxiter, "disp": True},
        )
        dynamic = dynamic_from_vector(res_dynamic.x)

    write_json(motor_json, motor)
    write_json(dynamic_json, dynamic)

    print(f"Motor parameters saved to {motor_json}")
    print(f"Dynamic parameters saved to {dynamic_json}")
    evaluate(datasets, dynamic, motor)


if __name__ == "__main__":
    main()
