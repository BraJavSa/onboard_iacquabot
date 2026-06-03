"""
model_validation_plot.py
========================
Validation plot for the Fossen 3-DOF vessel model.
Scientific-article aesthetic: white background, serif fonts, clean axes.

Usage
-----
python model_validation_plot.py \
    --experiment experiment_01.mat \
    --dynamic-json dynamic.json \
    --motor-json   motor.json

Optional flags
--------------
--data-dir      Directory that contains the experiment and JSON files (default: script dir)
--experiment    File name (or full path) of the .mat experiment to validate
--dynamic-json  Path to dynamic parameters JSON (default: auto-detected)
--motor-json    Path to motor parameters JSON   (default: auto-detected)
--fast          Use fast (Euler) integrator instead of RK45
--save          If given, save the figure to this path instead of showing it interactively
"""

import argparse
import json
import os
import sys

import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np
from matplotlib.gridspec import GridSpec
from matplotlib.ticker import AutoMinorLocator

try:
    from model_validation_utils import (
        default_json_paths,
        load_experiment,
        rms,
        simulate_experiment,
    )
except ImportError:
    print(
        "[ERROR] Could not import model_validation_utils.\n"
        "Make sure model_validation_utils.py is on PYTHONPATH or in the same directory."
    )
    sys.exit(1)


# ═══════════════════════════════════════════════════════════════════════════
#  Style — scientific article
# ═══════════════════════════════════════════════════════════════════════════

COLOR_MEASURED = "#000000"
COLOR_MODEL    = "#1a56a0"
COLOR_FILL     = "#aec6e8"

YLIMS = {
    "u": (-1.0,  1.6),
    "v": (-1.0,  1.0),
    "r": (-1.0,  1.0),
}

YLABELS = {
    "u": "Surge velocity\n$u$  [m/s]",
    "v": "Sway velocity\n$v$  [m/s]",
    "r": "Yaw rate\n$r$  [rad/s]",
}


def apply_article_style():
    plt.rcParams.update(
        {
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
            "legend.fontsize":     8,
            "legend.handlelength": 2.2,
            "lines.linewidth":     1.2,
        }
    )


# ═══════════════════════════════════════════════════════════════════════════
#  Figure builder
# ═══════════════════════════════════════════════════════════════════════════

def build_figure(ds, u_hat, v_hat, r_hat):
    apply_article_style()

    t = ds["t"]
    signals = [
        ("u", ds["u"], u_hat),
        ("v", ds["v"], v_hat),
        ("r", ds["r"], r_hat),
    ]

    fig = plt.figure(figsize=(6.5, 7.5), dpi=150)
    fig.patch.set_facecolor("white")

    gs = GridSpec(
        3, 1,
        figure=fig,
        left=0.14, right=0.97,
        top=0.98, bottom=0.07,
        hspace=0.38,
    )

    for row, (key, y_meas, y_sim) in enumerate(signals):
        ax = fig.add_subplot(gs[row])

        rms_val = rms(y_meas, y_sim)

        # error shading
        ax.fill_between(
            t, y_meas, y_sim,
            color=COLOR_FILL,
            alpha=0.55,
            linewidth=0,
            zorder=2,
        )

        # measured
        ax.plot(
            t, y_meas,
            color=COLOR_MEASURED,
            linewidth=1.2,
            linestyle="-",
            label="Measured",
            zorder=4,
        )

        # simulated
        ax.plot(
            t, y_sim,
            color=COLOR_MODEL,
            linewidth=1.1,
            linestyle="--",
            label="Model",
            zorder=5,
        )

        # axes limits
        ax.set_xlim(t[0], t[-1])
        ax.set_ylim(YLIMS[key])
        ax.set_ylabel(YLABELS[key], labelpad=6, multialignment="center")
        ax.yaxis.set_minor_locator(AutoMinorLocator(4))
        ax.xaxis.set_minor_locator(AutoMinorLocator(4))
        ax.tick_params(which="both", top=True, right=True)

        if row < 2:
            ax.set_xticklabels([])
        else:
            ax.set_xlabel("Time (s)", labelpad=3)

        # legend: Measured | Model | Error region | (blank) RMS = x.xxxx
        err_patch = mpatches.Patch(
            facecolor=COLOR_FILL, alpha=0.6,
            edgecolor="none",
            label="Error region",
        )
        # Invisible handle so RMS appears as a plain text entry
        rms_handle = mpatches.Patch(visible=False, label=f"RMS = {rms_val:.4f}")

        handles, labels = ax.get_legend_handles_labels()
        ax.legend(
            handles + [err_patch, rms_handle],
            labels  + ["Error region", f"RMS = {rms_val:.4f}"],
            loc="upper right",
            fontsize=7.5,
            handlelength=2.0,
            borderpad=0.5,
            labelspacing=0.3,
            handletextpad=0.4,
        )

    return fig


# ═══════════════════════════════════════════════════════════════════════════
#  CLI
# ═══════════════════════════════════════════════════════════════════════════

def parse_args():
    parser = argparse.ArgumentParser(
        description="Plot real vs simulated vessel velocities (Fossen 3-DOF)."
    )
    parser.add_argument("--data-dir", default=os.path.dirname(os.path.abspath(__file__)))
    parser.add_argument("--experiment",   default=None)
    parser.add_argument("--dynamic-json", default=None)
    parser.add_argument("--motor-json",   default=None)
    parser.add_argument("--fast",  action="store_true")
    parser.add_argument("--save",  default=None, metavar="FILE")
    return parser.parse_args()


def resolve_experiment_path(args) -> str:
    if args.experiment is None:
        return os.path.join(args.data_dir, "experiment_01.mat")
    if os.path.isabs(args.experiment):
        return args.experiment
    return os.path.join(args.data_dir, args.experiment)


def load_json(path: str) -> dict:
    with open(path, "r") as f:
        return json.load(f)


def main():
    args = parse_args()

    default_dynamic_json, default_motor_json = default_json_paths(args.data_dir)
    dynamic_json = args.dynamic_json or default_dynamic_json
    motor_json   = args.motor_json   or default_motor_json

    for path, label in [(dynamic_json, "dynamic"), (motor_json, "motor")]:
        if not os.path.exists(path):
            print(f"[ERROR] {label.capitalize()} JSON not found: {path}")
            sys.exit(1)

    dynamic = load_json(dynamic_json)
    motor   = load_json(motor_json)

    exp_path = resolve_experiment_path(args)
    if not os.path.exists(exp_path):
        print(f"[ERROR] Experiment file not found: {exp_path}")
        sys.exit(1)

    print(f"Loading  : {exp_path}")
    ds = load_experiment(exp_path)

    print("Simulating ...")
    u_hat, v_hat, r_hat = simulate_experiment(ds, dynamic, motor, fast=args.fast)

    print(f"\n{'Signal':<8}  {'RMS':>8}")
    print("-" * 20)
    for key, y_meas, y_sim in [("u", ds["u"], u_hat), ("v", ds["v"], v_hat), ("r", ds["r"], r_hat)]:
        print(f"  {key:<6}  {rms(y_meas, y_sim):>8.4f}")

    fig = build_figure(ds, u_hat, v_hat, r_hat)

    if args.save:
        fig.savefig(args.save, dpi=300, bbox_inches="tight", facecolor="white")
        print(f"\nFigure saved to: {args.save}")
    else:
        plt.show()


if __name__ == "__main__":
    main()