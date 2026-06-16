"""
validation_de.py
Usa MOTOR_TEMPLATE fijo y el JSON del DE para graficar
real vs modelado de los 10 experimentos y guardar un PDF.

Uso:
    python validation_de.py
    python validation_de.py --dynamic-json identified_dynamics_de.json
"""

import argparse
import json
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np
from matplotlib.gridspec import GridSpec
from matplotlib.ticker import AutoMinorLocator
from matplotlib.backends.backend_pdf import PdfPages

from model_validation_utils import (
    default_json_paths,
    load_experiment,
    rms,
    simulate_experiment,
)

# ── motor fijo ───────────────────────────────────────────────────────────────
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
    "positions_yx": [[-0.5, 0.40], [0.5, 0.40], [-0.5, -0.50], [0.5, -0.50]],
    "angles_deg": [0.0, 0.0, 0.0, 0.0],
}

# ── estilo artículo ──────────────────────────────────────────────────────────
COLOR_MEASURED = "#000000"
COLOR_MODEL    = "#1a56a0"
COLOR_FILL     = "#aec6e8"

YLIMS   = {"u": (-1.0, 1.6), "v": (-1.0, 1.0), "r": (-1.0, 1.0)}
YLABELS = {
    "u": "Surge velocity\n$u$  [m/s]",
    "v": "Sway velocity\n$v$  [m/s]",
    "r": "Yaw rate\n$r$  [rad/s]",
}

def apply_style():
    plt.rcParams.update({
        "figure.facecolor": "white", "axes.facecolor": "white",
        "axes.edgecolor": "black",   "axes.linewidth": 0.8,
        "xtick.direction": "in",     "ytick.direction": "in",
        "xtick.major.size": 4.0,     "ytick.major.size": 4.0,
        "xtick.minor.size": 2.5,     "ytick.minor.size": 2.5,
        "axes.grid": True,           "grid.color": "#d0d0d0",
        "grid.linewidth": 0.45,      "grid.linestyle": "--",
        "font.family": "serif",
        "font.serif": ["Times New Roman", "DejaVu Serif", "serif"],
        "mathtext.fontset": "dejavuserif",
        "axes.labelsize": 9,         "xtick.labelsize": 8,
        "ytick.labelsize": 8,        "legend.fontsize": 7.5,
        "lines.linewidth": 1.2,
    })


def build_figure(ds, u_hat, v_hat, r_hat):
    apply_style()
    t = ds["t"]
    signals = [("u", ds["u"], u_hat), ("v", ds["v"], v_hat), ("r", ds["r"], r_hat)]

    fig = plt.figure(figsize=(6.5, 7.5), dpi=150)
    fig.patch.set_facecolor("white")
    fig.suptitle(ds["name"], fontsize=10, fontweight="bold", y=1.005)
    gs = GridSpec(3, 1, figure=fig, left=0.14, right=0.97,
                  top=0.96, bottom=0.07, hspace=0.38)

    for row, (key, y_meas, y_sim) in enumerate(signals):
        ax = fig.add_subplot(gs[row])
        rms_val = rms(y_meas, y_sim)

        ax.fill_between(t, y_meas, y_sim, color=COLOR_FILL, alpha=0.55, linewidth=0, zorder=2)
        ax.plot(t, y_meas, color=COLOR_MEASURED, lw=1.2, ls="-",  label="Measured", zorder=4)
        ax.plot(t, y_sim,  color=COLOR_MODEL,    lw=1.1, ls="--", label="Model",    zorder=5)

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

        err_patch = mpatches.Patch(facecolor=COLOR_FILL, alpha=0.6,
                                   edgecolor="none", label="Error region")
        rms_handle = mpatches.Patch(visible=False, label=f"RMS = {rms_val:.4f}")
        handles, labels = ax.get_legend_handles_labels()
        ax.legend(handles + [err_patch, rms_handle],
                  labels  + ["Error region", f"RMS = {rms_val:.4f}"],
                  loc="upper right", borderpad=0.5, labelspacing=0.3, handletextpad=0.4)
    return fig


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--data-dir",     default=os.path.dirname(os.path.abspath(__file__)))
    parser.add_argument("--dynamic-json", default=None)
    parser.add_argument("--output",       default=None)
    return parser.parse_args()


def main():
    args = parse_args()

    default_dynamic_json, _ = default_json_paths(args.data_dir)
    # Por defecto busca el JSON del DE
    dynamic_json = args.dynamic_json or os.path.join(
        args.data_dir, "identified_dynamics_de.json"
    )
    if not os.path.exists(dynamic_json):
        dynamic_json = default_dynamic_json

    output_pdf = args.output or os.path.join(
        args.data_dir,
        os.path.splitext(os.path.basename(dynamic_json))[0] + "_validation.pdf"
    )

    with open(dynamic_json) as f:
        dynamic = json.load(f)

    paths    = [os.path.join(args.data_dir, f"experiment_{i:02d}.mat") for i in range(1, 11)]
    datasets = [load_experiment(p) for p in paths if os.path.exists(p)]
    if not datasets:
        raise RuntimeError("No se encontraron archivos de experimento.")

    print(f"Modelo:  {dynamic_json}")
    print(f"Salida:  {output_pdf}")
    print(f"{'Experimento':<16} {'RMS_u':>7} {'RMS_v':>7} {'RMS_r':>7}")

    with PdfPages(output_pdf) as pdf:
        for ds in datasets:
            u_hat, v_hat, r_hat = simulate_experiment(ds, dynamic, MOTOR_TEMPLATE, fast=False)
            print(f"  {ds['name']:<14} {rms(ds['u'],u_hat):>7.4f} "
                  f"{rms(ds['v'],v_hat):>7.4f} {rms(ds['r'],r_hat):>7.4f}")
            fig = build_figure(ds, u_hat, v_hat, r_hat)
            pdf.savefig(fig, bbox_inches="tight", facecolor="white")
            plt.close(fig)

    print(f"\nPDF guardado: {output_pdf}")


if __name__ == "__main__":
    main()