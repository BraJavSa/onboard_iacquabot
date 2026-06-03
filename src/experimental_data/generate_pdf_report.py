"""
model_validation_report.py
===========================
Generates a multi-page PDF report with one page per experiment (up to 10),
each showing measured vs. simulated velocities (u, v, r) with error shading
and RMS values, using the Fossen 3-DOF vessel model.

The PDF filename is derived from the experiment files found in --data-dir.

─────────────────────────────────────────────────────────────────────────────
LANGUAGE SETTING  ← change here
─────────────────────────────────────────────────────────────────────────────
"""

LANGUAGE = "es"   # "en" = English  |  "es" = Spanish

# ─────────────────────────────────────────────────────────────────────────────

import argparse
import json
import os
import sys
import io

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.gridspec import GridSpec
from matplotlib.ticker import AutoMinorLocator
from matplotlib.backends.backend_pdf import PdfPages

import numpy as np

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
#  Localisation strings
# ═══════════════════════════════════════════════════════════════════════════

STRINGS = {
    "en": {
        "page_title":   "Model Validation",
        "time_label":   "Time (s)",
        "measured":     "Measured",
        "model":        "Model",
        "error_region": "Error region",
        "ylabels": {
            "u": "Surge velocity\n$u$  [m/s]",
            "v": "Sway velocity\n$v$  [m/s]",
            "r": "Yaw rate\n$r$  [rad/s]",
        },
    },
    "es": {
        "page_title":   "Validación del Modelo",
        "time_label":   "Tiempo (s)",
        "measured":     "Medido",
        "model":        "Modelo",
        "error_region": "Región de error",
        "ylabels": {
            "u": "Velocidad de avance\n$u$  [m/s]",
            "v": "Velocidad lateral\n$v$  [m/s]",
            "r": "Velocidad angular\n$r$  [rad/s]",
        },
    },
}


# ═══════════════════════════════════════════════════════════════════════════
#  Plot constants
# ═══════════════════════════════════════════════════════════════════════════

COLOR_MEASURED = "#000000"
COLOR_MODEL    = "#1a56a0"
COLOR_FILL     = "#aec6e8"

YLIMS = {
    "u": (-1.0,  1.6),
    "v": (-1.0,  1.0),
    "r": (-1.0,  1.0),
}


# ═══════════════════════════════════════════════════════════════════════════
#  Style
# ═══════════════════════════════════════════════════════════════════════════

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
#  Single-experiment page
# ═══════════════════════════════════════════════════════════════════════════

def make_page(ds, u_hat, v_hat, r_hat, exp_name: str, lang: str) -> plt.Figure:
    """Return a Figure with three stacked panels for one experiment."""
    S = STRINGS[lang]
    t = ds["t"]
    signals = [
        ("u", ds["u"], u_hat),
        ("v", ds["v"], v_hat),
        ("r", ds["r"], r_hat),
    ]

    fig = plt.figure(figsize=(6.5, 7.5))
    fig.patch.set_facecolor("white")

    # Page title (experiment name)
    fig.text(
        0.5, 0.995,
        f"{S['page_title']} — {exp_name}",
        ha="center", va="top",
        fontsize=9, fontweight="bold",
        fontfamily="serif",
    )

    gs = GridSpec(
        3, 1,
        figure=fig,
        left=0.14, right=0.97,
        top=0.955, bottom=0.07,
        hspace=0.38,
    )

    for row, (key, y_meas, y_sim) in enumerate(signals):
        ax = fig.add_subplot(gs[row])
        rms_val = rms(y_meas, y_sim)

        # error shading
        ax.fill_between(
            t, y_meas, y_sim,
            color=COLOR_FILL, alpha=0.55,
            linewidth=0, zorder=2,
        )

        # measured
        ax.plot(t, y_meas,
                color=COLOR_MEASURED, linewidth=1.2,
                linestyle="-", label=S["measured"], zorder=4)

        # simulated
        ax.plot(t, y_sim,
                color=COLOR_MODEL, linewidth=1.1,
                linestyle="--", label=S["model"], zorder=5)

        # axes
        ax.set_xlim(t[0], t[-1])
        ax.set_ylim(YLIMS[key])
        ax.set_ylabel(S["ylabels"][key], labelpad=6, multialignment="center")
        ax.yaxis.set_minor_locator(AutoMinorLocator(4))
        ax.xaxis.set_minor_locator(AutoMinorLocator(4))
        ax.tick_params(which="both", top=True, right=True)

        if row < 2:
            ax.set_xticklabels([])
        else:
            ax.set_xlabel(S["time_label"], labelpad=3)

        # legend
        err_patch = mpatches.Patch(
            facecolor=COLOR_FILL, alpha=0.6,
            edgecolor="none", label=S["error_region"],
        )
        rms_handle = mpatches.Patch(
            visible=False, label=f"RMS = {rms_val:.4f}",
        )
        handles, labels = ax.get_legend_handles_labels()
        ax.legend(
            handles + [err_patch, rms_handle],
            labels  + [S["error_region"], f"RMS = {rms_val:.4f}"],
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
        description="Generate a PDF validation report for all experiments."
    )
    parser.add_argument("--data-dir",     default=os.path.dirname(os.path.abspath(__file__)))
    parser.add_argument("--dynamic-json", default=None)
    parser.add_argument("--motor-json",   default=None)
    parser.add_argument("--output",       default=None,
                        help="Output PDF path (default: <data-dir>/<exp_prefix>_validation.pdf)")
    parser.add_argument("--fast",  action="store_true",
                        help="Use Euler integrator instead of RK45.")
    parser.add_argument("--language", default=None, choices=["en", "es"],
                        help="Override LANGUAGE variable (en/es).")
    return parser.parse_args()


def load_json(path: str) -> dict:
    with open(path, "r") as f:
        return json.load(f)


def main():
    args = parse_args()
    lang = args.language if args.language else LANGUAGE
    if lang not in STRINGS:
        print(f"[ERROR] Unknown language '{lang}'. Use 'en' or 'es'.")
        sys.exit(1)

    default_dynamic_json, default_motor_json = default_json_paths(args.data_dir)
    dynamic_json = args.dynamic_json or default_dynamic_json
    motor_json   = args.motor_json   or default_motor_json

    for path, label in [(dynamic_json, "dynamic"), (motor_json, "motor")]:
        if not os.path.exists(path):
            print(f"[ERROR] {label.capitalize()} JSON not found: {path}")
            sys.exit(1)

    dynamic = load_json(dynamic_json)
    motor   = load_json(motor_json)

    # Collect experiments 01–10
    exp_paths = []
    for i in range(1, 11):
        p = os.path.join(args.data_dir, f"experiment_{i:02d}.mat")
        if os.path.exists(p):
            exp_paths.append(p)

    if not exp_paths:
        print("[ERROR] No experiment files found in:", args.data_dir)
        sys.exit(1)

    print(f"Found {len(exp_paths)} experiment(s). Language: {lang}.")

    # Output PDF name derived from the experiment set
    if args.output:
        pdf_path = args.output
    else:
        first_name = os.path.splitext(os.path.basename(exp_paths[0]))[0]
        # e.g.  experiment_01_validation.pdf
        prefix = first_name.replace("_01", "")   # → "experiment"
        pdf_path = os.path.join(args.data_dir, f"{prefix}_validation.pdf")

    apply_article_style()

    with PdfPages(pdf_path) as pdf:
        for exp_path in exp_paths:
            exp_name = os.path.splitext(os.path.basename(exp_path))[0]
            print(f"  Processing {exp_name} ...", end=" ", flush=True)

            try:
                ds = load_experiment(exp_path)
                u_hat, v_hat, r_hat = simulate_experiment(ds, dynamic, motor, fast=args.fast)
                fig = make_page(ds, u_hat, v_hat, r_hat, exp_name, lang)
                pdf.savefig(fig, dpi=300, bbox_inches="tight", facecolor="white")
                plt.close(fig)

                # console RMS summary
                for key, ym, ys in [("u", ds["u"], u_hat),
                                     ("v", ds["v"], v_hat),
                                     ("r", ds["r"], r_hat)]:
                    pass   # already shown in figure; omit repetition here
                print("done")

            except Exception as e:
                print(f"FAILED ({e})")

        # PDF metadata
        d = pdf.infodict()
        d["Title"]   = "Fossen 3-DOF Model Validation Report"
        d["Author"]  = "model_validation_report.py"
        d["Subject"] = "Vessel system identification — output-error method"

    print(f"\nReport saved to: {pdf_path}")


if __name__ == "__main__":
    main()