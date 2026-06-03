"""
USV Force Calculator — Modelo de Fossen 3-DOF
Calcula T_u, T_v, T_r a partir de comandos PWM o cmd [-1, 1]
y grafica las fuerzas resultantes.

Propulsores:
  FR (+0.30, +0.45)  — normal
  FL (-0.30, +0.45)  — invertido
  BR (+0.30, -0.55)  — normal
  BL (-0.30, -0.55)  — invertido
Todos apuntan en dirección surge (+x body frame).
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib.gridspec as gridspec
from matplotlib.patches import FancyArrowPatch, Circle, FancyBboxPatch

# ─────────────────────────────────────────────
# 1. MODELO DE EMPUJE T200
# ─────────────────────────────────────────────

# Parámetros Richards/Chapman calibrados contra curvas del fabricante
T200_PARAMS = {
    "pos": dict(A=0.000001, K=40.0209, B=2.6249, v=0.1615, C=0.9432, M=0.00001),
    "neg": dict(A=-31.499,  K=-0.00001, B=3.6986, v=0.3264, C=0.9713, M=-1.0),
}

def thrust_T200(cmd: float) -> float:
    """
    Convierte comando normalizado cmd ∈ [-1, 1] a fuerza [N].
    Zona muerta: |cmd| <= 0.01 → 0 N.
    """
    if np.abs(cmd) <= 0.01:
        return 0.0
    p = T200_PARAMS["pos"] if cmd > 0 else T200_PARAMS["neg"]
    return p["A"] + (p["K"] - p["A"]) / (p["C"] + np.exp(-p["B"] * (cmd - p["M"]))) ** (1.0 / p["v"])

thrust_T200_vec = np.vectorize(thrust_T200)


def pwm_to_cmd(pwm_us: float, inverted: bool = False) -> float:
    """
    Convierte PWM [µs] → cmd ∈ [-1, 1] aplicando corrección de inversión.
    Rango: [1100, 1900], neutro 1500.
    """
    cmd = (pwm_us - 1500.0) / 400.0   # 1900-1500 = 400
    return -cmd if inverted else cmd


# ─────────────────────────────────────────────
# 2. CONFIGURACIÓN DEL VEHÍCULO
# ─────────────────────────────────────────────

# Posiciones de motores en body frame [m]: (x_lateral, y_longitudinal)
MOTORS = {
    "FR": {"pos": np.array([ 0.30,  0.45]), "inverted": False},
    "FL": {"pos": np.array([-0.30,  0.45]), "inverted": True },
    "BR": {"pos": np.array([ 0.30, -0.55]), "inverted": False},
    "BL": {"pos": np.array([-0.30, -0.55]), "inverted": True },
}

# Dimensiones del bote [m]
BOAT_WIDTH  = 0.6756   # 26.6"
BOAT_LENGTH = 1.1976   # 3.93'


# ─────────────────────────────────────────────
# 3. FUNCIÓN PRINCIPAL DE CÁLCULO
# ─────────────────────────────────────────────

def compute_tau(cmds: dict) -> dict:
    """
    Calcula las fuerzas generalizadas τ = [T_u, T_v, T_r].

    Parámetros
    ----------
    cmds : dict con claves 'FR', 'FL', 'BR', 'BL', valores cmd ∈ [-1, 1]
           (después de haber deshecho la inversión del hardware, si aplica)

    Retorna
    -------
    dict con:
        thrusts   : dict {motor: fuerza [N]}
        T_u       : fuerza de surge [N]
        T_v       : fuerza de sway  [N]  (= 0, todos apuntan en surge)
        T_r       : torque de yaw   [N·m]
    """
    thrusts = {}
    T_u = T_v = T_r = 0.0

    for name, cfg in MOTORS.items():
        cmd_raw = cmds[name]
        # La corrección de inversión ya viene deshecha en los datos procesados.
        # Si se pasan comandos "hardware" (como llegarían al ESP32 antes de invertir),
        # descomentar la línea siguiente:
        # cmd_eff = -cmd_raw if cfg["inverted"] else cmd_raw
        cmd_eff = cmd_raw
        T_i = thrust_T200(cmd_eff)
        thrusts[name] = T_i

        xi, yi = cfg["pos"]
        # Todos los propulsores empujan en +x (surge). No hay componente sway.
        T_u += T_i
        # Torque yaw: (r × F)_z = -y_i * T_i  (fuerza en +x, posición (xi, yi))
        T_r += -yi * T_i

    return {"thrusts": thrusts, "T_u": T_u, "T_v": T_v, "T_r": T_r}


# ─────────────────────────────────────────────
# 4. EJEMPLO DE USO
# ─────────────────────────────────────────────

if __name__ == "__main__":

    # --- Caso de ejemplo: avance recto al 60% ---
    example_cmds = {"FR": 0.6, "FL": 0.6, "BR": 0.6, "BL": 0.6}
    result = compute_tau(example_cmds)
    print("=== Caso ejemplo: avance recto (cmd=0.6 en todos) ===")
    for m, T in result["thrusts"].items():
        print(f"  {m}: {T:+.3f} N")
    print(f"  T_u = {result['T_u']:+.3f} N")
    print(f"  T_v = {result['T_v']:+.3f} N")
    print(f"  T_r = {result['T_r']:+.3f} N·m\n")

    # ─────────────────────────────────────────
    # 5. BARRIDO DE COMANDOS Y VISUALIZACIÓN
    # ─────────────────────────────────────────

    cmd_range = np.linspace(-1, 1, 400)

    # --- Curva de empuje T200 ---
    thrust_curve = thrust_T200_vec(cmd_range)

    # --- Barrido: avance simétrico (cmd igual en los 4) ---
    Tu_sym = []
    Tr_sym = []
    for c in cmd_range:
        r = compute_tau({"FR": c, "FL": c, "BR": c, "BL": c})
        Tu_sym.append(r["T_u"])
        Tr_sym.append(r["T_r"])
    Tu_sym = np.array(Tu_sym)
    Tr_sym = np.array(Tr_sym)

    # --- Barrido: giro diferencial puro (derecha +, izquierda −) ---
    Tu_turn = []
    Tr_turn = []
    for c in cmd_range:
        r = compute_tau({"FR": c, "FL": -c, "BR": c, "BL": -c})
        Tu_turn.append(r["T_u"])
        Tr_turn.append(r["T_r"])
    Tu_turn = np.array(Tu_turn)
    Tr_turn = np.array(Tr_turn)

    # --- Barrido: asimetría front/back (frente +, popa −) ---
    Tu_asym = []
    Tr_asym = []
    for c in cmd_range:
        r = compute_tau({"FR": c, "FL": c, "BR": -c, "BL": -c})
        Tu_asym.append(r["T_u"])
        Tr_asym.append(r["T_r"])
    Tu_asym = np.array(Tu_asym)
    Tr_asym = np.array(Tr_asym)

    # ─────────────────────────────────────────
    # 6. FIGURA PRINCIPAL
    # ─────────────────────────────────────────

    fig = plt.figure(figsize=(16, 12))
    fig.patch.set_facecolor("#F8F8F6")

    gs = gridspec.GridSpec(
        3, 3,
        figure=fig,
        hspace=0.45, wspace=0.38,
        left=0.07, right=0.97, top=0.93, bottom=0.07
    )

    C_BLUE   = "#185FA5"
    C_TEAL   = "#0F6E56"
    C_CORAL  = "#993C1D"
    C_AMBER  = "#854F0B"
    C_GRAY   = "#5F5E5A"
    C_PURPLE = "#534AB7"
    C_GREEN  = "#3B6D11"

    # ── (A) Curva de empuje T200 ──────────────────────────────────────
    ax_thrust = fig.add_subplot(gs[0, 0])
    ax_thrust.plot(cmd_range, thrust_curve, color=C_BLUE, lw=2)
    ax_thrust.axhline(0, color=C_GRAY, lw=0.7, ls="--", alpha=0.5)
    ax_thrust.axvline(0, color=C_GRAY, lw=0.7, ls="--", alpha=0.5)
    ax_thrust.fill_between(cmd_range, thrust_curve, 0,
                           where=(thrust_curve >= 0), alpha=0.12, color=C_TEAL)
    ax_thrust.fill_between(cmd_range, thrust_curve, 0,
                           where=(thrust_curve < 0),  alpha=0.12, color=C_CORAL)
    ax_thrust.set_xlabel("cmd [-1, 1]", fontsize=10)
    ax_thrust.set_ylabel("Empuje [N]", fontsize=10)
    ax_thrust.set_title("Curva T200: cmd → Thrust", fontsize=11, fontweight="bold")
    ax_thrust.text(0.65, 33, f"máx fwd\n{36.38:.1f} N", fontsize=8, color=C_TEAL, ha="center")
    ax_thrust.text(-0.65, -24, f"máx rev\n{-28.44:.1f} N", fontsize=8, color=C_CORAL, ha="center")
    ax_thrust.grid(True, alpha=0.25)
    ax_thrust.set_facecolor("#FFFFFF")

    # ── (B) Vista superior del bote ───────────────────────────────────
    ax_boat = fig.add_subplot(gs[0, 1])
    ax_boat.set_aspect("equal")
    ax_boat.set_facecolor("#FFFFFF")
    ax_boat.set_xlim(-0.55, 0.55)
    ax_boat.set_ylim(-0.80, 0.75)

    # Casco (silueta simplificada)
    from matplotlib.patches import Polygon as MplPolygon
    hw = BOAT_WIDTH / 2
    hl = BOAT_LENGTH / 2
    hull_x = [hw*0.6, hw, hw, hw*0.6, 0, -hw*0.6, -hw, -hw, -hw*0.6]
    hull_y = [hl, hl*0.5, -hl*0.5, -hl, -hl*0.85, -hl, -hl*0.5, hl*0.5, hl]
    hull = MplPolygon(list(zip(hull_x, hull_y)),
                      closed=True, facecolor="#D3D1C7", edgecolor="#5F5E5A", lw=1.2, zorder=1)
    ax_boat.add_patch(hull)

    # Proa (triángulo)
    prow = MplPolygon([(0, hl*0.85), (-hw*0.6, hl), (hw*0.6, hl)],
                      closed=True, facecolor="#B4B2A9", edgecolor="#5F5E5A", lw=0.8, zorder=2)
    ax_boat.add_patch(prow)

    # CoM
    ax_boat.plot(0, 0, "k+", ms=10, mew=1.5, zorder=5)
    ax_boat.annotate("CoM", (0.03, -0.03), fontsize=8, color=C_GRAY)

    # Propulsores
    motor_colors = {"FR": C_TEAL, "FL": C_CORAL, "BR": C_TEAL, "BL": C_CORAL}
    motor_inv    = {"FR": False,   "FL": True,    "BR": False,  "BL": True }
    for name, cfg in MOTORS.items():
        xi, yi = cfg["pos"]
        color = motor_colors[name]
        circ = Circle((xi, yi), 0.055, facecolor=color, edgecolor="white",
                      lw=1, zorder=4, alpha=0.85)
        ax_boat.add_patch(circ)
        ax_boat.text(xi, yi, name, ha="center", va="center",
                     fontsize=8, fontweight="bold", color="white", zorder=5)
        tag = "inv" if motor_inv[name] else "nor"
        tag_color = C_CORAL if motor_inv[name] else C_TEAL
        ax_boat.text(xi + 0.01, yi - 0.10, tag, ha="center",
                     fontsize=7, color=tag_color)

    # Ejes del body frame
    ax_boat.annotate("", xy=(0.42, 0), xytext=(0, 0),
                     arrowprops=dict(arrowstyle="->", color=C_BLUE, lw=1.2))
    ax_boat.text(0.44, -0.02, "+x\n(sway)", fontsize=7, color=C_BLUE, ha="center")
    ax_boat.annotate("", xy=(0, 0.60), xytext=(0, 0),
                     arrowprops=dict(arrowstyle="->", color=C_BLUE, lw=1.2))
    ax_boat.text(0.06, 0.60, "+y\n(surge)", fontsize=7, color=C_BLUE)

    ax_boat.set_title("Planta del USV — posición motores", fontsize=11, fontweight="bold")
    ax_boat.set_xlabel("Lateral [m]", fontsize=9)
    ax_boat.set_ylabel("Longitudinal [m]", fontsize=9)
    ax_boat.grid(True, alpha=0.2)

    inv_patch  = mpatches.Patch(color=C_CORAL, label="Invertido (FL, BL)")
    nor_patch  = mpatches.Patch(color=C_TEAL,  label="Normal (FR, BR)")
    ax_boat.legend(handles=[nor_patch, inv_patch], fontsize=7,
                   loc="lower right", framealpha=0.8)

    # ── (C) Vector de fuerzas para el caso ejemplo ───────────────────
    ax_vec = fig.add_subplot(gs[0, 2])
    ax_vec.set_aspect("equal")
    ax_vec.set_facecolor("#FFFFFF")
    ax_vec.set_xlim(-0.55, 0.55)
    ax_vec.set_ylim(-0.80, 0.75)

    # Casco de fondo (translúcido)
    hull2 = MplPolygon(list(zip(hull_x, hull_y)),
                       closed=True, facecolor="#E8E8E4", edgecolor="#CCCCCA",
                       lw=1, zorder=1, alpha=0.6)
    ax_vec.add_patch(hull2)
    ax_vec.plot(0, 0, "k+", ms=8, mew=1.2, zorder=5)

    # Flechas por motor
    SCALE = 0.012   # m/N para la visualización
    for name, cfg in MOTORS.items():
        xi, yi = cfg["pos"]
        T_i = result["thrusts"][name]
        dy = T_i * SCALE  # fuerza en dirección surge (+y)
        color = motor_colors[name]
        if abs(T_i) > 0.01:
            ax_vec.annotate("", xy=(xi, yi + dy), xytext=(xi, yi),
                            arrowprops=dict(arrowstyle="-|>", color=color,
                                            lw=2, mutation_scale=12))
        ax_vec.text(xi, yi + dy + 0.05, f"{T_i:.1f}N",
                    ha="center", fontsize=7.5, color=color, fontweight="bold")
        circ2 = Circle((xi, yi), 0.045, facecolor=color, edgecolor="white",
                       lw=1, zorder=4, alpha=0.7)
        ax_vec.add_patch(circ2)
        ax_vec.text(xi, yi, name, ha="center", va="center",
                    fontsize=7.5, fontweight="bold", color="white", zorder=5)

    # Resultante total
    T_net = result["T_u"]
    dy_net = T_net * SCALE
    ax_vec.annotate("", xy=(0, dy_net), xytext=(0, 0),
                    arrowprops=dict(arrowstyle="-|>", color="black",
                                   lw=2.5, mutation_scale=15))
    ax_vec.text(0.1, dy_net / 2, f"T_u={T_net:.1f} N",
                fontsize=8.5, color="black", fontweight="bold")

    ax_vec.set_title(
        f"Vectores de fuerza (cmd=0.6)\nT_r = {result['T_r']:.3f} N·m",
        fontsize=11, fontweight="bold"
    )
    ax_vec.set_xlabel("Lateral [m]", fontsize=9)
    ax_vec.set_ylabel("Longitudinal [m]", fontsize=9)
    ax_vec.grid(True, alpha=0.2)

    # ── (D) T_u vs cmd — avance simétrico ────────────────────────────
    ax_tu = fig.add_subplot(gs[1, 0])
    ax_tu.plot(cmd_range, Tu_sym, color=C_BLUE, lw=2, label="avance simétrico")
    ax_tu.axhline(0, color=C_GRAY, lw=0.7, ls="--", alpha=0.5)
    ax_tu.axvline(0, color=C_GRAY, lw=0.7, ls="--", alpha=0.5)
    ax_tu.fill_between(cmd_range, Tu_sym, 0,
                       where=(Tu_sym >= 0), alpha=0.10, color=C_TEAL)
    ax_tu.fill_between(cmd_range, Tu_sym, 0,
                       where=(Tu_sym < 0),  alpha=0.10, color=C_CORAL)
    ax_tu.set_xlabel("cmd [-1, 1]", fontsize=10)
    ax_tu.set_ylabel("T_u [N]", fontsize=10)
    ax_tu.set_title("Surge force vs cmd\n(4 motores iguales)", fontsize=11, fontweight="bold")
    ax_tu.text(0.02, 0.94, f"máx: {Tu_sym.max():.1f} N",
               transform=ax_tu.transAxes, fontsize=8, color=C_TEAL)
    ax_tu.text(0.02, 0.86, f"mín: {Tu_sym.min():.1f} N",
               transform=ax_tu.transAxes, fontsize=8, color=C_CORAL)
    ax_tu.grid(True, alpha=0.25)
    ax_tu.set_facecolor("#FFFFFF")

    # ── (E) T_r vs cmd — giro diferencial ────────────────────────────
    ax_tr = fig.add_subplot(gs[1, 1])
    ax_tr.plot(cmd_range, Tr_turn, color=C_PURPLE, lw=2, label="diferencial puro")
    ax_tr.axhline(0, color=C_GRAY, lw=0.7, ls="--", alpha=0.5)
    ax_tr.axvline(0, color=C_GRAY, lw=0.7, ls="--", alpha=0.5)
    ax_tr.fill_between(cmd_range, Tr_turn, 0,
                       where=(Tr_turn >= 0), alpha=0.10, color=C_PURPLE)
    ax_tr.fill_between(cmd_range, Tr_turn, 0,
                       where=(Tr_turn < 0),  alpha=0.10, color=C_AMBER)
    ax_tr.set_xlabel("cmd derecha = +c, izquierda = −c", fontsize=9)
    ax_tr.set_ylabel("T_r [N·m]", fontsize=10)
    ax_tr.set_title("Yaw torque vs cmd\n(diferencial puro)", fontsize=11, fontweight="bold")
    ax_tr.text(0.02, 0.94, f"máx: {Tr_turn.max():.1f} N·m",
               transform=ax_tr.transAxes, fontsize=8, color=C_PURPLE)
    ax_tr.text(0.02, 0.86, f"mín: {Tr_turn.min():.1f} N·m",
               transform=ax_tr.transAxes, fontsize=8, color=C_AMBER)
    ax_tr.grid(True, alpha=0.25)
    ax_tr.set_facecolor("#FFFFFF")

    # ── (F) Asimetría frente/popa ────────────────────────────────────
    ax_asym = fig.add_subplot(gs[1, 2])
    ax_asym.plot(cmd_range, Tr_asym, color=C_AMBER, lw=2, label="asim. front/back")
    ax_asym.plot(cmd_range, Tu_asym, color=C_GREEN, lw=2, label="Tu asim.", ls="--")
    ax_asym.axhline(0, color=C_GRAY, lw=0.7, ls="--", alpha=0.5)
    ax_asym.axvline(0, color=C_GRAY, lw=0.7, ls="--", alpha=0.5)
    ax_asym.set_xlabel("cmd frente=+c, popa=−c", fontsize=9)
    ax_asym.set_ylabel("Fuerza / Torque", fontsize=10)
    ax_asym.set_title("Asimetría longitudinal\n(frente vs popa)", fontsize=11, fontweight="bold")
    ax_asym.legend(fontsize=8)
    ax_asym.grid(True, alpha=0.25)
    ax_asym.set_facecolor("#FFFFFF")

    # ── (G) Mapa de calor: T_u(cmd_left, cmd_right) ─────────────────
    ax_hm_u = fig.add_subplot(gs[2, 0])
    cr = np.linspace(-1, 1, 60)
    CL, CR = np.meshgrid(cr, cr)
    TU_MAP = np.zeros_like(CL)
    for i in range(len(cr)):
        for j in range(len(cr)):
            cl_val = cr[i]
            cr_val = cr[j]
            r = compute_tau({"FR": cr_val, "FL": cl_val,
                             "BR": cr_val, "BL": cl_val})
            TU_MAP[i, j] = r["T_u"]

    im1 = ax_hm_u.pcolormesh(CR, CL, TU_MAP, cmap="RdBu_r", shading="auto")
    fig.colorbar(im1, ax=ax_hm_u, label="T_u [N]", fraction=0.04, pad=0.04)
    ax_hm_u.set_xlabel("cmd derecha", fontsize=9)
    ax_hm_u.set_ylabel("cmd izquierda", fontsize=9)
    ax_hm_u.set_title("Mapa T_u(cmd_L, cmd_R)", fontsize=11, fontweight="bold")
    ax_hm_u.set_facecolor("#FFFFFF")

    # ── (H) Mapa de calor: T_r(cmd_left, cmd_right) ─────────────────
    ax_hm_r = fig.add_subplot(gs[2, 1])
    TR_MAP = np.zeros_like(CL)
    for i in range(len(cr)):
        for j in range(len(cr)):
            cl_val = cr[i]
            cr_val = cr[j]
            r = compute_tau({"FR": cr_val, "FL": cl_val,
                             "BR": cr_val, "BL": cl_val})
            TR_MAP[i, j] = r["T_r"]

    im2 = ax_hm_r.pcolormesh(CR, CL, TR_MAP, cmap="PuOr", shading="auto")
    fig.colorbar(im2, ax=ax_hm_r, label="T_r [N·m]", fraction=0.04, pad=0.04)
    ax_hm_r.set_xlabel("cmd derecha", fontsize=9)
    ax_hm_r.set_ylabel("cmd izquierda", fontsize=9)
    ax_hm_r.set_title("Mapa T_r(cmd_L, cmd_R)", fontsize=11, fontweight="bold")
    ax_hm_r.set_facecolor("#FFFFFF")

    # ── (I) Tabla resumen de casos ───────────────────────────────────
    ax_tab = fig.add_subplot(gs[2, 2])
    ax_tab.axis("off")

    test_cases = [
        ("Avance máx",     {"FR":1.0,"FL":1.0,"BR":1.0,"BL":1.0}),
        ("Retro máx",      {"FR":-1.0,"FL":-1.0,"BR":-1.0,"BL":-1.0}),
        ("Giro CW máx",    {"FR":1.0,"FL":-1.0,"BR":1.0,"BL":-1.0}),
        ("Giro CCW máx",   {"FR":-1.0,"FL":1.0,"BR":-1.0,"BL":1.0}),
        ("Avance 60%",     {"FR":0.6,"FL":0.6,"BR":0.6,"BL":0.6}),
        ("Giro CW 50%",    {"FR":0.5,"FL":-0.5,"BR":0.5,"BL":-0.5}),
        ("Solo FR",        {"FR":1.0,"FL":0.0,"BR":0.0,"BL":0.0}),
        ("Solo FL",        {"FR":0.0,"FL":1.0,"BR":0.0,"BL":0.0}),
    ]

    col_labels = ["Caso", "T_u [N]", "T_v [N]", "T_r [N·m]"]
    table_data = []
    for label, cmds_tc in test_cases:
        r = compute_tau(cmds_tc)
        table_data.append([label,
                           f"{r['T_u']:+.2f}",
                           f"{r['T_v']:+.2f}",
                           f"{r['T_r']:+.2f}"])

    tbl = ax_tab.table(
        cellText=table_data,
        colLabels=col_labels,
        loc="center",
        cellLoc="center"
    )
    tbl.auto_set_font_size(False)
    tbl.set_fontsize(8.5)
    tbl.scale(1.0, 1.45)

    # Estilo de cabecera
    for j in range(len(col_labels)):
        tbl[0, j].set_facecolor("#D3D1C7")
        tbl[0, j].set_text_props(fontweight="bold")

    # Colorear T_r
    for i in range(1, len(table_data) + 1):
        val = float(table_data[i-1][3])
        if val > 0.5:
            tbl[i, 3].set_facecolor("#E6F1FB")
        elif val < -0.5:
            tbl[i, 3].set_facecolor("#FAECE7")

    ax_tab.set_title("Casos de referencia", fontsize=11, fontweight="bold", pad=12)

    # ── Título global ─────────────────────────────────────────────────
    fig.suptitle(
        "USV — Modelo de fuerzas T200 · Propulsión diferencial 4 motores",
        fontsize=14, fontweight="bold", y=0.97, color="#2C2C2A"
    )

    plt.savefig("usv_forces.png",
                dpi=150, bbox_inches="tight", facecolor=fig.get_facecolor())
    plt.show()
    print("\nFigura guardada en: usv_forces.png")