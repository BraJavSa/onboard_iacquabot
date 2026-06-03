import numpy as np
import matplotlib.pyplot as plt
from scipy.io import loadmat

# ─── Modelo de empuje T200 ────────────────────────────────────────────────────
T200 = {
    "pos": dict(A=0.000001, K=40.0209, B=2.6249, v=0.1615, C=0.9432, M=0.00001),
    "neg": dict(A=-31.4990, K=-0.00001, B=3.6986, v=0.3264, C=0.9713, M=-1.0000),
}
MAX_FWD =  36.3827
MAX_REV = -28.4393

def cmd_to_thrust(cmd):
    """cmd ∈ [-1, 1] → fuerza [N], con saturación."""
    cmd = np.asarray(cmd, dtype=float)
    T = np.zeros_like(cmd)
    pos = cmd >  0.01
    neg = cmd < -0.01
    p = T200["pos"]
    T[pos] = p["A"] + (p["K"] - p["A"]) / (
        p["C"] + np.exp(-p["B"] * (cmd[pos] - p["M"]))
    ) ** (1.0 / p["v"])
    n = T200["neg"]
    T[neg] = n["A"] + (n["K"] - n["A"]) / (
        n["C"] + np.exp(-n["B"] * (cmd[neg] - n["M"]))
    ) ** (1.0 / n["v"])
    return np.clip(T, MAX_REV, MAX_FWD)

PWM_MID = 1500
PWM_MAX = 1900
MOTOR_NAMES    = ["BL",  "FL",  "BR",  "FR"]
MOTOR_INVERTED = [True,  True,  False, False]

def pwm_to_cmd(pwm_us, inverted=False):
    cmd = (np.asarray(pwm_us, dtype=float) - PWM_MID) / (PWM_MAX - PWM_MID)
    return -cmd if inverted else cmd

B = np.array([
    [ 1,      1,      1,      1    ],  # tau_u
    [ 0,      0,      0,      0    ],  # tau_v
    [ -0.30,  0.30,   -0.30,  0.30 ],  # tau_r = x_i * T_i
])

# ─── Cargar datos ─────────────────────────────────────────────────────────────
D = loadmat("experiment_10.mat")
t   = D["t"].squeeze()
yaw = D["yaw"].squeeze()
vx  = D["vx"].squeeze()
vy  = D["vy"].squeeze()

pwm_raw = [
    D["pwm1"].squeeze(),
    D["pwm2"].squeeze(),
    D["pwm3"].squeeze(),   # BR — normal
    D["pwm4"].squeeze(),   # FR — normal
]

# ─── Velocidades en body frame ────────────────────────────────────────────────
# MAVROS publica el twist del odómetro en child_frame_id = base_link,
# es decir, vx y vy YA están en el body frame (surge/sway).
# NO se debe aplicar rotación adicional con yaw.
u = vx                  # surge   [m/s]  — directamente de base_link
v = vy                  # sway    [m/s]  — directamente de base_link
r = D["wz"].squeeze()   # yaw rate [rad/s]

# ─── Pipeline: PWM → cmd → thrust → τ ────────────────────────────────────────
cmd    = [pwm_to_cmd(pwm_raw[i], MOTOR_INVERTED[i]) for i in range(4)]
thrust = [cmd_to_thrust(cmd[i])                     for i in range(4)]

# T_mat columnas en orden FR, FL, BR, BL  →  índices 3, 1, 2, 0
T_mat = np.vstack([thrust[3], thrust[1], thrust[2], thrust[0]])  # (4, N)
tau   = B @ T_mat                                                  # (3, N)

tau_u = tau[0]
tau_r = tau[2]

# ─── Plots ────────────────────────────────────────────────────────────────────
fig, axes = plt.subplots(5, 1, figsize=(13, 14), sharex=True)

axes[0].plot(t, tau_u, color="steelblue", lw=1.5)
axes[0].axhline(0, color='k', lw=0.5, ls='--')
axes[0].set_ylabel("[N]")
axes[0].set_title(r"$\tau_u$ — surge force")
axes[0].grid(True)

axes[1].plot(t, tau_r, color="firebrick", lw=1.5)
axes[1].axhline(0, color='k', lw=0.5, ls='--')
axes[1].set_ylabel("[N·m]")
axes[1].set_title(r"$\tau_r$ — yaw torque")
axes[1].grid(True)

axes[2].plot(t, u, color="steelblue", lw=1.5)
axes[2].axhline(0, color='k', lw=0.5, ls='--')
axes[2].set_ylabel("[m/s]")
axes[2].set_title("u — surge velocity (body frame, directo de base_link)")
axes[2].grid(True)

axes[3].plot(t, v, color="darkorange", lw=1.5)
axes[3].axhline(0, color='k', lw=0.5, ls='--')
axes[3].set_ylabel("[m/s]")
axes[3].set_title("v — sway velocity (body frame, directo de base_link)")
axes[3].grid(True)

axes[4].plot(t, r, color="seagreen", lw=1.5)
axes[4].axhline(0, color='k', lw=0.5, ls='--')
axes[4].set_ylabel("[rad/s]")
axes[4].set_xlabel("Time [s]")
axes[4].set_title("r — yaw rate")
axes[4].grid(True)

plt.tight_layout()
plt.show()