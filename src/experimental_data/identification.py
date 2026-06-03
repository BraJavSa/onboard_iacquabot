import numpy as np
import matplotlib.pyplot as plt
from scipy.io import loadmat
from scipy.integrate import solve_ivp
from scipy.interpolate import interp1d

# ──────────────────────────────────────────────────────────────
# T200
# ──────────────────────────────────────────────────────────────

T200 = {
    "pos": dict(A=0.000001, K=40.0209, B=2.6249, v=0.1615, C=0.9432, M=0.00001),
    "neg": dict(A=-31.4990, K=-0.00001, B=3.6986, v=0.3264, C=0.9713, M=-1.0000),
}

MAX_FWD = 36.3827
MAX_REV = -28.4393

def cmd_to_thrust(cmd):
    cmd = np.asarray(cmd, dtype=float)

    T = np.zeros_like(cmd)

    pos = cmd > 0.01
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

MOTOR_INVERTED = [True, True, False, False]

def pwm_to_cmd(pwm_us, inverted=False):
    cmd = (np.asarray(pwm_us) - PWM_MID) / (PWM_MAX - PWM_MID)
    return -cmd if inverted else cmd

# FR FL BR BL
B_alloc = np.array([
    [ 1.0,  1.0,  1.0,  1.0],
    [ 0.0,  0.0,  0.0,  0.0],
    [-0.5, 0.5,-0.5, 0.5]
])

# ──────────────────────────────────────────────────────────────
# CARGA DE DATOS
# ──────────────────────────────────────────────────────────────

def load_dataset(filename):
    D = loadmat(filename)
    
    t = D["t"].squeeze()
    u = D["vx"].squeeze()
    v = D["vy"].squeeze()
    r = D["wz"].squeeze()
    
    pwm_raw = [D[f"pwm{i+1}"].squeeze() for i in range(4)]
    cmd = [pwm_to_cmd(pwm_raw[i], MOTOR_INVERTED[i]) for i in range(4)]
    thrust = [cmd_to_thrust(cmd[i]) for i in range(4)]
    
    Tmat = np.vstack([
        thrust[3],  # FR
        thrust[1],  # FL
        thrust[2],  # BR
        thrust[0],  # BL
    ])
    
    tau = B_alloc @ Tmat
    tau_u = tau[0]
    tau_r = tau[2]
    
    return t, u, v, r, tau_u, tau_r

# Cargar datos para IDENTIFICACIÓN GLOBAL
import os

train_files = [f"experiment_{i:02d}.mat" for i in range(1, 11)]
datasets = []

for f in train_files:
    if os.path.exists(f):
        t, u, v, r, tau_u, tau_r = load_dataset(f)
        tau_u_fn = interp1d(t, tau_u, bounds_error=False, fill_value="extrapolate")
        tau_r_fn = interp1d(t, tau_r, bounds_error=False, fill_value="extrapolate")
        datasets.append({
            "t": t, "u": u, "v": v, "r": r,
            "tau_u_fn": tau_u_fn, "tau_r_fn": tau_r_fn
        })

print(f"Cargados {len(datasets)} datasets para entrenamiento global.")

from scipy.signal import savgol_filter
from scipy.optimize import minimize

# ──────────────────────────────────────────────────────────────
# IDENTIFICACIÓN FOSSEN 3DOF (OUTPUT ERROR / GLOBAL COUPLED)
# ──────────────────────────────────────────────────────────────
# Se identifica el modelo globalmente minimizando el error 
# a través de todos los experimentos simultáneamente.

p0 = [100.0, 80.0, 18.0, 35.0, 60.0, 80.0, 0.0, 30.0, 15.0]

def cost_function(p):
    m11, m22, m33, Xu, Xuu, Yv, Yvv, Nr, Nrr = p
    
    # Penalizar parámetros no físicos (masas negativas o amortiguamientos negativos)
    if any(x < 1.0 for x in [m11, m22, m33, Xu, Yv, Nr]):
        return 1e6 + sum([max(0, 1.0 - x)**2 for x in p]) * 1000

    total_error = 0.0
    
    for ds in datasets:
        def ode(ts, x):
            us, vs, rs = x
            Tu = float(ds["tau_u_fn"](ts))
            Tr = float(ds["tau_r_fn"](ts))
            
            # Modelo acoplado de Fossen 3DOF
            du_s = (Tu + m22*vs*rs - Xu*us - Xuu*abs(us)*us) / m11
            dv_s = (-m11*us*rs - Yv*vs - Yvv*abs(vs)*vs) / m22
            dr_s = (Tr + (m11 - m22)*us*vs - Nr*rs - Nrr*abs(rs)*rs) / m33
            return [du_s, dv_s, dr_s]
        
        t_ds = ds["t"]
        sol = solve_ivp(ode, [t_ds[0], t_ds[-1]], [ds["u"][0], ds["v"][0], ds["r"][0]], t_eval=t_ds, method='RK23', rtol=1e-3, atol=1e-3)
        
        if not sol.success or sol.y.shape[1] != len(t_ds):
            return 1e6
        
        u_sim, v_sim, r_sim = sol.y
        
        # Normalizar los errores para este dataset
        e_u = np.mean((ds["u"] - u_sim)**2) / (np.var(ds["u"]) + 1e-6)
        e_v = np.mean((ds["v"] - v_sim)**2) / (np.var(ds["v"]) + 1e-6)
        e_r = np.mean((ds["r"] - r_sim)**2) / (np.var(ds["r"]) + 1e-6)
        
        total_error += e_u + 2.0 * e_v + e_r
        
    return total_error / len(datasets)

print("Iniciando identificación acoplada global (puede tardar unos segundos)...")
res = minimize(cost_function, p0, method='Nelder-Mead', options={'maxiter': 150, 'disp': False})

mXu, mYv, IzNr, Xu, Xuu, Yv, Yvv, Nr, Nrr = res.x

print()
print("="*70)
print("PARAMETROS IDENTIFICADOS (MODELO ACOPLADO)")
print("="*70)

print(f"m-Xudot  = {mXu:.6f}")
print(f"m-Yvdot  = {mYv:.6f}")
print(f"Iz-Nrdot = {IzNr:.6f}")

print(f"Xu       = {Xu:.6f}")
print(f"Xuu      = {Xuu:.6f}")

print(f"Yv       = {Yv:.6f}")
print(f"Yvv      = {Yvv:.6f}")

print(f"Nr       = {Nr:.6f}")
print(f"Nrr      = {Nrr:.6f}")

# ──────────────────────────────────────────────────────────────
# VALIDACIÓN DINÁMICA CON DATOS DE PRUEBA
# ──────────────────────────────────────────────────────────────

# Cargar datos para VALIDACIÓN (escogemos el 10 como representante)
t_val, u_val, v_val, r_val, tau_u_val, tau_r_val = load_dataset("experiment_10.mat")

print(f"\nDataset de Validación: experiment_10.mat")
print(f"Samples = {len(t_val)}")
print(f"Duration = {t_val[-1]-t_val[0]:.2f} s")

tau_u_val_fn = interp1d(t_val, tau_u_val, bounds_error=False, fill_value="extrapolate")
tau_r_val_fn = interp1d(t_val, tau_r_val, bounds_error=False, fill_value="extrapolate")

def dynamics_val(tt, x):
    u_, v_, r_ = x
    tauu = float(tau_u_val_fn(tt))
    taur = float(tau_r_val_fn(tt))

    du_ = (tauu + mYv*v_*r_ - Xu*u_ - Xuu*abs(u_)*u_) / mXu
    dv_ = (-mXu*u_*r_ - Yv*v_ - Yvv*abs(v_)*v_) / mYv
    dr_ = (taur + (mXu - mYv)*u_*v_ - Nr*r_ - Nrr*abs(r_)*r_) / IzNr

    return [du_, dv_, dr_]

sol = solve_ivp(
    dynamics_val,
    [t_val[0], t_val[-1]],
    [u_val[0], v_val[0], r_val[0]],
    t_eval=t_val,
    method='RK45',
    rtol=1e-5,
    atol=1e-5
)

u_hat = sol.y[0]
v_hat = sol.y[1]
r_hat = sol.y[2]

# ──────────────────────────────────────────────────────────────
# R²
# ──────────────────────────────────────────────────────────────

def r2(y, yh):
    return 1 - np.sum((y-yh)**2)/(np.sum((y-np.mean(y))**2) + 1e-8)

print()
print("="*70)
print("RESULTADOS DE VALIDACION")
print("="*70)

print(f"R2 surge = {r2(u_val,u_hat):.4f}")
print(f"R2 sway  = {r2(v_val,v_hat):.4f}")
print(f"R2 yaw   = {r2(r_val,r_hat):.4f}")
print(f"max |v|  = {np.max(np.abs(v_val)):.6f}")
print(f"std v    = {np.std(v_val):.6f}")

# ──────────────────────────────────────────────────────────────
# PLOTS
# ──────────────────────────────────────────────────────────────

fig, ax = plt.subplots(3,1, figsize=(14,10), sharex=True)
fig.suptitle("Entrenado con Todos (01-10), Validando en exp 10", fontsize=14, fontweight='bold')

ax[0].plot(t_val, u_val, label="u real")
ax[0].plot(t_val, u_hat, "--", label="u modelo")
ax[0].grid()
ax[0].legend()
ax[0].set_ylabel("m/s")
ax[0].set_title(f"Surge (u) - R2: {r2(u_val,u_hat):.4f}")

ax[1].plot(t_val, v_val, label="v real")
ax[1].plot(t_val, v_hat, "--", label="v modelo")
ax[1].grid()
ax[1].legend()
ax[1].set_ylabel("m/s")
ax[1].set_title(f"Sway (v) - R2: {r2(v_val,v_hat):.4f}")

ax[2].plot(t_val, r_val, label="r real")
ax[2].plot(t_val, r_hat, "--", label="r modelo")
ax[2].grid()
ax[2].legend()
ax[2].set_ylabel("rad/s")
ax[2].set_xlabel("Tiempo [s]")
ax[2].set_title(f"Yaw (r) - R2: {r2(r_val,r_hat):.4f}")

plt.tight_layout()
plt.show()