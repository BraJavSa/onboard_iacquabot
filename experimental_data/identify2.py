#!/usr/bin/env python3
import os
import numpy as np
from scipy.io import loadmat
from scipy.signal import savgol_filter
from scipy.integrate import solve_ivp
from scipy.interpolate import interp1d
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

# =============================================================================
# 1. PARÁMETROS FIJOS DEL MOTOR (Para calcular Tu y Tr a partir de PWM)
# =============================================================================
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

# pwms list in pwm_to_thrust_forces is [pwm1, pwm2, pwm3, pwm4] which corresponds to [BL, FL, BR, FR].
# So T will be computed in [BL, FL, BR, FR] order.
# We map y_coords to match this order:
# BL -> positions_yx[3][0]
# FL -> positions_yx[1][0]
# BR -> positions_yx[2][0]
# FR -> positions_yx[0][0]
y_coords = [
    MOTOR_TEMPLATE["positions_yx"][3][0],  # BL (0.35)
    MOTOR_TEMPLATE["positions_yx"][1][0],  # FL (0.35)
    MOTOR_TEMPLATE["positions_yx"][2][0],  # BR (-0.35)
    MOTOR_TEMPLATE["positions_yx"][0][0],  # FR (-0.35)
]

def pwm_to_thrust_forces(pwm1, pwm2, pwm3, pwm4):
    pwms = [pwm1, pwm2, pwm3, pwm4]
    T = np.zeros(4)
    params = MOTOR_TEMPLATE["T200"]
    
    for i in range(4):
        s_i = -1.0 if MOTOR_TEMPLATE["motor_inverted"][i] else 1.0
        cmd = s_i * (pwms[i] - MOTOR_TEMPLATE["pwm_mid"]) / (MOTOR_TEMPLATE["pwm_max"] - MOTOR_TEMPLATE["pwm_mid"])
        
        if abs(cmd) <= 0.01:
            T[i] = 0.0
        elif cmd > 0.01:
            p = params["pos"]
            T[i] = p["A"] + (p["K"] - p["A"]) / ((p["C"] + np.exp(-p["B"] * (cmd - p["M"])))**(1.0 / p["v"]))
        else:
            p = params["neg"]
            T[i] = p["A"] + (p["K"] - p["A"]) / ((p["C"] + np.exp(-p["B"] * (cmd - p["M"])))**(1.0 / p["v"]))
            
        # Satura el empuje a [max_rev, max_fwd]
        T[i] = np.clip(T[i], MOTOR_TEMPLATE["max_rev"], MOTOR_TEMPLATE["max_fwd"])
            
    Tu = np.sum(T)
    # Momento en r: tau_r = sum(y_i * T_i) para coincidir con la convención FRD/NED de wz (r)
    Tr = np.sum([y * t for y, t in zip(y_coords, T)])
    return Tu, Tr

# =============================================================================
# 2. CONSTRUCCIÓN DE LA REGRESIÓN GLOBAL (Idéntico a tu script de MATLAB)
# =============================================================================
def build_global_phi_tau(acc_u, acc_v, acc_r, u, v, r, Tu, Tr):
    N = len(u)
    Phi_list = []
    Tau_list = []
    
    for k in range(N):
        # ---- Fila Surge (Ecuación 1) ----
        row_u = np.zeros(12)
        row_u[0] = acc_u[k]          # m - Xdu
        row_u[1] = -v[k] * r[k]      # m - Ydv
        row_u[3] = u[k]              # Xu
        row_u[4] = abs(u[k]) * u[k]  # Xuu
        Phi_list.append(row_u)
        Tau_list.append(Tu[k])
        
        # ---- Fila Sway (Ecuación 2) ----
        row_v = np.zeros(12)
        row_v[0] = 0.0               # Se elimina el término Coriolis separado para evitar colinealidad exacta con Yur
        row_v[1] = acc_v[k]          # m - Ydv (Inercia)
        row_v[5] = v[k]              # Yv (Damp lineal)
        row_v[6] = abs(v[k]) * v[k]  # Yvv (Damp cuadrático)
        row_v[9] = r[k]              # Yr (Coupling lineal yaw)
        row_v[10] = abs(r[k]) * r[k] # Yrr (Coupling cuadrático yaw)
        row_v[11] = u[k] * r[k]      # Yur (Coupling neto que engloba Coriolis y Yur)
        Phi_list.append(row_v)
        Tau_list.append(0.0)         # Tv es fijos en 0 por diseño
        
        # ---- Fila Yaw (Ecuación 3) ----
        row_r = np.zeros(12)
        row_r[2] = acc_r[k]          # Iz - Ndr (Inercia)
        row_r[7] = r[k]              # Nr (Damp lineal)
        row_r[8] = abs(r[k]) * r[k]  # Nrr (Damp cuadrático)
        Phi_list.append(row_r)
        Tau_list.append(Tr[k])
        
    return np.array(Phi_list), np.array(Tau_list)

# =============================================================================
# 2.5. SIMULACIÓN Y EVALUACIÓN DE MÉTRICAS DE ERROR
# =============================================================================
def simulate_vessel(t, u, v, r, Tu, Tr, theta):
    # theta: [m-Xdu, m-Ydv, Iz-Ndr, Xu, Xuu, Yv, Yvv, Nr, Nrr, Yr, Yrr, Yur]
    m11, m22, m33, Xu, Xuu, Yv, Yvv, Nr, Nrr, Yr, Yrr, Yur = theta
    
    # Interpoladores para fuerzas Tu y torque Tr de entrada
    Tu_fn = interp1d(t, Tu, bounds_error=False, fill_value="extrapolate")
    Tr_fn = interp1d(t, Tr, bounds_error=False, fill_value="extrapolate")
    
    def ode_system(tt, state):
        u_, v_, r_ = state
        tau_u = float(Tu_fn(tt))
        tau_r = float(Tr_fn(tt))
        
        # Modelo 3DOF Fossen acoplado con términos de acoplamiento en sway (v)
        du = (tau_u + m22 * v_ * r_ - Xu * u_ - Xuu * abs(u_) * u_) / m11
        dv = (-Yv * v_ - Yvv * abs(v_) * v_ - Yr * r_ - Yrr * abs(r_) * r_ - Yur * u_ * r_) / m22
        dr = (tau_r - Nr * r_ - Nrr * abs(r_) * r_) / m33
        
        return [du, dv, dr]
        
    sol = solve_ivp(
        ode_system,
        [t[0], t[-1]],
        [u[0], v[0], r[0]],
        t_eval=t,
        method="RK45",
        rtol=1e-5,
        atol=1e-5
    )
    return sol.y[0], sol.y[1], sol.y[2]

def compute_metrics(y_real, y_sim):
    rms_val = float(np.sqrt(np.mean((y_real - y_sim)**2)))
    r2_val = float(1.0 - np.sum((y_real - y_sim)**2) / (np.sum((y_real - np.mean(y_real))**2) + 1e-8))
    return rms_val, r2_val

# =============================================================================
# 3. PIPELINE PRINCIPAL DE IDENTIFICACIÓN
# =============================================================================
def main():
    FS = 30.0
    dt = 1.0 / FS
    
    # Cargar los 10 experimentos completos (.mat en la carpeta actual)
    valid_mats = sorted([f for f in os.listdir(".") if f.startswith("experiment_") and f.endswith(".mat")])
    
    print(f"Archivos .mat cargados para la regresión conjunta: {valid_mats}")
    if not valid_mats:
        print("No se encontraron archivos .mat de experimentos.")
        return

    Phi_global = []
    Tau_global = []
    experiments_data = []
    
    print("\n--- ANALIZANDO RANGO DINÁMICO NATIVO (FLU) ---")
    print(f"{'Experimento':<18} | {'Max |u|':<10} | {'Max |v|':<10} | {'Max |r|':<10} | {'Max |Tr|':<10}")
    print("-" * 65)

    for mat_file in valid_mats:
        try:
            mat_data = loadmat(mat_file)
            
            # Al estar en FLU, vx=u, vy=v, wz=r de manera directa
            u_raw = mat_data["vx"].flatten()
            v_raw = mat_data["vy"].flatten()
            r_raw = mat_data["wz"].flatten()
            
            # Filtro Savitzky-Golay para limpiar el ruido del sensor a 30Hz sin desfasar el tiempo
            # Ajustamos una ventana de 25 muestras y polinomio de grado 2
            u = savgol_filter(u_raw, 25, 2)
            v = savgol_filter(v_raw, 25, 2)
            r = savgol_filter(r_raw, 25, 2)
            
            # Calcular aceleraciones numéricas a partir de las señales filtradas
            acc_u = np.gradient(u, dt)
            acc_v = np.gradient(v, dt)
            acc_r = np.gradient(r, dt)
            
            # Reconstruir las fuerzas reales Tu y Tr generadas por tus motores
            pwm1 = mat_data["pwm1"].flatten()
            pwm2 = mat_data["pwm2"].flatten()
            pwm3 = mat_data["pwm3"].flatten()
            pwm4 = mat_data["pwm4"].flatten()
            
            N_samples = len(u)
            Tu_arr = np.zeros(N_samples)
            Tr_arr = np.zeros(N_samples)
            
            for i in range(N_samples):
                Tu_arr[i], Tr_arr[i] = pwm_to_thrust_forces(pwm1[i], pwm2[i], pwm3[i], pwm4[i])
            
            print(f"{mat_file:<18} | {np.max(np.abs(u)):.4f}     | {np.max(np.abs(v)):.4f}     | {np.max(np.abs(r)):.4f}     | {np.max(np.abs(Tr_arr)):.4f}")

            # Construir el regresor acoplado para este experimento
            Phi_exp, Tau_exp = build_global_phi_tau(acc_u, acc_v, acc_r, u, v, r, Tu_arr, Tr_arr)
            
            # Excluimos experiment_01.mat de la regresión conjunta por ser solo ruido
            if mat_file != "experiment_01.mat":
                Phi_global.append(Phi_exp)
                Tau_global.append(Tau_exp)
            
            # Guardar datos para posterior simulación y graficación
            t_arr = mat_data["t"].flatten() if "t" in mat_data else np.arange(N_samples) * dt
            experiments_data.append({
                "name": mat_file,
                "t": t_arr,
                "u_raw": u_raw,
                "v_raw": v_raw,
                "r_raw": r_raw,
                "u_filt": u,
                "v_filt": v,
                "r_filt": r,
                "Tu": Tu_arr,
                "Tr": Tr_arr
            })
            
        except Exception as e:
            print(f"Error procesando el archivo {mat_file}: {e}")
            continue

    # Unificar todas las muestras de los experimentos en la matriz global
    Phi_global = np.vstack(Phi_global)
    Tau_global = np.concatenate(Tau_global)
    
    print(f"\nTamaño de la matriz de Regresión Global Phi (excluyendo exp_01): {Phi_global.shape}")
    print("--- Ejecutando Mínimos Cuadrados Globales (OLS) ---")
    
    # Inversión de matriz OLS (theta = Phi \ Tau)
    theta_ols, _, _, _ = np.linalg.lstsq(Phi_global, Tau_global, rcond=None)
    
    param_names = ['m-Xdu', 'm-Ydv', 'Iz-Ndr', 'Xu', 'Xuu', 'Yv', 'Yvv', 'Nr', 'Nrr', 'Yr', 'Yrr', 'Yur']
    
    print("\n================ PARÁMETROS IDENTIFICADOS (OLS UNCONSTRAINED) ================")
    for i, name in enumerate(param_names):
        print(f" {name:<8}: {theta_ols[i]:.6f}")
    print("==============================================================================")
    
    print("\n--- Ejecutando Mínimos Cuadrados con Restricciones Físicas (lsq_linear) ---")
    from scipy.optimize import lsq_linear
    
    # Límites físicos para evitar inestabilidad (m11, m22, m33, Xu, Yv, Nr >= 1.0; Xuu, Yvv, Nrr >= 0.0)
    # Yr, Yrr, Yur son acoplamientos y pueden ser positivos o negativos.
    # Usamos cotas realistas para un bote de ~30kg físicos.
    lb = [20.0, 30.0, 5.0, 1.0, 0.0, 50.0, 0.0, 1.0, 0.0, -500.0, -200.0, -500.0]
    ub = [150.0, 150.0, 50.0, 150.0, 150.0, 300.0, 100.0, 150.0, 100.0, 500.0, 200.0, 500.0]
    
    res = lsq_linear(Phi_global, Tau_global, bounds=(lb, ub))
    theta = res.x
    
    print("\n================ PARÁMETROS IDENTIFICADOS FINALES (BOUNDED LSTSQ) ================")
    for i, name in enumerate(param_names):
        print(f" {name:<8}: {theta[i]:.6f}")
    print("==================================================================================")
    
    # Guardar en archivo python independiente
    output_py_file = "wamv_identified_params.py"
    with open(output_py_file, "w") as f:
        f.write("# Parámetros de Fossen identificados globalmente con restricciones de estabilidad, velocidad y acoplamiento en sway\n")
        for i, name in enumerate(param_names):
            clean_name = name.replace("-", "")
            f.write(f"{clean_name} = {theta[i]:.10f};\n")
        f.write(f"\ntheta = {list(theta)}\n")
        
    print(f"\n¡Parámetros guardados exitosamente en '{output_py_file}'!")

    # ---- SECCIÓN DE SIMULACIÓN Y MÉTRICAS ----
    print("\n--- EJECUTANDO SIMULACIÓN Y EVALUACIÓN DE MÉTRICAS (BOUNDED LSTSQ) ---")
    print(f"{'Experimento':<18} | {'Surge (u)':<17} | {'Sway (v)':<17} | {'Yaw (r)':<17}")
    print(f"{'':<18} | {'RMS':<7} {'R2':<8} | {'RMS':<7} {'R2':<8} | {'RMS':<7} {'R2':<8}")
    print("-" * 78)
    
    num_exps = len(experiments_data)
    fig, axes = plt.subplots(num_exps, 3, figsize=(15, 2.5 * num_exps), squeeze=False)
    
    for idx, exp in enumerate(experiments_data):
        try:
            t_exp = exp["t"]
            u_real = exp["u_raw"]
            v_real = exp["v_raw"]
            r_real = exp["r_raw"]
            
            # Simular usando RK45
            u_sim, v_sim, r_sim = simulate_vessel(t_exp, u_real, v_real, r_real, exp["Tu"], exp["Tr"], theta)
            
            # Calcular métricas de error
            rms_u, r2_u = compute_metrics(u_real, u_sim)
            rms_v, r2_v = compute_metrics(v_real, v_sim)
            rms_r, r2_r = compute_metrics(r_real, r_sim)
            
            print(f"{exp['name']:<18} | {rms_u:.4f}  {r2_u:>7.3f} | {rms_v:.4f}  {r2_v:>7.3f} | {rms_r:.4f}  {r2_r:>7.3f}")
            
            # Surge
            axes[idx, 0].plot(t_exp, u_real, 'k-', alpha=0.5, label='Medido' if idx == 0 else "")
            axes[idx, 0].plot(t_exp, u_sim, 'b--', label='Modelo' if idx == 0 else "")
            if idx == 0:
                axes[idx, 0].legend(loc='upper right')
            axes[idx, 0].set_ylabel(f"{exp['name']}\nu [m/s]")
            axes[idx, 0].grid(True)
            axes[idx, 0].set_title(f"u (RMS: {rms_u:.3f}, R2: {r2_u:.2f})", fontsize=9)
            
            # Sway
            axes[idx, 1].plot(t_exp, v_real, 'k-', alpha=0.5, label='Medido' if idx == 0 else "")
            axes[idx, 1].plot(t_exp, v_sim, 'g--', label='Modelo' if idx == 0 else "")
            if idx == 0:
                axes[idx, 1].legend(loc='upper right')
            axes[idx, 1].grid(True)
            axes[idx, 1].set_title(f"v (RMS: {rms_v:.3f}, R2: {r2_v:.2f})", fontsize=9)
            
            # Yaw
            axes[idx, 2].plot(t_exp, r_real, 'k-', alpha=0.5, label='Medido' if idx == 0 else "")
            axes[idx, 2].plot(t_exp, r_sim, 'r--', label='Modelo' if idx == 0 else "")
            if idx == 0:
                axes[idx, 2].legend(loc='upper right')
            axes[idx, 2].grid(True)
            axes[idx, 2].set_title(f"r (RMS: {rms_r:.3f}, R2: {r2_r:.2f})", fontsize=9)
            
            if idx == num_exps - 1:
                axes[idx, 0].set_xlabel('Tiempo [s]')
                axes[idx, 1].set_xlabel('Tiempo [s]')
                axes[idx, 2].set_xlabel('Tiempo [s]')
                
        except Exception as e:
            print(f"Error en simulación de {exp['name']}: {e}")
            continue
            
    plt.tight_layout()
    unified_plot_path = "validation_all_experiments.png"
    plt.savefig(unified_plot_path, dpi=150)
    plt.close(fig)
    print(f"\n¡Figura unificada de validación guardada exitosamente en '{unified_plot_path}'!")

if __name__ == "__main__":
    main()