# Optimal dynamic parameters identified with Powell method (Asymmetric)
import numpy as np

# --- DYNAMIC PARAMETERS ---
m11 = 135.7315760000
m22 = 75.3375750000
m33 = 23.7138320000
Xu_pos = 2.9043366685
Xu_neg = 150.0000000000
Xuu_pos = 95.3145111531
Xuu_neg = 142.1201310966
Yv = 50.0000000000
Yvv = 0.0000000000
Yvr = -81.7884720000
Yr = -22.5235860000
Yrv = 37.7555150000
Yrr = 10.3380750000
Yur = -89.3941620000
Nv = 10.2410310000
Nvv = 49.0015750000
Nvr = -25.7744760000
Nr = 34.3242850000
Nrv = 12.7384390000
Nrr = 15.4391350000

# For compatibility, average values for Xu and Xuu
Xu = 0.5 * (Xu_pos + Xu_neg)
Xuu = 0.5 * (Xuu_pos + Xuu_neg)
theta = [m11, m22, m33, Xu, Xuu, Yv, Yvv, Yvr, Yr, Yrv, Yrr, Yur, Nv, Nvv, Nvr, Nr, Nrv, Nrr]

# --- MOTOR PARAMETERS ---
motor_neg_A = -31.4990000000
motor_neg_B = 3.6986000000
motor_neg_v = 0.3264000000
motor_neg_C = 0.9713000000
motor_neg_M = -1.0000000000
max_rev = -28.4393000000

motor_pos_K = 40.0209000000
motor_pos_B = 2.6249000000
motor_pos_v = 0.1615000000
motor_pos_C = 0.9432000000
motor_pos_M = 0.0000100000
max_fwd = 36.3827000000
