import scipy.io
import numpy as np
import glob

archivos = sorted(glob.glob('*.mat'))

for archivo in archivos:
    mat = scipy.io.loadmat(archivo)
    
    t  = mat['t'].flatten()
    x  = mat['x'].flatten()
    y  = mat['y'].flatten()
    vx = mat['vx'].flatten()
    vy = mat['vy'].flatten()
    wz = mat['wz'].flatten()

    speed = np.sqrt(vx**2 + vy**2)
    dist  = np.sqrt(x**2 + y**2)

    print(f"--- {archivo} ---")
    print(f"  Duración:                {t[-1] - t[0]:.1f} s")
    print(f"  Velocidad máxima:        {speed.max():.3f} m/s")
    print(f"  Velocidad angular máx:   {np.abs(wz).max():.3f} rad/s")
    print(f"  Distancia máxima origen: {dist.max():.1f} m")
    print()