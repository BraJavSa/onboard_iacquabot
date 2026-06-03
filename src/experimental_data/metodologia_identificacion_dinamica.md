# Metodología de identificación dinámica y validación experimental

## 1. Objetivo

El procedimiento desarrollado tuvo como objetivo obtener un modelo dinámico planar de 3 grados de libertad para el vehículo acuático, a partir de datos experimentales registrados en ensayos de navegación. El modelo identificado relaciona las fuerzas y momentos generados por los propulsores con las velocidades medidas del vehículo en surge, sway y yaw, denotadas respectivamente como \(u\), \(v\) y \(r\).

La metodología se implementó en tres etapas principales:

1. Conversión de comandos PWM a fuerzas/momentos generalizados.
2. Identificación global de parámetros dinámicos mediante un enfoque de error de salida.
3. Estimación de una saturación equivalente de corriente y generación de reportes de validación.

Los scripts centrales de este procedimiento son `identification.py`, `find_saturation.py` y `generate_pdf_report.py`.

## 2. Datos experimentales

Se utilizaron archivos experimentales en formato `.mat`, nombrados como `experiment_01.mat` hasta `experiment_10.mat`. Cada archivo contiene:

- Vector temporal \(t\).
- Velocidad longitudinal \(u = v_x\).
- Velocidad lateral \(v = v_y\).
- Velocidad angular de guiñada \(r = \omega_z\).
- Señales PWM individuales de los cuatro propulsores, `pwm1` a `pwm4`.

El conjunto completo de experimentos disponibles fue usado para la identificación global de los parámetros. Posteriormente, los mismos experimentos fueron utilizados para evaluar la capacidad del modelo de reproducir la respuesta medida. En particular, `experiment_10.mat` se empleó como caso representativo de validación dinámica en `identification.py`; por estar incluido en el conjunto de identificación, esta evaluación debe interpretarse como validación interna o reproducción dinámica, no como validación independiente fuera de muestra.

## 3. Conversión de PWM a comando normalizado

Cada señal PWM fue convertida a un comando adimensional mediante

\[
c_i = \frac{\mathrm{PWM}_i - \mathrm{PWM}_{mid}}{\mathrm{PWM}_{max} - \mathrm{PWM}_{mid}},
\]

donde

\[
\mathrm{PWM}_{mid} = 1500 \ \mu s, \qquad
\mathrm{PWM}_{max} = 1900 \ \mu s.
\]

Debido a la orientación física de los motores, se aplicó inversión de signo a los motores definidos como invertidos. En los scripts, el vector de inversión es

\[
\mathrm{MOTOR\_INVERTED} = [\mathrm{True}, \mathrm{True}, \mathrm{False}, \mathrm{False}].
\]

Por tanto, para un motor invertido se usa \(-c_i\) en lugar de \(c_i\).

## 4. Modelo estático de propulsor T200

La fuerza producida por cada propulsor se obtuvo a partir del comando normalizado usando un modelo estático no lineal tipo logística generalizada, identificado para propulsores Blue Robotics T200. Para comandos positivos y negativos se emplearon dos conjuntos de parámetros independientes:

\[
T(c) =
A + \frac{K - A}{\left(C + \exp[-B(c-M)]\right)^{1/v}}.
\]

El modelo se evaluó de forma separada para las ramas positiva y negativa. Además, la fuerza fue saturada a los límites

\[
T_{\max}^{+} = 36.3827 \ \mathrm{N}, \qquad
T_{\max}^{-} = -28.4393 \ \mathrm{N}.
\]

Para comandos cercanos a cero, específicamente \(|c| \leq 0.01\), la fuerza se consideró nula. Esta zona muerta evita introducir fuerzas espurias debidas a pequeñas desviaciones alrededor del PWM neutro.

## 5. Asignación de fuerzas y momento de guiñada

Las fuerzas individuales de los cuatro propulsores se reorganizaron en el orden físico:

\[
\mathbf{T} =
\begin{bmatrix}
T_{FR} & T_{FL} & T_{BR} & T_{BL}
\end{bmatrix}^{T}.
\]

En la implementación, esta reorganización se obtiene desde las señales `pwm1` a `pwm4` mediante

\[
\mathbf{T} =
\begin{bmatrix}
T_{pwm4} & T_{pwm2} & T_{pwm3} & T_{pwm1}
\end{bmatrix}^{T}.
\]

Las fuerzas y momentos generalizados se calcularon con la matriz de asignación

\[
\boldsymbol{\tau}
=
\begin{bmatrix}
\tau_u \\
\tau_v \\
\tau_r
\end{bmatrix}
=
\mathbf{B}\mathbf{T},
\]

donde

\[
\mathbf{B} =
\begin{bmatrix}
1 & 1 & 1 & 1 \\
0 & 0 & 0 & 0 \\
-0.30 & 0.30 & -0.30 & 0.30
\end{bmatrix}.
\]

Esta matriz representa una configuración donde los propulsores contribuyen directamente a la fuerza en surge \(\tau_u\) y al momento de yaw \(\tau_r\). No se modela una fuerza actuada directa en sway, es decir, \(\tau_v = 0\).

## 6. Modelo dinámico identificado

Se empleó un modelo dinámico planar de 3 grados de libertad, con estructura compatible con la formulación estándar de Fossen para vehículos marinos. El vector de estado es

\[
\mathbf{x} =
\begin{bmatrix}
u & v & r
\end{bmatrix}^{T}.
\]

Las ecuaciones dinámicas identificadas fueron:

\[
\dot{u}
=
\frac{
\tau_u + m_{22}vr - X_u u - X_{uu}|u|u
}{m_{11}},
\]

\[
\dot{v}
=
\frac{
-m_{11}ur - Y_v v - Y_{vv}|v|v
}{m_{22}},
\]

\[
\dot{r}
=
\frac{
\tau_r + (m_{11}-m_{22})uv - N_r r - N_{rr}|r|r
}{m_{33}}.
\]

Los parámetros estimados fueron:

\[
\theta =
\begin{bmatrix}
m_{11} &
m_{22} &
m_{33} &
X_u &
X_{uu} &
Y_v &
Y_{vv} &
N_r &
N_{rr}
\end{bmatrix}^{T}.
\]

En la nomenclatura del código, estos términos se reportan como:

- \(m_{11} = m - X_{\dot{u}}\), reportado como `m-Xudot`.
- \(m_{22} = m - Y_{\dot{v}}\), reportado como `m-Yvdot`.
- \(m_{33} = I_z - N_{\dot{r}}\), reportado como `Iz-Nrdot`.
- \(X_u, X_{uu}, Y_v, Y_{vv}, N_r, N_{rr}\), coeficientes de amortiguamiento lineal y cuadrático.

## 7. Identificación por error de salida

La identificación se realizó mediante un enfoque de error de salida global. Para cada experimento se integró numéricamente el modelo dinámico usando como condición inicial el primer valor medido:

\[
\mathbf{x}(t_0) =
\begin{bmatrix}
u(t_0) & v(t_0) & r(t_0)
\end{bmatrix}^{T}.
\]

Las entradas \(\tau_u(t)\) y \(\tau_r(t)\), calculadas a partir de los PWM, fueron interpoladas temporalmente mediante interpolación lineal con extrapolación en los bordes. Para cada conjunto de parámetros candidato, el modelo fue integrado sobre el mismo vector temporal de cada experimento usando `solve_ivp`.

La función de costo minimizada fue el promedio, sobre todos los experimentos, de errores cuadráticos medios normalizados por la varianza de cada señal:

\[
J(\theta)
=
\frac{1}{N_e}
\sum_{k=1}^{N_e}
\left[
\frac{\mathrm{MSE}(u_k,\hat{u}_k)}{\mathrm{Var}(u_k)+\epsilon}
+ 2
\frac{\mathrm{MSE}(v_k,\hat{v}_k)}{\mathrm{Var}(v_k)+\epsilon}
+
\frac{\mathrm{MSE}(r_k,\hat{r}_k)}{\mathrm{Var}(r_k)+\epsilon}
\right],
\]

donde \(N_e\) es el número de experimentos disponibles, \(\hat{u}\), \(\hat{v}\) y \(\hat{r}\) son las trayectorias simuladas, y \(\epsilon=10^{-6}\) evita divisiones por varianzas muy pequeñas.

El error asociado a sway fue ponderado por un factor 2. Esta ponderación aumenta la influencia relativa de \(v\) en la identificación, lo cual es relevante porque sway no está directamente actuado en la matriz de asignación y su dinámica se observa principalmente a través de los acoplamientos del modelo.

La optimización se resolvió mediante el método Nelder-Mead, inicializado con

\[
\theta_0 =
\begin{bmatrix}
100 & 80 & 18 & 35 & 60 & 80 & 0 & 30 & 15
\end{bmatrix}^{T}.
\]

Se penalizaron soluciones no físicas imponiendo masas/inercias efectivas positivas y amortiguamientos lineales positivos para \(m_{11}\), \(m_{22}\), \(m_{33}\), \(X_u\), \(Y_v\) y \(N_r\). En cambio, el coeficiente \(Y_{vv}\) no fue restringido a ser positivo en esta implementación.

Los parámetros obtenidos en `identification.py` fueron:

| Parámetro | Valor |
|---|---:|
| \(m_{11}=m-X_{\dot{u}}\) | 132.556203 |
| \(m_{22}=m-Y_{\dot{v}}\) | 159.117429 |
| \(m_{33}=I_z-N_{\dot{r}}\) | 18.018928 |
| \(X_u\) | 18.187662 |
| \(X_{uu}\) | 68.719290 |
| \(Y_v\) | 325.907865 |
| \(Y_{vv}\) | -0.006465 |
| \(N_r\) | 60.393403 |
| \(N_{rr}\) | 0.853737 |

## 8. Modelo equivalente de saturación de corriente

Después de identificar los parámetros dinámicos, se incorporó un modelo de saturación de corriente para representar una limitación global en la alimentación de los propulsores. La corriente de cada motor se aproximó como función cuadrática del PWM:

\[
I(\mathrm{PWM})
=
1.0895 \times 10^{-4} \mathrm{PWM}^{2}
- 0.32685 \mathrm{PWM}
+ 244.477.
\]

Para cada instante de tiempo se calculó la corriente total:

\[
I_{\mathrm{tot}}(t)
=
\sum_{i=1}^{4} I_i(t).
\]

Si la corriente total excede un valor límite \(I_{\mathrm{sat}}\), las corrientes individuales se escalan proporcionalmente:

\[
\alpha(t)
=
\begin{cases}
\frac{I_{\mathrm{sat}}}{I_{\mathrm{tot}}(t)}, & I_{\mathrm{tot}}(t) > I_{\mathrm{sat}},\\
1, & I_{\mathrm{tot}}(t) \leq I_{\mathrm{sat}},
\end{cases}
\]

\[
I_{i,\mathrm{sat}}(t) = \alpha(t) I_i(t).
\]

Luego, cada corriente saturada se convierte nuevamente a PWM resolviendo la ecuación cuadrática inversa. Para mantener la rama correcta de la solución, se usa la dirección del PWM original: la raíz superior para \(\mathrm{PWM}\geq1500\) y la raíz inferior para \(\mathrm{PWM}<1500\). Finalmente, los PWM saturados se transforman otra vez a comandos, fuerzas y momentos generalizados.

El valor de \(I_{\mathrm{sat}}\) se estimó en `find_saturation.py` manteniendo fijos los parámetros dinámicos previamente identificados y minimizando la misma función de error global. La optimización fue escalar, acotada al intervalo

\[
10 \ \mathrm{A} \leq I_{\mathrm{sat}} \leq 50 \ \mathrm{A},
\]

mediante el método bounded de `minimize_scalar`. El valor utilizado posteriormente en el reporte fue

\[
I_{\mathrm{sat}} = 48.79 \ \mathrm{A}.
\]

## 9. Validación dinámica y métrica de desempeño

La calidad de ajuste del modelo se evaluó integrando las ecuaciones dinámicas con las entradas reconstruidas a partir de los PWM medidos. La comparación se realizó entre las trayectorias medidas y simuladas para \(u\), \(v\) y \(r\).

La métrica principal fue el coeficiente de determinación:

\[
R^2
=
1 -
\frac{
\sum_i (y_i-\hat{y}_i)^2
}{
\sum_i (y_i-\bar{y})^2 + 10^{-8}
}.
\]

Esta métrica se calculó de manera independiente para surge, sway y yaw. Valores cercanos a 1 indican alta concordancia entre la señal medida y la simulada. Valores negativos indican que el modelo reproduce peor la señal que un predictor constante igual a la media.

## 10. Generación del reporte comparativo

El script `generate_pdf_report.py` automatiza la validación visual para los diez experimentos. Para cada archivo experimental:

1. Carga las señales medidas \(u\), \(v\), \(r\) y los PWM.
2. Calcula las entradas \(\tau_u\) y \(\tau_r\) sin saturación.
3. Calcula las entradas \(\tau_u\) y \(\tau_r\) con saturación de corriente usando \(I_{\mathrm{sat}}=48.79\ \mathrm{A}\).
4. Integra el modelo dinámico en ambos casos.
5. Calcula \(R^2\) para cada variable y para cada modelo de entrada.
6. Genera una página por experimento en el archivo `Validation_Report_Experiments.pdf`.

Cada página compara tres señales por variable:

- Señal medida.
- Simulación sin saturación de corriente.
- Simulación con saturación de corriente.

Esto permite evaluar visualmente si la saturación equivalente mejora la reproducción de la respuesta dinámica, especialmente en maniobras donde la demanda simultánea de los cuatro propulsores puede exceder la capacidad efectiva del sistema de alimentación.

## 11. Interpretación metodológica

El procedimiento completo constituye una identificación de parámetros por simulación directa del sistema, no una identificación basada en derivadas numéricas instantáneas. Esto es importante porque el método de error de salida evalúa la coherencia dinámica acumulada de las trayectorias simuladas, reduciendo la dependencia de estimaciones ruidosas de \(\dot{u}\), \(\dot{v}\) y \(\dot{r}\).

El uso simultáneo de todos los experimentos en la función de costo produce un único conjunto de parámetros globales, en lugar de parámetros ajustados caso por caso. Por tanto, el modelo resultante busca representar un comportamiento dinámico promedio y consistente del vehículo bajo las diferentes maniobras ensayadas.

La posterior identificación de \(I_{\mathrm{sat}}\) introduce una corrección fenomenológica de la entrada. Este parámetro no representa necesariamente una medición eléctrica directa de la corriente máxima disponible, sino una saturación equivalente que compensa la diferencia entre el empuje ideal predicho por el modelo estático de propulsor y el empuje efectivo observado durante los experimentos.

## 12. Alcances y limitaciones

El modelo identificado captura la dinámica planar en surge, sway y yaw, pero no incluye explícitamente roll, pitch, heave, efectos ambientales variables, retardo de actuadores ni dinámica interna de los propulsores. Además, la matriz de asignación considera nula la fuerza directa en sway, por lo que la velocidad lateral se explica mediante acoplamientos dinámicos y amortiguamiento.

La validación presentada en los scripts corresponde principalmente a reproducción sobre el conjunto experimental usado durante el ajuste. Para fortalecer el uso del modelo en un artículo científico, se recomienda reportar explícitamente esta condición y, si es posible, reservar ensayos adicionales como conjunto de validación independiente.

Finalmente, la saturación de corriente fue modelada como un escalamiento proporcional de corrientes individuales cuando la suma total supera \(I_{\mathrm{sat}}\). Esta aproximación es útil para representar una limitación global de potencia, pero no sustituye una caracterización eléctrica detallada de baterías, reguladores, controladores ESC y caídas de tensión bajo carga.
