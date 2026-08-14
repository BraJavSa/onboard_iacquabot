# Regression Matrix Conditioning & Sensitivity Analysis Results

**Dataset Summary:** Processed 8 experiments (Excluded: experiment_01.mat, experiment_02.mat).  
**Matrix Dimensions:** $\mathbf{\Phi} \in \mathbb{R}^{33606 \times 9}$ (33606 rows × 9 columns)

## 1. Matrix Condition Numbers & Global Diagnostics

| Metric | Value | Description |
| :--- | :--- | :--- |
| Unscaled $\kappa(\mathbf{\Phi})$ (2-norm) | `3.7718e+01` | Ratio $\sigma_{max} / \sigma_{min}$ |
| Unscaled $\kappa(\mathbf{\Phi})$ (Frobenius) | `5.4699e+01` | $\|\mathbf{\Phi}\|_F \cdot \|\mathbf{\Phi}^+\|_F$ |
| Scaled $\kappa(\bar{\mathbf{\Phi}})$ (Unit Norm) | `7.8126e+00` | Preconditioned Spectral Condition Number |
| $\sigma_{max}$ | `8.5756e+01` | Largest Singular Value |
| $\sigma_{min}$ | `2.2736e+00` | Smallest Singular Value |

### Singular Value Spectrum & Condition Indices

| Rank | Singular Value ($\sigma_i$) | Condition Index ($\sigma_{max} / \sigma_i$) |
| :---: | :---: | :---: |
| #1 | `8.5756e+01` | `1.00` |
| #2 | `4.3381e+01` | `1.98` |
| #3 | `4.2084e+01` | `2.04` |
| #4 | `3.3804e+01` | `2.54` |
| #5 | `1.9461e+01` | `4.41` |
| #6 | `1.5048e+01` | `5.70` |
| #7 | `1.0891e+01` | `7.87` |
| #8 | `7.0014e+00` | `12.25` |
| #9 | `2.2736e+00` | `37.72` |

## 2. Parameter Excitation & Multicollinearity Diagnostics (VIF)

| Parameter | Column $L_2$ Norm | RMS Power | VIF | Collinearity Status |
| :--- | :--- | :--- | :--- | :--- |
| **m11** | `3.2735e+01` | `1.7857e-01` | `1.25` | Low (Good) ✅ |
| **m22** | `1.7305e+01` | `9.4398e-02` | `1.27` | Low (Good) ✅ |
| **m33** | `4.2084e+01` | `2.2957e-01` | `1.00` | Low (Good) ✅ |
| **Xu** | `6.5195e+01` | `3.5564e-01` | `15.70` | High ⚠️ |
| **Xuu** | `5.6761e+01` | `3.0963e-01` | `15.69` | High ⚠️ |
| **Yv** | `1.8512e+01` | `1.0098e-01` | `7.52` | Moderate ⚠️ |
| **Yvv** | `6.5520e+00` | `3.5741e-02` | `7.48` | Moderate ⚠️ |
| **Nr** | `3.5693e+01` | `1.9471e-01` | `9.03` | Moderate ⚠️ |
| **Nrr** | `2.5570e+01` | `1.3948e-01` | `8.98` | Moderate ⚠️ |

## 3. Local Parameter Sensitivity Analysis (Residual Elasticity)

Evaluates the impact of a $\pm 10\%$ parameter perturbation on the force residual norm ($R_0 = 3259.5694$).

| Parameter | Nominal Value | $+10\%$ $\Delta$Residual | $-10\%$ $\Delta$Residual | Elasticity $S_j$ |
| :--- | :--- | :--- | :--- | :--- |
| **m11** | `89.6625` | `+0.40%` | `+0.40%` | `0.0405` |
| **m22** | `75.6842` | `+0.08%` | `+0.08%` | `0.0081` |
| **m33** | `20.6754` | `+0.04%` | `+0.04%` | `0.0036` |
| **Xu** | `51.3560` | `+0.53%` | `+0.53%` | `0.0526` |
| **Xuu** | `42.9939` | `+0.28%` | `+0.28%` | `0.0280` |
| **Yv** | `43.7767` | `+0.03%` | `+0.03%` | `0.0031` |
| **Yvv** | `-94.8556` | `+0.02%` | `+0.02%` | `0.0018` |
| **Nr** | `35.8862` | `+0.08%` | `+0.08%` | `0.0077` |
| **Nrr** | `6.4145` | `+0.00%` | `+0.00%` | `0.0001` |

## 4. Monte Carlo Noise Propagation Sensitivity (5% Noise, N=200)

| Parameter | Nominal Value | MC Mean | MC Std Dev | CV (%) | Noise Sensitivity |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **m11** | `89.6625` | `89.3085` | `0.0902` | `0.10%` | Robust (Low) ✅ |
| **m22** | `75.6842` | `75.1559` | `0.1811` | `0.24%` | Robust (Low) ✅ |
| **m33** | `20.6754` | `20.6195` | `0.0576` | `0.28%` | Robust (Low) ✅ |
| **Xu** | `51.3560` | `50.9754` | `0.2960` | `0.58%` | Robust (Low) ✅ |
| **Xuu** | `42.9939` | `43.3303` | `0.3398` | `0.78%` | Robust (Low) ✅ |
| **Yv** | `43.7767` | `42.2636` | `0.5432` | `1.29%` | Robust (Low) ✅ |
| **Yvv** | `-94.8556` | `-90.7437` | `1.5566` | `1.72%` | Robust (Low) ✅ |
| **Nr** | `35.8862` | `35.1767` | `0.3727` | `1.06%` | Robust (Low) ✅ |
| **Nrr** | `6.4145` | `7.3323` | `0.5189` | `7.08%` | Moderate ⚠️ |

## 5. Summary & Diagnostic Key Takeaways

- **Matrix Conditioning:** Scaled condition number $\kappa(\bar{\mathbf{\Phi}}) = 7.81$ confirms excellent numerical stability after column preconditioning.
- **Multicollinearity:** $X_u$ and $X_{uu}$ show high VIF (~15.7), indicating structural coupling between linear and quadratic surge damping.
- **Parameter Elasticity:** Surging dynamics ($X_u, m_{11}, X_{uu}$) are most sensitive to residual force errors.
- **Noise Robustness:** Parameters exhibit high noise robustness under 5% signal noise ($CV < 2\%$ for 8/9 parameters, $N_{rr}$ at $7.08\%$).
