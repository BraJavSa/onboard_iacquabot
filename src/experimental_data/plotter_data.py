import numpy as np
import matplotlib.pyplot as plt
from scipy.io import loadmat

D = loadmat("experiment_10.mat")

t = D["t"].squeeze()

x = D["x"].squeeze()
y = D["y"].squeeze()
yaw = D["yaw"].squeeze()

vx = D["vx"].squeeze()
vy = D["vy"].squeeze()
wz = D["wz"].squeeze()

u = np.cos(yaw)*vx + np.sin(yaw)*vy
v = -np.sin(yaw)*vx + np.cos(yaw)*vy
r = wz

fig, ax = plt.subplots(3, 1, figsize=(12, 8), sharex=True)

ax[0].plot(t, x)
ax[0].set_ylabel("x [m]")
ax[0].set_title("Position X")
ax[0].grid(True)

ax[1].plot(t, y)
ax[1].set_ylabel("y [m]")
ax[1].set_title("Position Y")
ax[1].grid(True)

ax[2].plot(t, yaw)
ax[2].set_ylabel("yaw [rad]")
ax[2].set_xlabel("Time [s]")
ax[2].set_title("Heading")
ax[2].grid(True)

fig, ax = plt.subplots(3, 1, figsize=(12, 8), sharex=True)

ax[0].plot(t, u)
ax[0].set_ylabel("u [m/s]")
ax[0].set_title("Surge Velocity")
ax[0].grid(True)

ax[1].plot(t, v)
ax[1].set_ylabel("v [m/s]")
ax[1].set_title("Sway Velocity")
ax[1].grid(True)

ax[2].plot(t, r)
ax[2].set_ylabel("r [rad/s]")
ax[2].set_xlabel("Time [s]")
ax[2].set_title("Yaw Rate")
ax[2].grid(True)

plt.figure(figsize=(12, 6))

plt.plot(t, D["pwm1"].squeeze(), label="PWM1")
plt.plot(t, D["pwm2"].squeeze(), label="PWM2")
plt.plot(t, D["pwm3"].squeeze(), label="PWM3")
plt.plot(t, D["pwm4"].squeeze(), label="PWM4")

plt.title("PWM Commands")
plt.xlabel("Time [s]")
plt.ylabel("PWM")
plt.grid(True)
plt.legend()

plt.show()