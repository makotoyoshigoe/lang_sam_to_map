import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider

# --- 初期パラメータ ---
k0 = 1.0
sigma0 = 0.3
d_th = 3.0

# --- 距離の範囲 ---
# d = np.linspace(0.01, 2.0, 500)
d1 = np.linspace(0.01, 0.25, 50)
d2 = np.linspace(0.25, d_th, 450)

# --- 斥力の式 ---
def inverse_force(d, k):
    return k / d

def exp_force(d, k, sigma):
    return k * np.exp(-d / sigma)

def org_repulsive_force(d, k, d_th):
    return k * (1 / d - 1 / d_th) / d

# --- 図の準備 ---
fig, ax = plt.subplots(figsize=(8, 5))
plt.subplots_adjust(left=0.15, bottom=0.30)

# 初期プロット
# line_inv, = ax.plot(d, inverse_force(d, k0), label="Inverse (k/d)")
line_org_rep, = ax.plot(d2, org_repulsive_force(d2, k0, d_th), label="k * (1/d-1/d_th)/d")
# line_exp, = ax.plot(d, exp_force(d, k0, sigma0), label="Exponential (k * exp(-d/sigma))")

ax.set_xlabel("Distance d [m]")
ax.set_ylabel("Force magnitude")
ax.set_title("Repulsive Force Comparison (Interactive Sliders)")
ax.grid(True)
ax.legend()

# --- スライダー領域 ---
ax_k = plt.axes([0.15, 0.18, 0.65, 0.03])
ax_sigma = plt.axes([0.15, 0.12, 0.65, 0.03])

slider_k = Slider(ax_k, "k", 0.1, 5.0, valinit=k0)
slider_sigma = Slider(ax_sigma, "sigma", 0.05, 1.0, valinit=sigma0)

# --- スライダー更新時の処理 ---
def update(val):
    k = slider_k.val
    # jsigma = slider_sigma.val
    
    # line_inv.set_ydata(inverse_force(d, k))
    # line_exp.set_ydata(exp_force(d, k, sigma))
    line_org_rep.set_ydata(org_repulsive_force(d2, k, d_th))
    
    fig.canvas.draw_idle()

slider_k.on_changed(update)
slider_sigma.on_changed(update)

plt.show()

