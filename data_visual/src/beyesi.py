import numpy as np
import matplotlib.pyplot as plt

# 状态空间（1D位置，离散化）
x = np.arange(0, 100)
N = len(x)

# 初始分布：均匀
bel = np.ones(N) / N

def predict(bel, motion, motion_noise=2):
    """预测步骤: 卷积平移"""
    new_bel = np.zeros_like(bel)
    for i in range(N):
        shift = (i - motion) % N
        new_bel[i] = bel[shift]
    # 加噪声：高斯模糊
    kernel = np.exp(-0.5*((np.arange(-10,11))/motion_noise)**2)
    kernel /= kernel.sum()
    return np.convolve(new_bel, kernel, mode="same")

def update(bel, z, sensor_noise=3):
    """更新步骤: 似然加权"""
    likelihood = np.exp(-0.5*((x - z)/sensor_noise)**2)  
    posterior = bel * likelihood
    return posterior / posterior.sum()

# 模拟运动+观测
motions = [5, 5, 5, 5, 5]
measurements = [7, 15, 22, 28, 35]

history = []
for u, z in zip(motions, measurements):
    bel = predict(bel, u)
    bel = update(bel, z)
    history.append(bel)

# 绘图
plt.figure(figsize=(8,4))
for t, h in enumerate(history):
    plt.plot(x, h, label=f"t={t+1}")
plt.xlabel("位置")
plt.ylabel("概率")
plt.legend()
plt.title("1D 贝叶斯滤波 (网格近似)")
plt.show()
