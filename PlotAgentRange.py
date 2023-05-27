import matplotlib.pyplot as plt
import numpy as np


def PlotAgentRange(pos_a, comm_distance, color, name):
    n = 20
    theta = np.linspace(0, 2 * np.pi, n)
    for i in range(pos_a.shape[0]):
        pos = np.tile(pos_a[i, :], (n, 1)) + comm_distance * np.array([np.cos(theta), np.sin(theta)]).T
        plt.plot(pos[:, 0], pos[:, 1], '--', color=color[i], linewidth=1, label=name)
    plt.legend()
    plt.show()

