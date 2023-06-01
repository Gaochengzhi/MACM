import numpy as np
import matplotlib.pyplot as plt


def PlotTaskLoitering(pos_t, radius_t, type_t, color, name=None):
    n = 20
    theta = np.linspace(0, 2 * np.pi, n)
    for i in range(pos_t.shape[0]):
        if type_t[i] == 1:
            pos = np.tile(pos_t[i, :], (n, 1)) + radius_t[i] * np.column_stack(
                (np.cos(theta), np.sin(theta))
            )
            plt.plot(pos[:, 0], pos[:, 1], color=color, linewidth=2)
    plt.legend()
