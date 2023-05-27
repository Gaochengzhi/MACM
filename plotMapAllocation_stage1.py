import matplotlib.pyplot as plt


def plotMapAllocation_stage1(X, n_rounds, na, color, name):
    for i in range(na):
        xx = X[:, i, :]
        xx = xx[:, xx[0, :] != 0]
        plt.plot(xx[0, :], xx[1, :], ':', color=color[i], linewidth=2, label=name)
    plt.legend()
    plt.show()

