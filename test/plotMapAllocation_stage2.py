import matplotlib.pyplot as plt


def plotMapAllocation_stage2(X, na, color, name):
    for i in range(na):
        xx = X[i]
        if xx is not None:
            plt.plot(xx[0, :], xx[1, :], ":", color=color[i], linewidth=2, label=name)
