import matplotlib.pyplot as plt


def PlotAllocTime(X_full_simu, t_plot, time_step, pos_t, map_width, colors, radius_t, task_type, nRatioPointsFigure, lineWidthFigure, markerSizeFigure):
    plt.xlim([0, map_width])
    plt.ylim([0, map_width])
    plt.xlabel('x [m]')
    plt.ylabel('y [m]')
    na = X_full_simu[0].shape[1]
    n_round = X_full_simu[0].shape[2]
    plt.plot(pos_t[:, 0], pos_t[:, 1], 'ks', markersize=markerSizeFigure/2.5, markerfacecolor=[0, 0, 0])
    PlotTaskLoitering(pos_t, radius_t, task_type, 'k--', 'Task loitering')
    
    round_plot = int(t_plot / time_step) + 1
    if round_plot >= n_round:
        round_plot = n_round-1
    
    X_prev = np.zeros((4, na, round_plot))
    for k in range(round_plot):
        X_prev[:, :, k] = X_full_simu[k][:, :, 0]
    
    X_next = np.zeros((4, na, n_round - round_plot))
    for i in range(na):
        X_next[:, i, :] = X_full_simu[round_plot][:, i, 1:]
    
    for i in range(na):
        xX_prev = X_prev[:, i, ::nRatioPointsFigure].reshape(4, -1)
        xX_next = X_next[:, i, ::nRatioPointsFigure].reshape(4, -1)
        plt.plot(xX_prev[0, :], xX_prev[1, :], color=colors[i], linewidth=lineWidthFigure)
        plt.plot(xX_next[0, :], xX_next[1, :], '--', color=colors[i], linewidth=lineWidthFigure-1)
        plt.plot(xX_next[0, 0], xX_next[1, 0], 'h', color=colors[i], markersize=markerSizeFigure, markerfacecolor=colors[i])
    
    plt.show()

