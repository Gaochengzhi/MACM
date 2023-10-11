import matplotlib.pyplot as plt


def PlotAnimationAndUtility(X_full_simu, SimuParamsCell, J, J_to_completion_target, rt_full_simu, PlotCommLimit, saveMovie, filename):
    n_rounds = SimuParamsCell.n_rounds
    time_step = SimuParamsCell.time_step
    pos_t = SimuParamsCell.pos_t
    map_width = SimuParamsCell.map_width
    colors = SimuParamsCell.colors
    radius_t = SimuParamsCell.radius_t
    task_type = SimuParamsCell.task_type
    comm_distance = SimuParamsCell.comm_distance

    nRatioPointsFigure = 1
    lineWidthFigure = 4
    markerSizeFigure = 20

    na = J.shape[1]
    J_tot = J + J_to_completion_target

    win = plt.figure(1)
    win.subplots_adjust(wspace=0.3, hspace=0.5)
    ax1 = win.add_subplot(2, 3, 1)
    ax1.set_xlim([0, n_rounds*time_step])
    ax1.set_ylim([0, max(max(J_tot))*1.1])
    ax1.set_title('Cost')
    ax1.set_xlabel('t [s]')
    ax2 = win.add_subplot(2, 3, 4)
    ax2.set_xlim([0, n_rounds*time_step])
    ax2.set_ylim([0, max(sum(rt_full_simu, 1))*1.1])
    ax2.set_xlabel('t [s]')
    ax3 = win.add_subplot(2, 3, (2, 6))
    ax3.set_xlim([0, 1])
    ax3.set_ylim([0, 1])
    t = list(range(n_rounds+1)) * time_step
    win.set_facecolor('w')
    win.set_figwidth(16)
    win.set_figheight(9)
    
    M = []
    for k in range(1, n_rounds-1):
        ax1.clear()
        t_plot = time_step * (k-1)
        for i in range(na):
            ax1.plot(t[:k], J_tot[:k, i], linewidth=2)
        ax2.clear()
        ax2.plot(t[:k], sum(rt_full_simu[:k, :], axis=1), 'k--', linewidth=3)
        U_tot = sum(rt_full_simu[:k-1, :], axis=1) - sum(J_tot[1:k, :], axis=1)
        U_tot[0] = 0
        ax2.plot(t[:k-1], U_tot, 'k', linewidth=3)
        ax2.legend(['Reward', 'Utility'], loc='lower right')
        ax3.clear()
        PlotAllocTime(X_full_simu, t_plot, time_step, pos_t, map_width, colors, radius_t, task_type, nRatioPointsFigure, lineWidthFigure, markerSizeFigure)
        if PlotCommLimit:
            pos_a = X_full_simu[k][:, :, 0]
            PlotAgentRange(pos_a, comm_distance, colors, '')
        plt.draw()
        if saveMovie:
            M.append(win.canvas.copy_from_bbox(win.bbox))
        plt.pause(0.001)
    
    if saveMovie:
        win = plt.figure()
        fps = int(1 / time_step)
        ani = animation.ArtistAnimation(win, M, interval=1000/fps, repeat_delay=1000)
        ani.save(filename, writer='imagemagick')
    
    plt.savefig(filename)
    plt.close(win)

