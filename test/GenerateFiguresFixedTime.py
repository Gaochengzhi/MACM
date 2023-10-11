import matplotlib.pyplot as plt
import numpy as np
import scipy.io as sio
from matplotlib2tikz import save as tikz_save

simu_number = 6
PlotCommLimit = 0
saveMovie = 0

nRatioPointsFigure = 10
lineWidthFigure = 2
markerSizeFigure = 12

SimuParamsCell = sio.loadmat('mat/Dynamics/simu_{}/SimuParamsCell.mat'.format(simu_number))

for t_plot in [0, 4, 10]:
    for CommLimit in [0, 1]:
        X_just_saved_CommLimit = sio.loadmat('mat/Dynamics/simu_{}/X_just_saved_CommLimit_{}.mat'.format(simu_number, CommLimit))
        J_to_completion_target_CommLimit = sio.loadmat('mat/Dynamics/simu_{}/J_to_completion_target_CommLimit_{}.mat'.format(simu_number, CommLimit))
        J_CommLimit = sio.loadmat('mat/Dynamics/simu_{}/J_CommLimit_{}.mat'.format(simu_number, CommLimit))
        p_GCAA_just_saved_CommLimit = sio.loadmat('mat/Dynamics/simu_{}/p_GCAA_just_saved_CommLimit_{}.mat'.format(simu_number, CommLimit))
        S_GCAA_ALL_just_saved_CommLimit = sio.loadmat('mat/Dynamics/simu_{}/S_GCAA_ALL_just_saved_CommLimit_{}.mat'.format(simu_number, CommLimit))
        rt_just_saved_CommLimit = sio.loadmat('mat/Dynamics/simu_{}/rt_just_saved_CommLimit_{}.mat'.format(simu_number, CommLimit))

        figure = plt.figure()
        ax = figure.add_subplot(111)
        PlotAllocTime(X_full_simu, t_plot, SimuParamsCell['time_step'], SimuParamsCell['pos_t'], SimuParamsCell['map_width'], SimuParamsCell['colors'], SimuParamsCell['radius_t'], SimuParamsCell['task_type'], nRatioPointsFigure, lineWidthFigure, markerSizeFigure)
        tikz_save('mat/Dynamics/simu_{}/Alloc_5_5_t_{}_CommLimit_{}.tex'.format(simu_number, t_plot, CommLimit))

        plt.close(figure)

