import scipy.io as sio

simu_number = 7
CommLimit = 1
PlotCommLimit = 0
saveMovie = 0

X_just_saved_CommLimit = sio.loadmat('mat/Dynamics/simu_{}/X_just_saved_CommLimit_{}.mat'.format(simu_number, CommLimit))
J_to_completion_target_CommLimit = sio.loadmat('mat/Dynamics/simu_{}/J_to_completion_target_CommLimit_{}.mat'.format(simu_number, CommLimit))
J_CommLimit = sio.loadmat('mat/Dynamics/simu_{}/J_CommLimit_{}.mat'.format(simu_number, CommLimit))
p_GCAA_just_saved_CommLimit = sio.loadmat('mat/Dynamics/simu_{}/p_GCAA_just_saved_CommLimit_{}.mat'.format(simu_number, CommLimit))
S_GCAA_ALL_just_saved_CommLimit = sio.loadmat('mat/Dynamics/simu_{}/S_GCAA_ALL_just_saved_CommLimit_{}.mat'.format(simu_number, CommLimit))
rt_just_saved_CommLimit = sio.loadmat('mat/Dynamics/simu_{}/rt_just_saved_CommLimit_{}.mat'.format(simu_number, CommLimit))
SimuParamsCell = sio.loadmat('mat/Dynamics/simu_{}/SimuParamsCell.mat'.format(simu_number))

filename = 'mat/Dynamics/MAllocUtil_{}.mat'.format(CommLimit)
MAllocUtil = PlotAnimationAndUtility(X_full_simu, SimuParamsCell, J, J_to_completion_target, rt_full_simu, PlotCommLimit, saveMovie, filename)
if saveMovie:
    sio.savemat('mat/Dynamics/MAllocUtil_{}.mat'.format(CommLimit), {'MAllocUtil': MAllocUtil})

