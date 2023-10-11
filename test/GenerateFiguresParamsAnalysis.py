import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib2tikz import save as tikz_save
import scipy.io as sio

simuName = 'Variation of parameters'
lineWidth = 3

AllParamsToAnalyze = ['RangeLimitation', 'RewardSuccessProbability', 'NumberAgentsTasks', 'TimeRatioLoiteringTasks']

# RangeLimitation
paramToAnalyze = AllParamsToAnalyze[0]
simuNumber = 1
ratioRangeMapWidth = sio.loadmat('mat/{}/{}/simu_{}/ratioRangeMapWidth-{}.mat'.format(simuName, paramToAnalyze, simuNumber, simuNumber))
totalUtilityAllocation = sio.loadmat('mat/{}/{}/simu_{}/totalUtilityAllocation-{}.mat'.format(simuName, paramToAnalyze, simuNumber, simuNumber))
SimuParamsCell = sio.loadmat('mat/{}/{}/simu_{}/SimuParamsCell.mat'.format(simuName, paramToAnalyze, simuNumber))

figure = plt.figure()
plt.plot(ratioRangeMapWidth, totalUtilityAllocation, linewidth=lineWidth)
plt.ylim([0, max(totalUtilityAllocation)*1.1])
plt.xlabel('Communication range')
plt.ylabel('Utility')
tikz_save('mat/{}/{}/simu_{}/RangeLimitationUtility.tex'.format(simuName, paramToAnalyze, simuNumber))
plt.close(figure)

# RewardSuccessProbability
paramToAnalyze = AllParamsToAnalyze[1]
simuNumber = 1
nomReward = sio.loadmat('mat/{}/{}/simu_{}/nomReward-{}.mat'.format(simuName, paramToAnalyze, simuNumber, simuNumber))
nomProbAT = sio.loadmat('mat/{}/{}/simu_{}/nomProbAT-{}.mat'.format(simuName, paramToAnalyze, simuNumber, simuNumber))
totalUtilityAllocation = sio.loadmat('mat/{}/{}/simu_{}/totalUtilityAllocation-{}.mat'.format(simuName, paramToAnalyze, simuNumber, simuNumber))
SimuParamsCell = sio.loadmat('mat/{}/{}/simu_{}/SimuParamsCell.mat'.format(simuName, paramToAnalyze, simuNumber))

figure = plt.figure()
ax = figure.add_subplot(111, projection='3d')
mechNomReward, meshNomProbAT = np.meshgrid(nomReward, nomProbAT)
ax.plot_surface(mechNomReward, meshNomProbAT, totalUtilityAllocation)
ax.set_xlabel('Nominal reward')
ax.set_ylabel('Task success probability')
ax.set_zlabel('Utility')
tikz_save('mat/{}/{}/simu_{}/RewardSuccessProbabilityUtility.tex'.format(simuName, paramToAnalyze, simuNumber))
plt.close(figure)

# NumberAgentsTasks
paramToAnalyze = AllParamsToAnalyze[2]
simuNumber = 3
nAVect = sio.loadmat('mat/{}/{}/simu_{}/nAVect-{}.mat'.format(simuName, paramToAnalyze, simuNumber, simuNumber))
nTVect = sio.loadmat('mat/{}/{}/simu_{}/nTVect-{}.mat'.format(simuName, paramToAnalyze, simuNumber, simuNumber))
totalUtilityAllocation = sio.loadmat('mat/{}/{}/simu_{}/totalUtilityAllocation-{}.mat'.format(simuName, paramToAnalyze, simuNumber, simuNumber))
totalComputationTime = sio.loadmat('mat/{}/{}/simu_{}/totalComputationTime-{}.mat'.format(simuName, paramToAnalyze, simuNumber, simuNumber))
SimuParamsCell = sio.loadmat('mat/{}/{}/simu_{}/SimuParamsCell.mat'.format(simuName, paramToAnalyze, simuNumber))

figure = plt.figure()
ax = figure.add_subplot(111, projection='3d')
mechnAVect, meshnTVect = np.meshgrid(nAVect, nTVect)
ax.plot_surface(mechnAVect, meshnTVect, totalUtilityAllocation)
ax.set_xlabel('Number of agents')
ax.set_ylabel('Number of tasks')
ax.set_zlabel('Utility')
tikz_save('mat/{}/{}/simu_{}/nTnAUtility.tex'.format(simuName, paramToAnalyze, simuNumber))

figure = plt.figure()
ax = figure.add_subplot(111, projection='3d')
ax.plot_surface(mechnAVect, meshnTVect, totalComputationTime)
ax.set_xlabel('Number of agents')
ax.set_ylabel('Number of tasks')
ax.set_zlabel('Computation time')
tikz_save('mat/{}/{}/simu_{}/nTnAComputationTime.tex'.format(simuName, paramToAnalyze, simuNumber))
plt.close(figure)

# TimeRatioLoiteringTasks
paramToAnalyze = AllParamsToAnalyze[3]
simuNumber = 1
ntLoiterVect = sio.loadmat('mat/{}/{}/simu_{}/ntLoiterVect-{}.mat'.format(simuName, paramToAnalyze, simuNumber, simuNumber))
totalUtilityAllocation = sio.loadmat('mat/{}/{}/simu_{}/totalUtilityAllocation-{}.mat'.format(simuName, paramToAnalyze, simuNumber, simuNumber))
totalComputationTime = sio.loadmat('mat/{}/{}/simu_{}/totalComputationTime-{}.mat'.format(simuName, paramToAnalyze, simuNumber, simuNumber))
SimuParamsCell = sio.loadmat('mat/{}/{}/simu_{}/SimuParamsCell.mat'.format(simuName, paramToAnalyze, simuNumber))

figure = plt.figure()
plt.plot(ntLoiterVect, totalComputationTime, linewidth=lineWidth)
plt.ylim([0, max(totalComputationTime)*1.1])
plt.xlabel('Ratio of loitering tasks')
plt.ylabel('Computation time')
tikz_save('mat/{}/{}/simu_{}/ntLoiterComputationTime.tex'.format(simuName, paramToAnalyze, simuNumber))
plt.close(figure)

