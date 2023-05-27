## Decentralized Control for Dynamic Task Allocation Problems for Multi-Agent Systems with Auctions
# Analysis of the variation of the parameters
#
# Author: Martin Braquet
# Date: November 2020

import numpy as np
import matplotlib.pyplot as plt
    
def optimalControlParametersAnalysis(): 
    addpath('GreedyCoalitionAuctionAlgorithm/')
    close_('all')
    clear('all')
    rng('default')
    rng(6)
    simuNumber = 3
    simuName = 'Variation of parameters'
    ## Setting of the manual parameters
    
    AllParamsToAnalyze = np.array(['RangeLimitation','RewardSuccessProbability','NumberAgentsTasks','TimeRatioLoiteringTasks'])
    paramToAnalyze = AllParamsToAnalyze[3]
    isUniformAgents = 0
    isUniformTasks = 1
    isPlotAlloc = 0
    isPlotRange = 0
    isCommLimit = 0
    nA = 10
    nT = 20
    ntLoiter = 0
    
    ratioRangeMapWidth = 0.3
    nRounds = 50
    maxInitSpeedAgents = 0.1
    nomReward = 1
    nomProbAT = 0.5
    ## Setting of the fixed parameters
    
    lambda_ = 1
    Lt = 1
    mapWidth = 1
    commDistance = ratioRangeMapWidth * mapWidth
    simuTime = 10
    timeStep = simuTime / nRounds
    posA = (0.1 + 0.8 * np.random.rand(nA,2)) * mapWidth
    posT = (0.1 + 0.8 * np.random.rand(nT,2)) * mapWidth
    tfT = simuTime * np.ones((nT,1))
    tlT = 0.2 * simuTime * np.ones((nT,1))
    taskType = np.zeros((nT,1))
    taskType[np.arange[1,ntLoiter+1]] = 1
    tlT[taskType == 0] = 0
    tlT,idx = __builtint__.sorted(tlT)
    posT = posT(idx,:)
    # Drag force to slow down the agents, final speed set to exp(-3) * vA0
    kdrag = 3 / simuTime
    if isUniformAgents:
        vA = np.zeros((nA,2))
    else:
        vA = (2 * np.random.rand(nA,2) - 1) * maxInitSpeedAgents
    
    maxInitSpeedTasks = 0.1
    if isUniformTasks:
        vT = np.zeros((nT,2))
    else:
        vT = (2 * np.random.rand(nT,2) - 1) * maxInitSpeedTasks
    
    R = 0.04 * mapWidth
    if isUniformTasks:
        radiusT = R * np.ones((nT,1))
    else:
        radiusT = (0.2 * np.random.rand(nT,1) + 1) * R
    
    # Reward after task completion
    if isUniformTasks:
        rBar = nomReward * np.ones((nT,1))
    else:
        rBar = nomReward * np.random.rand(nT,1)
    
    rBar[taskType == 1] = 2 * rBar(taskType == 1)
    # Probability that agent i successfully completes task j
    if isUniformAgents:
        probAT = nomProbAT * np.ones((nA,nT))
    else:
        probAT = np.random.rand(nA,nT)
    
    Tasks.Pos = posT
    Tasks.Speed = vT
    Tasks.N = nT
    Tasks.tf = tfT
    Tasks.lambda = lambda_
    Tasks.task_type = taskType
    Tasks.tloiter = tlT
    Tasks.radius = radiusT
    Tasks.r_bar = rBar
    Tasks.prob_a_t = probAT
    Tasks.task_type = taskType
    Agents.N = nA
    Agents.Lt = Lt * np.ones((1,nA))
    Agents.Pos = posA
    Agents.v_a = vA
    Agents.previous_task = np.zeros((nA,1))
    Agents.previous_winnerBids = np.zeros((nA,1))
    Agents.rin_task = np.zeros((nA,2))
    Agents.vin_task = np.zeros((nA,2))
    Agents.kdrag = kdrag
    # Fully connected graph
    G = not np.eye(Agents.N) 
    if isPlotAlloc:
        figure
        hold('on')
        colors = lines(nA)
        SimuParamsCell.colors = colors
    
    SimuParamsCell.n_rounds = nRounds
    SimuParamsCell.timeStep = timeStep
    SimuParamsCell.mapWidth = mapWidth
    SimuParamsCell.commDistance = commDistance
    SimuParamsCell.simuTime = simuTime
    SimuParamsCell.timeStep = timeStep
    SimuParamsCell.posA = posA
    SimuParamsCell.maxInitSpeedAgents = maxInitSpeedAgents
    SimuParamsCell.vA = vA
    SimuParamsCell.posT = posT
    SimuParamsCell.maxInitSpeedTasks = maxInitSpeedTasks
    SimuParamsCell.vT = vT
    SimuParamsCell.radiusT = radiusT
    SimuParamsCell.taskType = taskType
    SimuParamsCell.ntLoiter = ntLoiter
    SimuParamsCell.taskType = taskType
    SimuParamsCell.na = nA
    SimuParamsCell.nt = nT
    SimuParamsCell.tfT = tfT
    SimuParamsCell.tlT = tlT
    SimuParamsCell.R = R
    SimuParamsCell.radiusT = radiusT
    SimuParamsCell.nomReward = nomReward
    SimuParamsCell.rBar = rBar
    SimuParamsCell.lambda = lambda_
    SimuParamsCell.probAT = probAT
    SimuParamsCell.kdrag = kdrag
    ##
    
    if 'RangeLimitation' == paramToAnalyze:
        analyzeUtilityRangeLimitation()
    else:
        if 'RewardSuccessProbability' == paramToAnalyze:
            analyzeUtilityRewardSuccessProbability()
        else:
            if 'NumberAgentsTasks' == paramToAnalyze:
                analyzeTimeNumberAgentsTasks()
            else:
                if 'TimeRatioLoiteringTasks' == paramToAnalyze:
                    analyzeTimeRatioLoiteringTasks()
    
    ## Main subfunctions
    
    
def analyzeUtilityRangeLimitation(): 
    # Setting of the varying parameters
    ratioRangeMapWidth = np.arange(0,1+0.01,0.01)
    commDistance = ratioRangeMapWidth * mapWidth
    totalUtilityAllocation = np.zeros((len(ratioRangeMapWidth),1))
    for k in np.arange(1,len(ratioRangeMapWidth)+1).reshape(-1):
        if isPlotAlloc:
            plotMapAgentsTasks()
        for i in np.arange(1,nA+1).reshape(-1):
            for j in np.arange((i + 1),nA+1).reshape(-1):
                G[i,j] = norm(posA(i,:) - posA(j,:)) < commDistance(k)
                G[j,i] = G(i,j)
        # GCAA solution
        tic
        S_GCAA,pGCAA,__,__,Agents = GCAASolution(Agents,G,Tasks)
        pGCAA
        toc
        totalUtilityAllocation[k] = S_GCAA
        if isPlotAlloc:
            setPlotAllocation(pGCAA)
    
    mkdir(sprintf('mat/%s/%s/simu_%d',simuName,paramToAnalyze,simuNumber))
    save(sprintf('mat/%s/%s/simu_%d/ratioRangeMapWidth-%d.mat',simuName,paramToAnalyze,simuNumber),'ratioRangeMapWidth')
    save(sprintf('mat/%s/%s/simu_%d/totalUtilityAllocation-%d.mat',simuName,paramToAnalyze,simuNumber),'totalUtilityAllocation')
    save(sprintf('mat/%s/%s/simu_%d/SimuParamsCell.mat',simuName,paramToAnalyze,simuNumber),'SimuParamsCell')
    figure
    plt.plot(ratioRangeMapWidth,totalUtilityAllocation)
    plt.title('Total utility')
    return
    
    
def analyzeUtilityRewardSuccessProbability(): 
    # Setting of the varying parameters
    nomReward = np.arange(0,1+0.1,0.1)
    nomProbAT = np.arange(0,1+0.1,0.1)
    totalUtilityAllocation = np.zeros((len(nomReward),len(nomProbAT)))
    for k in np.arange(2,len(nomReward)+1).reshape(-1):
        for l in np.arange(2,len(nomProbAT)+1).reshape(-1):
            rBar = nomReward(k) * np.ones((nT,1))
            Tasks.r_bar = rBar
            probAT = nomProbAT(l) * np.ones((nA,nT))
            Tasks.prob_a_t = probAT
            if isPlotAlloc:
                plotMapAgentsTasks()
            if isCommLimit:
                setCommunicationLimitation()
            # GCAA solution
            tic
            S_GCAA,pGCAA,__,__,Agents = GCAASolution(Agents,G,Tasks)
            pGCAA
            toc
            totalUtilityAllocation[k,l] = S_GCAA
            if isPlotAlloc:
                setPlotAllocation(pGCAA)
    
    mkdir(sprintf('mat/%s/%s/simu_%d',simuName,paramToAnalyze,simuNumber))
    save(sprintf('mat/%s/%s/simu_%d/nomReward-%d.mat',simuName,paramToAnalyze,simuNumber),'nomReward')
    save(sprintf('mat/%s/%s/simu_%d/nomProbAT-%d.mat',simuName,paramToAnalyze,simuNumber),'nomProbAT')
    save(sprintf('mat/%s/%s/simu_%d/totalUtilityAllocation-%d.mat',simuName,paramToAnalyze,simuNumber),'totalUtilityAllocation')
    save(sprintf('mat/%s/%s/simu_%d/SimuParamsCell.mat',simuName,paramToAnalyze,simuNumber),'SimuParamsCell')
    figure
    mechNomReward,meshNomProbAT = np.meshgrid(nomReward,nomProbAT)
    surf(mechNomReward,meshNomProbAT,totalUtilityAllocation)
    plt.xlabel('nomReward')
    plt.ylabel('nomProbAT')
    plt.title('Total utility')
    return
    
    
def analyzeTimeNumberAgentsTasks(): 
    # Setting of the varying parameters
    nAVect = np.arange(1,80+2,2)
    nTVect = np.arange(1,80+2,2)
    totalUtilityAllocation = np.zeros((len(nAVect),len(nTVect)))
    totalComputationTime = np.zeros((len(nAVect),len(nTVect)))
    for k in np.arange(1,len(nAVect)+1).reshape(-1):
        for l in np.arange(1,len(nTVect)+1).reshape(-1):
            nA = nAVect(k)
            nT = nTVect(l)
            posA = (0.1 + 0.8 * np.random.rand(nA,2)) * mapWidth
            posT = (0.1 + 0.8 * np.random.rand(nT,2)) * mapWidth
            tfT = simuTime * np.ones((nT,1))
            tlT = 0.2 * simuTime * np.ones((nT,1))
            taskType = np.zeros((nT,1))
            taskType[np.arange[1,ntLoiter+1]] = 1
            tlT[taskType == 0] = 0
            tlT,idx = __builtint__.sorted(tlT)
            posT = posT(idx,:)
            vA = np.zeros((nA,2))
            vT = np.zeros((nT,2))
            R = 0.04 * mapWidth
            radiusT = R * np.ones((nT,1))
            rBar = nomReward * np.ones((nT,1))
            probAT = nomProbAT * np.ones((nA,nT))
            G = not np.eye(nA) 
            Tasks.Pos = posT
            Tasks.Speed = vT
            Tasks.N = nT
            Tasks.tf = tfT
            Tasks.lambda = lambda_
            Tasks.task_type = taskType
            Tasks.tloiter = tlT
            Tasks.radius = radiusT
            Tasks.r_bar = rBar
            Tasks.prob_a_t = probAT
            Tasks.task_type = taskType
            Agents.N = nA
            Agents.Lt = Lt * np.ones((1,nA))
            Agents.Pos = posA
            Agents.v_a = vA
            Agents.previous_task = np.zeros((nA,1))
            Agents.previous_winnerBids = np.zeros((nA,1))
            Agents.rin_task = np.zeros((nA,2))
            Agents.vin_task = np.zeros((nA,2))
            if isPlotAlloc:
                plotMapAgentsTasks()
            if isCommLimit:
                setCommunicationLimitation()
            # GCAA solution
            tic
            S_GCAA,pGCAA,__,__,Agents = GCAASolution(Agents,G,Tasks)
            #pGCAA
            totalComputationTime[k,l] = toc
            np.array([k,l])
            totalUtilityAllocation[k,l] = S_GCAA
            if isPlotAlloc:
                setPlotAllocation(pGCAA)
    
    mkdir(sprintf('mat/%s/%s/simu_%d',simuName,paramToAnalyze,simuNumber))
    save(sprintf('mat/%s/%s/simu_%d/nAVect-%d.mat',simuName,paramToAnalyze,simuNumber),'nAVect')
    save(sprintf('mat/%s/%s/simu_%d/nTVect-%d.mat',simuName,paramToAnalyze,simuNumber),'nTVect')
    save(sprintf('mat/%s/%s/simu_%d/totalUtilityAllocation-%d.mat',simuName,paramToAnalyze,simuNumber),'totalUtilityAllocation')
    save(sprintf('mat/%s/%s/simu_%d/totalComputationTime-%d.mat',simuName,paramToAnalyze,simuNumber),'totalComputationTime')
    save(sprintf('mat/%s/%s/simu_%d/SimuParamsCell.mat',simuName,paramToAnalyze,simuNumber),'SimuParamsCell')
    mechnAVect,meshnTVect = np.meshgrid(nAVect,nTVect)
    figure
    surf(mechnAVect,meshnTVect,totalUtilityAllocation)
    plt.xlabel('nA')
    plt.ylabel('nT')
    plt.title('Total utility')
    figure
    surf(mechnAVect,meshnTVect,totalComputationTime)
    plt.xlabel('nA')
    plt.ylabel('nT')
    plt.title('Computation time')
    return
    
    
def analyzeTimeRatioLoiteringTasks(): 
    # Setting of the varying parameters
    ntLoiterVect = np.arange(0,nT+1)
    totalUtilityAllocation = np.zeros((len(ntLoiterVect + 1),1))
    totalComputationTime = np.zeros((len(ntLoiterVect + 1),1))
    for k in np.arange(1,len(ntLoiterVect) + 1+1).reshape(-1):
        ntLoiter = ntLoiterVect(k)
        taskType[np.arange[1,ntLoiter+1]] = 1
        tlT[taskType == 0] = 0
        tlT[taskType == 1] = 2
        Tasks.tloiter = tlT
        Tasks.task_type = taskType
        Agents.previous_task = np.zeros((nA,1))
        Agents.previous_winnerBids = np.zeros((nA,1))
        Agents.rin_task = np.zeros((nA,2))
        Agents.vin_task = np.zeros((nA,2))
        if isPlotAlloc:
            plotMapAgentsTasks()
        if isCommLimit:
            setCommunicationLimitation()
        # GCAA solution
        tic
        S_GCAA,pGCAA,__,__,Agents = GCAASolution(Agents,G,Tasks)
        #pGCAA
        totalComputationTime[k] = toc
        ntLoiter
        totalUtilityAllocation[k] = S_GCAA
        if isPlotAlloc:
            setPlotAllocation(pGCAA)
    
    mkdir(sprintf('mat/%s/%s/simu_%d',simuName,paramToAnalyze,simuNumber))
    save(sprintf('mat/%s/%s/simu_%d/ntLoiterVect-%d.mat',simuName,paramToAnalyze,simuNumber),'ntLoiterVect')
    save(sprintf('mat/%s/%s/simu_%d/totalUtilityAllocation-%d.mat',simuName,paramToAnalyze,simuNumber),'totalUtilityAllocation')
    save(sprintf('mat/%s/%s/simu_%d/totalComputationTime-%d.mat',simuName,paramToAnalyze,simuNumber),'totalComputationTime')
    save(sprintf('mat/%s/%s/simu_%d/SimuParamsCell.mat',simuName,paramToAnalyze,simuNumber),'SimuParamsCell')
    figure
    plt.plot(ntLoiterVect,totalUtilityAllocation)
    plt.title('Total utility')
    figure
    plt.plot(ntLoiterVect,totalComputationTime)
    plt.title('Computation time')
    return
    
    ## Other subfunctions
    
    
def setCommunicationLimitation(): 
    for i in np.arange(1,nA+1).reshape(-1):
        for j in np.arange((i + 1),nA+1).reshape(-1):
            G[i,j] = norm(posA(i,:) - posA(j,:)) < commDistance
            G[j,i] = G(i,j)
    
    return
    
    
def setPlotAllocation(pGCAA = None): 
    # Find the optimal control solution for the given allocation p_GCAA
    X,__,__,__ = OptimalControlSolution(posA,vA,posT,vT,radiusT,pGCAA,Agents,tfT,Tasks.tloiter,timeStep,nRounds,nA,kdrag)
    plotMapAllocation(X,nRounds,nA,colors,'GCAA solution')
    plt.legend(legendUnq(gca))
    drawnow
    return
    
    
def plotMapAgentsTasks(): 
    clf
    hold('on')
    plt.xlim(np.array([0,mapWidth]))
    plt.ylim(np.array([0,mapWidth]))
    plt.xlabel('x [m]')
    plt.ylabel('y [m]')
    plt.title('Task-Agent allocation')
    for i in np.arange(1,nA+1).reshape(-1):
        plt.plot(posA(i,1),posA(i,2),'*','Color',colors(i,:),'MarkerSize',10,'DisplayName','Agents')
    
    plt.plot(posT(:,1),posT(:,2),'rs','MarkerSize',10,'DisplayName','Targets','MarkerFaceColor',np.array([1,0.6,0.6]))
    if isPlotRange:
        PlotAgentRange(posA,commDistance,colors,'Comm Range')
    
    PlotTaskLoitering(posT,radiusT,taskType,'r--','Task loitering')
    return
    
    return
    