import numpy as np
from itertools import combinations


def OptimumSolution(Agents, G, Tasks):
    na = Agents.N
    Lt = Agents.Lt[0]
    pos_a = Agents.Pos
    
    nt = Tasks.N
    pos_t = Tasks.Pos
    
    b = [[] for _ in range(na)]
    
    Nmin = min(nt, Lt * na)

    I = np.arange(na)
    J = np.arange(nt)

    S_opt = 0

    if Nmin == nt:
        list_b = AllAllocComb_Nt(1, J, [], b)
    else:
        list_b = AllAllocComb_NuLt(1, J, [], b)

    all_S = [None] * len(list_b)
    all_p = [None] * len(list_b)

    for l in range(len(list_b)):
        b_curr = list_b[l]
        
        stop = 0
        for i in range(na):
            if len(b_curr[i]) > 1:
                stop = 1
        if stop:
            continue
        
        winners = np.zeros((na, nt))
        for i in range(na):
            for j in range(nt):
                winners[i, j] = (sum(b_curr[i] == j) > 0)
        S, all_scores = ComputeScore(na, b_curr, winners)
        all_S[l] = S
        all_p[l] = b_curr
        if S > S_opt:
            S_opt = S
            p_opt = b_curr
            S_opt_all = all_scores

    def AllAllocComb_Nt(i, J, list_b, b):
        if i == na + 1:
            list_b.append(b.copy())
        else:
            for k in range(Lt + 1):
                allcombs = list(combinations(J, k))
                for l in range(len(allcombs)):
                    b[i - 1] = allcombs[l]
                    J_new = [j for j in J if j not in allcombs[l]]
                    list_b = AllAllocComb_Nt(i + 1, J_new, list_b, b)
        return list_b

    def AllAllocComb_NuLt(i, J, list_b, b):
        if i > na:
            list_b.append(b.copy())
        else:
            allcombs = list(combinations(J, Lt))
            for l in range(len(allcombs)):
                b[i - 1] = allcombs[l]
                J_new = [j for j in J if j not in allcombs[l]]
                list_b = AllAllocComb_NuLt(i + 1, J_new, list_b, b)
        return list_b

    def ComputeScore(nt, b, winners):
        S_j = np.zeros(nt)
        for j in range(nt):
            S_j[j] = CalcTaskUtility(Agents.Pos, Agents.v_a, Tasks.Pos[j], Tasks.tf[j], Tasks.r_bar[j], j, Tasks.prob_a_t, winners)
        S_new = np.sum(S_j)
        return S_new, S_j

    return S_opt, p_opt, S_opt_all

