import numpy as np
from collections import defaultdict
from GreedyCoalitionAuctionAlgorithm.GCAA_Init import GCAA_Init
from GreedyCoalitionAuctionAlgorithm.GCAA_Main import GCAA_Main
from WinnerVectorToMatrix import WinnerVectorToMatrix
from GreedyCoalitionAuctionAlgorithm.CalcTaskUtility import calc_task_utility


def GCAASolution_revised(Agents, G, TasksCells):
    na = Agents["N"]
    pos_a = Agents["Pos"]
    nt = TasksCells["N"]
    pos_t = TasksCells["Pos"]
    GCAA_Params = GCAA_Init(0, 0, TasksCells["prob_a_t"], TasksCells["lambda"])

    # Define the default agent and task
    agent_default = defaultdict(int)
    task_default = defaultdict(int)

    # Setting the default values
    agent_default["type"] = GCAA_Params["AGENT_TYPES"]["QUAD"]
    agent_default["fuel"] = 1

    task_default["type"] = GCAA_Params["TASK_TYPES"]["TRACK"]

    # Creating the agents and tasks
    agents = [agent_default.copy() for _ in range(na)]
    tasks = [task_default.copy() for _ in range(nt)]

    # Customizing the agents and tasks
    for n in range(na):
        agents[n]["id"] = n
        agents[n]["x"] = pos_a[n, 0]
        agents[n]["y"] = pos_a[n, 1]
        agents[n]["z"] = 0
        agents[n]["v_a"] = Agents["v_a"][n]
        agents[n]["Lt"] = Agents["Lt"][n]
        agents[n]["rin_task"] = []
        agents[n]["vin_task"] = []
        agents[n]["previous_task"] = Agents["previous_task"][n]
        agents[n]["previous_winnerBids"] = Agents["previous_winnerBids"][n]
        agents[n]["kdrag"] = Agents["kdrag"]

    for m in range(nt):
        tasks[m]["id"] = m
        tasks[m]["start"] = 0
        tasks[m]["end"] = 1e20
        tasks[m]["x"] = pos_t[m, 0]
        tasks[m]["y"] = pos_t[m, 1]
        tasks[m]["z"] = 0
        tasks[m]["tf"] = TasksCells["tf"][m]
        tasks[m]["value"] = TasksCells["r_bar"][m]
        tasks[m]["Speed"] = TasksCells["Speed"][m]
        tasks[m]["type"] = TasksCells["task_type"][m]
        tasks[m]["radius"] = TasksCells["radius"][m]
        tasks[m]["tloiter"] = TasksCells["tloiter"][m]

    GCAA_Assignments, S_GCAA_agents, S_GCAA_ALL_agents, agents = GCAA_Main(
        agents, tasks, G, TasksCells["prob_a_t"], TasksCells["lambda"]
    )

    p = [None] * na
    for i in range(na):
        p[i] = GCAA_Assignments[i]["path"]
        ind = [idx for idx, val in enumerate(p) if val == -1]
        if ind:
            p[i] = p[i][: ind[0]]

    print("p: ", p)
    winners = np.zeros(na)
    for i in range(na):
        if p[i]:
            winners[i] = p[i]

    winners_matrix = WinnerVectorToMatrix(na, nt, winners)

    S_GCAA_ALL = np.zeros(nt)
    rt = np.zeros(nt)
    for j in range(nt):
        S_GCAA_ALL[j] = calc_task_utility(
            Agents["Pos"],
            Agents["v_a"],
            TasksCells["Pos"][j, :2],
            TasksCells["Speed"][j],
            TasksCells["tf"][j],
            TasksCells["r_bar"][j],
            j,
            TasksCells["prob_a_t"],
            winners_matrix,
            TasksCells["lambda"],
            Agents["kdrag"],
        )
        rt[j] = TasksCells["r_bar"][j] * (
            1 - np.prod(1 - winners_matrix[:, j] * TasksCells["prob_a_t"][:, j])
        )

    S_GCAA = np.sum(S_GCAA_ALL)
    for i in range(na):
        task_idx = p[i] - 1 if p[i] else None
        if task_idx is None:
            Agents["previous_task"][i] = 0
            Agents["previous_winnerBids"][i] = 0
        else:
            if (
                tasks[task_idx]["tloiter"] > 0
                and (tasks[task_idx]["tf"] - tasks[task_idx]["tloiter"])
                / tasks[task_idx]["tloiter"]
                < 1
            ):
                p[i] = [Agents["previous_task"][i]]
                agents[i]["rin_task"] = []
            else:
                Agents["previous_task"][i] = task_idx
                Agents["previous_winnerBids"][i] = S_GCAA_ALL_agents[i]

    for i in range(na):
        if len(agents[i]["rin_task"]) > 0:
            Agents["rin_task"][i] = agents[i]["rin_task"]
            Agents["vin_task"][i] = agents[i]["vin_task"]

    tmp = np.hstack(p)
    taskInd = TasksCells["Pos"][tmp - 1, 2]

    return S_GCAA, p, taskInd, S_GCAA_ALL, rt, Agents
