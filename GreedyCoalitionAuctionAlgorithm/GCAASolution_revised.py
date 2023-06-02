import numpy as np
from GreedyCoalitionAuctionAlgorithm.GCAA_Init import GCAA_Init
from GreedyCoalitionAuctionAlgorithm.GCAA_Main import GCAA_Main


def GCAASolution_revised(Agents, G, TasksCells):
    na = Agents["N"]
    pos_a = Agents["Pos"]

    nt = TasksCells["N"]
    pos_t = TasksCells["Pos"]

    # Initialize global variables

    # Initialize GCAA parameters
    GCAA_Params = GCAA_Init(0, 0, TasksCells["prob_a_t"], TasksCells["lambda"])

    # Define agents and tasks

    # Define agent default fields
    agent_default = {
        "id": 0,
        "type": 0,
        "avail": 0,
        "clr": [],
        "x": 0,
        "y": 0,
        "z": 0,
        "nom_vel": 0,
        "fuel": 0,
        "Lt": 0,
        "v_a": [0, 0],
        "rin_task": [],
        "vin_task": [],
        "previous_task": [],
        "previous_winnerBids": [],
        "kdrag": 0,
    }

    # Define task default fields
    task_default = {
        "id": 0,
        "type": 0,
        "value": 0,
        "start": 0,
        "end": 0,
        "duration": 0,
        "tf": 0,
        "x": 0,
        "y": 0,
        "z": 0,
        "Speed": 0,
        "radius": 0,
        "tloiter": 0,
    }

    # Create default agents and tasks
    agent_quad = agent_default.copy()
    agent_quad["type"] = GCAA_Params["AGENT_TYPES"]["QUAD"]

    agent_quad["nom_vel"] = 0
    agent_quad["fuel"] = 1

    task_track = task_default.copy()
    task_track["type"] = GCAA_Params["TASK_TYPES"]["TRACK"]

    task_track["value"] = 0
    task_track["duration "] = 0
    # Create random agents
    agents = []
    for n in range(na):
        agent = agent_quad.copy()
        agent["id"] = n + 1
        agent["x"] = pos_a[n, 0]
        agent["y"] = pos_a[n, 1]
        agent["v_a"] = Agents["v_a"][n]
        agent["Lt"] = Agents["Lt"][n]
        agent["rin_task"] = []
        agent["vin_task"] = []
        agent["previous_task"] = Agents["previous_task"][n]
        agent["previous_winnerBids"] = Agents["previous_winnerBids"][n]
        agent["kdrag"] = Agents["kdrag"]
        agents.append(agent)

    # Create random tasks
    tasks = []
    for m in range(nt):
        task = task_track.copy()
        task["id"] = m + 1
        task["x"] = pos_t[m, 0]
        task["y"] = pos_t[m, 1]
        task["z"] = pos_t[m, 2]
        task["tf"] = TasksCells["tf"][m]
        task["value"] = TasksCells["r_bar"][m]
        task["Speed"] = TasksCells["Speed"][m]
        task["type"] = TasksCells["task_type"][m]
        task["radius"] = TasksCells["radius"][m]
        task["tloiter"] = TasksCells["tloiter"][m]
        tasks.append(task)

    # Run GCAA

    # debug here
    GCAA_Assignments, S_GCAA_agents, S_GCAA_ALL_agents, agents = GCAA_Main(
        agents, tasks, G, TasksCells["prob_a_t"], TasksCells["lambda"]
    )

    # Extract assignment paths
    p = []
    for i in range(na):
        path = GCAA_Assignments[i]["path"]
        ind = np.where(path == -1)[0]
        if len(ind) > 0:
            path = path[: ind[0]]

        p.append(path)

    # Extract winners and convert to winner matrix
    winners = np.zeros(na)
    for i in range(na):
        if len(p[i]) > 0:
            winners[i] = p[i][0]
    winners_matrix = winner_vector_to_matrix(na, nt, winners)

    # Calculate scores and rewards
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
            j + 1,
            TasksCells["prob_a_t"],
            winners_matrix,
            TasksCells["lambda"],
            Agents["kdrag"],
        )
        rt[j] = TasksCells["r_bar"][j] * (
            1 - np.prod(1 - winners_matrix[:, j] * TasksCells["prob_a_t"][:, j])
        )

    # Calculate total score
    S_GCAA = np.sum(S_GCAA_ALL)

    # Fix tasks if completion is close
    for i in range(na):
        task_idx = p[i]
        if len(task_idx) == 0:
            Agents["previous_task"][i] = 0
            Agents["previous_winnerBids"][i] = 0
        else:
            task_idx = task_idx[0]
            if (
                tasks[task_idx]["tloiter"] > 0
                and (tasks[task_idx]["tf"] - tasks[task_idx]["tloiter"])
                / tasks[task_idx]["tloiter"]
                < 1
            ):
                p[i] = Agents["previous_task"][i]
                agents[i]["rin_task"] = []
            else:
                Agents["previous_task"][i] = task_idx
                Agents["previous_winnerBids"][i] = S_GCAA_ALL_agents[i]

    # Extract rin_task and vin_task
    rin_task = np.zeros((na, nt))
    vin_task = np.zeros((na, nt))
    for i in range(na):
        rin_task[i] = agents[i]["rin_task"]
        vin_task[i] = agents[i]["vin_task"]

    tmp = np.concatenate(p)
    taskInd = TasksCells["Pos"][tmp, 2]

    return S_GCAA, p, taskInd, S_GCAA_ALL, rt, Agents
