import numpy as np


def gcaa_solution(agents, G, tasks_cells):
    na = agents["N"]
    pos_a = agents["Pos"]

    nt = tasks_cells["N"]
    pos_t = tasks_cells["Pos"]

    # Initialize global variables

    # Initialize GCAA parameters
    GCAA_Params = gcaa_init(0, 0, tasks_cells["prob_a_t"], tasks_cells["lambda"])

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

    task_track = task_default.copy()
    task_track["type"] = GCAA_Params["TASK_TYPES"]["TRACK"]

    # Create random agents
    agents_list = []
    for n in range(na):
        agent = agent_quad.copy()
        agent["id"] = n + 1
        agent["x"] = pos_a[n, 0]
        agent["y"] = pos_a[n, 1]
        agent["v_a"] = agents["v_a"][n]
        agent["Lt"] = agents["Lt"][n]
        agent["rin_task"] = []
        agent["vin_task"] = []
        agent["previous_task"] = agents["previous_task"][n]
        agent["previous_winnerBids"] = agents["previous_winnerBids"][n]
        agent["kdrag"] = agents["kdrag"]
        agents_list.append(agent)

    # Create random tasks
    tasks_list = []
    for m in range(nt):
        task = task_track.copy()
        task["id"] = m + 1
        task["x"] = pos_t[m, 0]
        task["y"] = pos_t[m, 1]
        task["tf"] = tasks_cells["tf"][m]
        task["value"] = tasks_cells["r_bar"][m]
        task["Speed"] = tasks_cells["Speed"][m]
        task["type"] = tasks_cells["task_type"][m]
        task["radius"] = tasks_cells["radius"][m]
        task["tloiter"] = tasks_cells["tloiter"][m]
        tasks_list.append(task)

    # Run GCAA
    GCAA_Assignments, S_GCAA_agents, S_GCAA_ALL_agents, agents_list = gcaa_main(
        agents_list, tasks_list, G, tasks_cells["prob_a_t"], tasks_cells["lambda"]
    )

    p = []
    for i in range(na):
        path = GCAA_Assignments[i]["path"]
        ind = np.where(path == -1)[0]
        if len(ind) > 0:
            path = path[: ind[0]]
        p.append(path.tolist())

    winners = np.zeros(na)
    for i in range(na):
        if len(p[i]) > 0:
            winners[i] = p[i][0]

    winners_matrix = winner_vector_to_matrix(na, nt, winners)

    S_GCAA_ALL = np.zeros(nt)
    rt = np.zeros(nt)
    for j in range(nt):
        S_GCAA_ALL[j] = calc_task_utility(
            agents["Pos"],
            agents["v_a"],
            tasks_cells["Pos"][j],
            tasks_cells["Speed"][j],
            tasks_cells["tf"][j],
            tasks_cells["r_bar"][j],
            j + 1,
            tasks_cells["prob_a_t"],
            winners_matrix,
            tasks_cells["lambda"],
            agents["kdrag"],
        )
        rt[j] = tasks_cells["r_bar"][j] * (
            1 - np.prod(1 - winners_matrix[:, j] * tasks_cells["prob_a_t"][:, j])
        )

    S_GCAA = np.sum(S_GCAA_ALL)

    # Fix the tasks if the completion is close
    for i in range(na):
        task_idx = p[i]
        if len(task_idx) == 0:
            agents["previous_task"][i] = 0
            agents["previous_winnerBids"][i] = 0
        else:
            task_idx = task_idx[0]
            if (
                tasks_list[task_idx]["tloiter"] > 0
                and (tasks_list[task_idx]["tf"] - tasks_list[task_idx]["tloiter"])
                / tasks_list[task_idx]["tloiter"]
                < 1
            ):
                p[i] = agents["previous_task"][i]
                agents_list[i]["rin_task"] = []
            else:
                agents["previous_task"][i] = task_idx
                agents["previous_winnerBids"][i] = S_GCAA_ALL_agents[i]

    rin_task = np.zeros((na, nt))
    vin_task = np.zeros((na, nt))
    for i in range(na):
        rin_task[i] = agents_list[i]["rin_task"]
        vin_task[i] = agents_list[i]["vin_task"]

    return (
        p,
        S_GCAA,
        S_GCAA_ALL,
        rt,
        {
            "Pos": agents["Pos"],
            "v_a": agents["v_a"],
            "Lt": agents["Lt"],
            "previous_task": agents["previous_task"],
            "previous_winnerBids": agents["previous_winnerBids"],
            "rin_task": rin_task,
            "vin_task": vin_task,
        },
    )
