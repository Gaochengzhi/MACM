import numpy as np
from GreedyCoalitionAuctionAlgorithm.CalcUtility import calc_utility


def gcaa_bundle_add(GCAA_Params, GCAA_Data, agent, tasks, agent_idx):
    if GCAA_Data["fixedAgents"][agent_idx] == 1:
        return GCAA_Data, agent

    rin_t = None
    vin_t = None
    M = len(tasks)
    task_pos = [[task["x"], task["y"]] for task in tasks]
    task_v = [task["Speed"] for task in tasks]
    task_tf = [task["tf"] for task in tasks]
    task_tloiter = [task["tloiter"] for task in tasks]
    task_radius = [task["radius"] for task in tasks]
    task_type = [task["type"] for task in tasks]
    task_value = [task["value"] for task in tasks]

    U = -1e14
    b = []

    winners_matrix = [[0] * GCAA_Params["M"] for _ in range(GCAA_Params["N"])]
    for i in range(GCAA_Params["N"]):
        if GCAA_Data["winners"][i] > 0:
            winners_matrix[i][GCAA_Data["winners"][i]] = 1

    # Only pick a task that is not assigned yet
    availTasks = [
        j for j in range(1, GCAA_Params["M"] + 1) if j not in GCAA_Data["winners"]
    ]

    # If all tasks are assigned, pick any task with positive utility
    if not availTasks:
        availTasks = list(range(1, M + 1))
        allTasksAssigned = True
        U = 0

    newRin = False
    for j in availTasks:
        if task_tf[j - 1] > task_tloiter[j - 1]:
            b_new = j

            winners_matrix[agent_idx][:] = [0] * GCAA_Params["M"]
            winners_matrix[agent_idx][j - 1] = 1
            rin_t_new, vin_t_new, U_new = calc_utility(
                [agent["x"], agent["y"]],
                agent["v_a"],
                task_pos,
                task_v,
                task_type,
                task_radius,
                task_tloiter,
                task_tf,
                task_value,
                b_new,
                agent_idx,
                GCAA_Params["prob_a_t"],
                GCAA_Params["N"],
                winners_matrix,
                GCAA_Params["lambda"],
                agent["kdrag"],
            )

            if U_new > U:
                U = U_new
                b = b_new
                rin_t = rin_t_new
                vin_t = vin_t_new
                newRin = True

    GCAA_Data["path"] = b
    GCAA_Data["winnerBids"][agent_idx] = U
    if not b:
        b = 0
    GCAA_Data["winners"][agent_idx] = b

    if newRin:
        agent["rin_task"] = rin_t
        agent["vin_task"] = vin_t

    return GCAA_Data, agent
