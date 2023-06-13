import numpy as np
from scipy.optimize import linear_sum_assignment


def assign_tasks(Agents, TasksCells):
    agent_num = Agents["N"]
    agent_position = Agents["Pos"]
    task_num = TasksCells["N"]
    task_position = TasksCells["Pos"]

    # Calculate distance matrix
    cost_matrix = np.zeros((agent_num, task_num))
    for i in range(agent_num):
        for j in range(task_num):
            cost_matrix[i, j] = np.linalg.norm(agent_position[i] - task_position[j, :2])

    # Assign tasks to agents using Hungarian algorithm
    row_ind, col_ind = linear_sum_assignment(cost_matrix)

    return col_ind.tolist()  # The tasks assigned to each agent


def GCAASolution_revised(Agents, G, TasksCells):
    agent_num = Agents["N"]
    agent_position = Agents["Pos"]
    task_num = TasksCells["N"]
    task_position = TasksCells["Pos"]
    p = assign_tasks(Agents, TasksCells)
    p = [i + 1 for i in p]

    # Agents["previous_task"] = np.array([])

    # for i in range(na):
    #     if len(agents[i]["rin_task"]) > 0:
    #         Agents["rin_task"][i] = agents[i]["rin_task"]
    #         Agents["vin_task"][i] = agents[i]["vin_task"]
    # p = [1, 3, 8, 10, 13, 18, 22, 24]

    # for i in range(agent_num):
    #     task_idx = p[i] - 1 if p[i] else None
    #     if task_idx is None:
    #         Agents["previous_task"][i] = 0
    #         Agents["previous_winnerBids"][i] = 0
    tmp = np.hstack(p)
    taskInd = TasksCells["Pos"][tmp - 1, 2]

    return p, taskInd, Agents
