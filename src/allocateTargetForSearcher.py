import numpy as np
import sys
from updateCatchWayPoint import updateCatchWayPoint
from fillBetween import distance



def deleteGoalsBetweenPosAndAgent(agent, controlPos):
    position = agent["position"]
    goals = agent["goals"]
    dist_to_control = distance(position, controlPos)
    filtered_goals = [
        goal
        for goal in goals
        if distance(position, goal) >= dist_to_control
        or np.array_equal(goal, goals[-1])
        or np.array_equal(goal, goals[0])
    ]

    return filtered_goals


def allocateTargetForSearcher(
    catchAgents, targetSearched, forbidArea, taskArea, centralPoints
):
    num_agents = len(catchAgents)
    num_targets = len(targetSearched)

    agent_positions = {agent["id"]: agent["position"] for agent in catchAgents}
    target_positions = {target["id"]: target["position"] for target in targetSearched}

    target_counter = {target["id"]: 0 for target in targetSearched}

    distances = np.zeros((num_agents, num_targets))

    for i in range(num_agents):
        for j in range(num_targets):
            if catchAgents[i]["id"] not in targetSearched[j]["forbidenAgents"]:
                distances[i][j] = distance(
                    agent_positions[catchAgents[i]["id"]],
                    target_positions[targetSearched[j]["id"]],
                )
            else:
                distances[i][j] = sys.maxsize

    agent_assignments = np.zeros(num_agents, dtype=int)

    while np.min(distances) != sys.maxsize:
        min_dist = np.min(distances)
        agent_idx, target_idx = np.where(distances == min_dist)
        agent_idx = agent_idx[0]
        target_idx = target_idx[0]

        target_id = targetSearched[target_idx]["id"]
        allocated_time = targetSearched[target_idx]["allocatedTime"]

        if target_counter[target_id] < allocated_time:
            agent_assignments[agent_idx] = target_idx
            catchAgents[agent_idx]["targets"].append(targetSearched[target_idx])
            target_counter[target_id] += 1
            targetSearched[target_idx]["allocatedTime"] -= 1

            updateCatchWayPoint(
                catchAgents[agent_idx],
                [targetSearched[target_idx]],
                forbidArea,
                taskArea,
                100,
                centralPoints,
            )
            if len(catchAgents[agent_idx]["goals"]) != 0:
                catchAgents[agent_idx]["goals"] = deleteGoalsBetweenPosAndAgent(
                    catchAgents[agent_idx], targetSearched[target_idx]["position"]
                )
        else:
            distances[:, target_idx] = sys.maxsize
            continue

        distances[agent_idx, :] = sys.maxsize

    return catchAgents

# Test the function
# catchAgents = [
#     {"id": 1, "position": [0, 0], "targets": []},
#     {"id": 2, "position": [0, 1], "targets": []},
#     {"id": 3, "position": [1, 0], "targets": []},
# ]
# targetSearched = [
#     {"id": 1, "position": [3, 4], "allocatedTime": 2, "forbidenAgents": []},
#     {"id": 2, "position": [5, 6], "allocatedTime": 1, "forbidenAgents": []},
# ]
# result = allocateTargetForSearcher(catchAgents, targetSearched)

# print(result)
