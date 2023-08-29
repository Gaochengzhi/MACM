import numpy as np
import sys


def calculate_distance(p1, p2):
    return np.linalg.norm(np.array(p1) - np.array(p2))


def allocateTargetForSearcher(catchAgents, targetSearched):
    num_agents = len(catchAgents)
    num_targets = len(targetSearched)

    agent_positions = {agent["id"]: agent["position"] for agent in catchAgents}
    target_positions = {target["id"]: target["position"] for target in targetSearched}

    distances = np.zeros((num_agents, num_targets))
    for i in range(num_agents):
        for j in range(num_targets):
            if catchAgents[i]["id"] not in targetSearched[j]["forbidenAgents"]:
                distances[i][j] = calculate_distance(
                    agent_positions[catchAgents[i]["id"]],
                    target_positions[targetSearched[j]["id"]],
                )
            else:
                distances[i][j] = sys.maxsize

    agent_assignments = np.zeros(num_agents, dtype=int)
    target_assignments = np.zeros(num_targets, dtype=int)

    for _ in range(min(num_agents, num_targets)):
        min_dist = np.min(distances)
        agent_idx, target_idx = np.where(distances == min_dist)
        agent_idx = agent_idx[0]
        target_idx = target_idx[0]

        agent_assignments[agent_idx] = target_idx
        target_assignments[target_idx] = agent_idx

        distances[agent_idx, :] = sys.maxsize
        distances[:, target_idx] = sys.maxsize

    for agent_idx, target_idx in enumerate(agent_assignments):
        if target_idx < num_targets:
            target = targetSearched[target_idx]
            catchAgents[agent_idx]["targets"].append(target)

    return catchAgents


# Example usage
# catchAgents = [
#     {"id": "agent1", "position": [0, 0], "targetsWayPoints": [], "targets": []},
#     {"id": "agent2", "position": [1, 1], "targetsWayPoints": [], "targets": []},
#     # ... other agents
# ]

# targetSearched = [
#     {"id": "car119", "position": [2, 2], "forbidenAgents": ["agent1"]},
#     {"id": "car120", "position": [3, 3], "forbidenAgents": []},
#     # ... other targets
# ]

# allocated_agents, allocation_result = allocateTargetForSearcher(
#     catchAgents, targetSearched
# )
# print(catchAgents)
