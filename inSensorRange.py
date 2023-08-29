import numpy as np


def inSensorRange(agent, p_obstacle):
    agent_position = np.array(agent["position"])
    distance = np.sum((agent_position - p_obstacle) ** 2) < agent["detectRadius"] ** 2

    return distance
