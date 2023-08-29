import numpy as np
from transCoordinate import lla2ecef
import random


def getTaskArea():
    taskArea = np.array(
        [
            [121.66473222, 38.85573388],
            [121.71089122, 38.85573388],
            [121.71940282, 38.82943606],
            [121.71940282, 38.79479375],
            [121.70958175, 38.76925451],
            [121.66505958, 38.76925451],
            [121.65572957, 38.79441445],
            [121.65572957, 38.82791887],
            [121.66473222, 38.85573388],
        ]
    )

    taskPosition = lla2ecef(taskArea)

    return taskPosition


def getAgentInitInfo():
    rawAgentData = np.array(
        [
            [121.65377598, 38.82595636],
            [121.65377598, 38.82305520],
            [121.65377598, 38.81996062],
            [121.65377598, 38.81709814],
            [121.65377598, 38.81431302],
            [121.65377598, 38.81152791],
            [121.65377598, 38.80847201],
            [121.65377598, 38.80557085],
        ]
    )
    agentInitPostion = lla2ecef(rawAgentData)
    return agentInitPostion


def getForbidArea():
    forbidAreaRaw = [
        [
            [121.69149851, 38.84478591],
            [121.70113445, 38.84478591],
            [121.70113445, 38.83903791],
            [121.69149851, 38.83903791],
            [121.69149851, 38.84478591],
        ],
        [
            [121.66891136, 38.7882024],
            [121.67933418, 38.7882024],
            [121.67933418, 38.78209728],
            [121.66891136, 38.78209728],
            [121.66891136, 38.7882024],
        ],
    ]

    forbidArea = []
    for area in forbidAreaRaw:
        if len(area) == 0:
            continue
        area = lla2ecef(np.array(area))
        forbidArea.append(area)
    return forbidArea


def getAgents():
    agents = []
    agentInitPos = getAgentInitInfo()
    for i in range(8):
        agent = {
            "id": i + 1,
            "detectRadius": random.randint(250, 350),
            "detectAngl": sorted([random.randint(0, 180), random.randint(0, 180)]),
            "threatRadius": random.randint(80, 120),
            "sailViteLimit": random.randint(10, 20),
            "certainRadiusLimit": random.randint(10, 20),
            "position": agentInitPos[i].tolist(),
            "velocity": [0, 0],
            "newControl": [0, 0],
            "targets": [],
            "targetsWayPoints": [],
            "wayPoints": [],
            "goals": [],
            "path": [],
            "type": 0,  # 0 is detect state and 1 is catch state
        }
        agents.append(agent)
    return agents


def getInitTaskInfo():
    taskInitInfo = {
        "agents": getAgents(),
        "taskArea": getTaskArea(),
        "forbidArea": getForbidArea(),
    }

    return taskInitInfo
