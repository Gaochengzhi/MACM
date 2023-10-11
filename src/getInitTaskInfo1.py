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
            [121.670202393184, 38.8305490483220],
            [121.724228956069, 38.8155184167620],
            [121.733656677979, 38.8173018689000],
            [121.679177391814, 38.8417819833486],
            [121.670006306374, 38.8306083751535],
            [121.724131022242, 38.8155481062406],
            [121.733172186335, 38.8169176728800],
            [121.678981245566, 38.8418413540387],
        ]
    )
    # rawAgentData = np.array(
    #     [
    #         [121.670489559397, 38.8307409801551],
    #         [121.724351997866, 38.8145408574218],
    #         [121.734177521274, 38.8177148844954],
    #         [121.679501031313, 38.8426840213227],
    #         [121.670489559397, 38.8317409801551],
    #         [121.724351997866, 38.8155408574218],
    #         [121.734177521274, 38.8167148844954],
    #         [121.679501031313, 38.8426840213227],
    #     ]
    # )
    agentInitPostion = lla2ecef(rawAgentData)
    return agentInitPostion


def getForbidArea():
    forbidAreaRaw = []

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
