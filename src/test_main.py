import numpy as np
import copy
import warnings
import math
import random

# Start listener in a separate thread


warnings.filterwarnings("ignore")
import matplotlib.pyplot as plt
from getControls import getControls
from inSensorRange import inSensorRange
from divdeStrip import divide_into_stripes, smoothAndAvoidObstacles, find_central_point
from fillBetween import fillbetween, distance

from addTargets import addTargets
from getInitTaskInfo import getInitTaskInfo
from getFakeTarget import getFakeTarget
from transCoordinate import transform_coordinates_for_plot, ecef2lla, lla2ecef
from updateCatchWayPoint import updateCatchWayPoint

from recieveAndSendInfo import getNewTarget
from transCoordinate import transform_coordinates_for_plot
from allocateTargetForSearcher import (
    allocateTargetForSearcher,
    deleteGoalsBetweenPosAndAgent,
)
import threading
import time


colorline = ["red", "green", "blue", "yellow", "black", "pink", "orange", "purple"]


def inCatchRange(agent, target):
    agent_position = np.array(agent["position"])
    distance = (
        np.sum((agent_position - target["position"]) ** 2)
        < (target["certainRadius"] + 50) ** 2
    )
    return distance


def findTargets(agent, targetSearched, fakeTargets):
    targetsGet = getNewTarget(agent, targetSearched, fakeTargets)
    if len(targetsGet) > 0:
        for target in targetsGet:
            target["allocatedTime"] -= 1
            newTarget = copy.deepcopy(target)
            targetSearched.append(target)
            agent["targets"].append(newTarget)
            updateCatchWayPoint(
                agent,
                [newTarget],
                forbidArea,
                taskArea,
                arrive_distance + 100,
                centralPoints,
            )
            # deleteGoalsBetweenPosAndAgent(agent, newTarget["position"])
        return True
    return False


class SetRestCheckTimer:
    def __init__(self, target, agent, rest_time):
        self.target = target
        self.agent = agent
        self.rest_time = rest_time
        self.lock = threading.Lock()
        self.timer_thread = threading.Thread(target=self._start_timer)

    def _start_timer(self):
        time.sleep(self.rest_time)
        with self.lock:
            if self.agent["id"] in self.target["forbidenAgents"]:
                self.target["forbidenAgents"].remove(self.agent["id"])

    def start(self):
        with self.lock:
            if self.agent["id"] not in self.target["forbidenAgents"]:
                self.target["forbidenAgents"].append(self.agent["id"])
        self.timer_thread.start()


def TargetWayPointFinished(agent, targetId):
    for targetWayPoint in agent["targetsWayPoints"]:
        if targetWayPoint["id"] == targetId:
            if len(targetWayPoint["targetPath"]) < 5:
                return True
    return False


def updateTargetRestCheck(agent, targetId, globalTargets):
    for target in globalTargets:
        if target["id"] == targetId:
            if target["restCheck"] > 1:
                target["restCheck"] -= 1
                restTime = target["restTime"]
                restTimer = SetRestCheckTimer(target, agent, restTime)
                print("targetId: ", targetId, " restTime: ", restTime)
                restTimer.start()
                target["allocated"] = False
            elif target["restCheck"] == 1:
                target["restCheck"] -= 1
                target["finished"] = True


def getControlPosition(agent, dt):
    position = agent["position"] + np.array(agent["newControl"])
    return position


def calculate_central_points(forbiddenAreas, margin_distance=3):
    central_points = []
    for forbidArea in forbiddenAreas:
        central_point = np.mean(forbidArea, axis=0)
        central_points.append(central_point)
    return np.array(central_points)


InitInfo = getInitTaskInfo()
taskArea = InitInfo["taskArea"]
allAgents = InitInfo["agents"]
forbidArea = InitInfo["forbidArea"]

p_agent = np.array([agent["position"] for agent in allAgents])
p_all = np.concatenate((taskArea, p_agent))
deviation_x = np.min(p_all[:, 0])
deviation_y = np.min(p_all[:, 1])

p_fask_targets = getFakeTarget()
p_fask_targets = transform_coordinates_for_plot(
    p_fask_targets, deviation_x, deviation_y
)
fakeTargets = []
for i in range(len(p_fask_targets)):
    fakeTargets.append(addTargets(str(i + 1), p_fask_targets[i], [0, 0]))


p_taskArea = transform_coordinates_for_plot(taskArea, deviation_x, deviation_y)
p_agent = transform_coordinates_for_plot(p_agent, deviation_x, deviation_y)
for area in forbidArea:
    transform_coordinates_for_plot(area, deviation_x, deviation_y)
for i in range(len(allAgents)):
    allAgents[i]["position"] = p_agent[i]

for i in range(len(forbidArea)):
    if i == len(forbidArea) - 1:
        forbidArea[i][0], forbidArea[i][1] = (
            forbidArea[i][0] + 5,
            forbidArea[i][1] - 5,
        )

taskCenter = find_central_point(taskArea)
centralPoints = calculate_central_points(forbidArea)

## paramters
n_searcher = len(allAgents)
arrive_distance = 20
fillDistance = 100
# get n_searcher biggest agent beaed on detectRadius
detectAgents = sorted(allAgents, key=lambda x: x["detectRadius"], reverse=True)[
    :n_searcher
]
# catchAgents = rest of agents  aside from detectagent
catchAgents = sorted(allAgents, key=lambda x: x["detectRadius"], reverse=True)[
    n_searcher:
]
detectRadius = np.array([agent["detectRadius"] for agent in detectAgents])
detectVelocity = np.array([agent["sailViteLimit"] for agent in detectAgents])
detectRadius = np.append(detectRadius[-1], detectRadius[:-1])
detectVelocity = np.append(detectVelocity[-1], detectVelocity[:-1])


wayPoints = divide_into_stripes(p_taskArea, detectAgents, detectRadius, detectVelocity)


for i in range(len(detectAgents)):
    detectAgents[i]["wayPoints"] = smoothAndAvoidObstacles(
        wayPoints[i], detectAgents[i]["position"], forbidArea, p_taskArea
    )
# plot all wayPoints
plt.plot(taskArea[:, 0], taskArea[:, 1], "b")
plt.fill(taskArea[:, 0], taskArea[:, 1], alpha=0.1, color="blue")
for area in forbidArea:
    plt.plot(area[:, 0], area[:, 1], "r")
    plt.fill(area[:, 0], area[:, 1], alpha=0.1, color="red")
for i in range(len(detectAgents)):
    # plt detectAgents[i]["wayPoints"]
    for wayPoint in detectAgents[i]["wayPoints"]:
        plt.plot(wayPoint[0], wayPoint[1], "o", color=colorline[i], markersize=2)
        # plt.pause(0.03)


# plt all fakeTargets
for fakeTarget in fakeTargets:
    plt.plot(fakeTarget["position"][0], fakeTarget["position"][1], "x", color="red")
    plt.text(
        fakeTarget["position"][0] + 50,
        fakeTarget["position"][1] + 20,
        str(fakeTarget["id"]),
        color="red",
    )
# plt.show()
# exit()
init_test_goals = [
    {
        "id": agent["id"],
        "wayPoints": ecef2lla(
            transform_coordinates_for_plot(
                np.array(agent["wayPoints"]), deviation_x, deviation_y, False
            )
        ),
    }
    for agent in detectAgents
]
dt = 2
targetSearched = []
agents_count = len(allAgents)
Finished = False
while not Finished:
    # Stage1 allocate wayPoints and TargetWayPoints to searcher and catcher
    for agent in detectAgents:
        # avoid last point is not removed
        if len(agent["wayPoints"]) == 0:
            agent["goals"] = []

        if len(agent["wayPoints"]) == 0 and len(agent["targetsWayPoints"]) == 0:
            detectAgents.remove(agent)
            catchAgents.append(agent)

    # catcher algorithem
    freeAgents = [agent for agent in catchAgents if len(agent["targets"]) == 0]
    freeTargets = [
        target
        for target in targetSearched
        if target["allocatedTime"] > 0 and not target["finished"]
    ]

    if len(freeTargets) > 0 and len(freeAgents) > 0:
        allocateTargetForSearcher(
            freeAgents, freeTargets, forbidArea, taskArea, centralPoints
        )

    # Stage2 get and catch target
    for agent in allAgents:
        if findTargets(agent, targetSearched, fakeTargets):
            pass
        for target in targetSearched:
            if (
                inSensorRange(agent, target["position"])
                and target["id"] not in [targ["id"] for targ in agent["targets"]]
                and not target["finished"]
                and target["allocatedTime"] > 0
                and agent["id"] not in target["forbidenAgents"]
            ):
                target["allocatedTime"] -= 1
                newTarget = copy.deepcopy(target)
                agent["targets"].append(newTarget)
                updateCatchWayPoint(
                    agent,
                    [newTarget],
                    forbidArea,
                    taskArea,
                    arrive_distance + 100,
                    centralPoints,
                )
                deleteGoalsBetweenPosAndAgent(agent, newTarget["position"])

        agent["type"] = 0
        for target in agent["targets"]:
            # slow down
            if inCatchRange(agent, target):
                agent["type"] = 1
            if TargetWayPointFinished(agent, target["id"]):
                updateTargetRestCheck(agent, target["id"], targetSearched)
                agent["targets"].remove(target)
                agent["targetsWayPoints"] = [
                    targetWayPoint
                    for targetWayPoint in agent["targetsWayPoints"]
                    if targetWayPoint["id"] != target["id"]
                ]
                # break

    # cooperate obstacle avoidence
    for i in range(agents_count):
        obstacles_for_agent_i = []

        for k in range(agents_count):
            if i != k and inSensorRange(allAgents[i], allAgents[k]["position"]):
                if all(
                    allAgents[k]["id"] != obstacle["id"]
                    for obstacle in obstacles_for_agent_i
                ):
                    obstacles_for_agent_i.append(allAgents[k])

        for k in range(len(targetSearched)):
            if targetSearched[k]["finished"]:
                continue
            if inSensorRange(allAgents[i], targetSearched[k]["position"]):
                if all(
                    targetSearched[k]["id"] != obstacle["id"]
                    for obstacle in obstacles_for_agent_i
                ):
                    obstacles_for_agent_i.append(targetSearched[k])
        if len(allAgents[i]["goals"]) > 0 or len(allAgents[i]["targetsWayPoints"]) > 0:
            # agentGoal = [0, 0] + allAgents[i]["position"]
            if len(allAgents[i]["targetsWayPoints"]) != 0:
                agentGoal = allAgents[i]["targetsWayPoints"][0]["targetPath"][0]
            elif len(allAgents[i]["goals"]) > 0:
                agentGoal = allAgents[i]["goals"][0]
            else:
                # randomPos = [random.randint(-30, 30), random.randint(20, 30)]
                # agentGoal = allAgents[i]["position"] + randomPos
                pass

        else:
            # agentGoal = taskArea[i % len(taskArea)]
            agentGoal = allAgents[i]["position"]

        allAgents[i]["newControl"] = getControls(
            allAgents[i], obstacles_for_agent_i, dt, forbidArea, taskArea, agentGoal
        )

    # Stage3 update allAgents position
    for agent in allAgents:
        controlPos = getControlPosition(agent, dt)
        agent["velocity"] = agent["newControl"]
        if len(agent["targetsWayPoints"]) != 0:
            targetWaypointObj = agent["targetsWayPoints"][0]
            if len(targetWaypointObj["targetPath"]) > 0:
                # agent["position"] = targetWaypointObj["targetPath"][0]
                # agent["position"] = controlPos
                if (
                    distance(agent["position"], targetWaypointObj["targetPath"][0])
                    < arrive_distance
                ):
                    agent["targetsWayPoints"][0]["targetPath"] = agent[
                        "targetsWayPoints"
                    ][0]["targetPath"][1:]
        elif len(agent["goals"]) < 1:
            if len(agent["wayPoints"]) != 0:
                entryPoint = agent["wayPoints"][0]
                agent["goals"] = fillbetween(
                    agent["position"],
                    entryPoint,
                    forbidArea,
                    taskArea,
                    centralPoints,
                    fillDistance,
                )
                if not np.array_equal(agent["velocity"], np.array([0, 0])):
                    # agent["goals"] = deleteGoalsBetweenPosAndAgent(agent, controlPos)
                    pass
                agent["wayPoints"] = agent["wayPoints"][1:]
        elif len(agent["goals"]) > 0:
            # agent["position"] = agent["goals"][0]
            if distance(agent["position"], agent["goals"][0]) < arrive_distance:
                agent["goals"] = agent["goals"][1:]
        agent["position"] = controlPos
        agent["path"].append(agent["position"])
    plt.clf()
    plt.plot(taskArea[:, 0], taskArea[:, 1], "b")
    plt.fill(taskArea[:, 0], taskArea[:, 1], alpha=0.1, color="blue")
    for area in forbidArea:
        plt.plot(area[:, 0], area[:, 1], "r")
        plt.fill(area[:, 0], area[:, 1], alpha=0.1, color="red")

    for i in range(len(allAgents)):
        plt.plot(
            allAgents[i]["position"][0],
            allAgents[i]["position"][1],
            "*",
            color=colorline[i],
            markersize=10,
            alpha=0.5,
            label="Agents",
        )
        if len(allAgents[i]["goals"]) > 0:
            plt.plot(
                np.array(allAgents[i]["goals"])[:, 0],
                np.array(allAgents[i]["goals"])[:, 1],
                ".",
                color=colorline[i],
                markersize=2,
                alpha=0.59,
                label="goals",
            )
        if len(allAgents[i]["targetsWayPoints"]) > 0:
            plt.plot(
                np.array(allAgents[i]["targetsWayPoints"][0]["targetPath"])[:, 0],
                np.array(allAgents[i]["targetsWayPoints"][0]["targetPath"])[:, 1],
                ".",
                color=colorline[i],
                markersize=2,
                alpha=0.59,
                label="targetsWayPoints",
            )
    # plot all path in agent
    for i in range(len(allAgents)):
        plt.plot(
            np.array(allAgents[i]["path"])[:, 0],
            np.array(allAgents[i]["path"])[:, 1],
            "-",
            alpha=0.2,
            color=colorline[i],
            linewidth=1,
        )

        # plt.text(
        #     allAgents[i]["position"][0] + 50,
        #     allAgents[i]["position"][1] + 150,
        #     "goals:" + str(len(allAgents[i]["goals"])),
        #     color=colorline[i],
        # )
        # plt.text(
        #     allAgents[i]["position"][0] + 50,
        #     allAgents[i]["position"][1] - 150,
        #     "t:" + str(len(allAgents[i]["targetsWayPoints"])),
        #     color=colorline[i],
        # )
        # if len(allAgents[i]["targetsWayPoints"]) > 0:
        #     plt.text(
        #         allAgents[i]["position"][0] + 100,
        #         allAgents[i]["position"][1] + 100,
        #         "tid:" + str((allAgents[i]["targetsWayPoints"][0]["id"])),
        #         color=colorline[i],
        #     )
        plt.plot(
            np.array(allAgents[i]["path"])[:, 0],
            np.array(allAgents[i]["path"])[:, 1],
            color=(0, 1, 0, 0.3),
            linewidth=allAgents[i]["detectRadius"] / 25,
        )
    for target in targetSearched:
        if not target["finished"]:
            plt.plot(target["position"][0], target["position"][1], "x", color="red")
            plt.text(
                target["position"][0],
                target["position"][1] - 150,
                str(target["restCheck"]),
                color="blue",
            )

    allWayPointRemoved = all(len(agent["wayPoints"]) == 0 for agent in allAgents)

    allTargetFinished = all([target["finished"] for target in targetSearched])
    Finished = allTargetFinished and allWayPointRemoved

    # top left plt bassed on dynamic axix
    # plt.text(top, left, "targetSearched: " + str(len(targetSearched)))
    plt.text(
        0,
        1,
        "targetSearched: " + str(len(targetSearched)),
        color="blue",
        verticalalignment="top",
        horizontalalignment="left",
        transform=plt.gca().transAxes,
    )
    if Finished:
        plt.text(
            0,
            0,
            "Finished",
            color="red",
            verticalalignment="top",
            horizontalalignment="left",
            transform=plt.gca().transAxes,
        )

        plt.pause(1)
    plt.axis("equal")
    plt.pause(0.0001)
