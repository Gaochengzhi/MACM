import numpy as np
import warnings

warnings.filterwarnings("ignore")
import matplotlib.pyplot as plt
from getControls import getControls
from inSensorRange import inSensorRange
from divdeStrip import divide_into_stripes, smoothAndAvoidObstacles, filter_waypoints
from fillBetween import fillbetween, distance

from addTargets import addTargets
from getInitTaskInfo import getInitTaskInfo
from getFakeTarget import getFakeTarget
from transCoordinate import transform_coordinates_for_plot
from updateCatchWayPoint import updateCatchWayPoint

from recieveAndSendInfo import getNewTarget
from transCoordinate import transform_coordinates_for_plot
from allocateTargetForSearcher import allocateTargetForSearcher

import threading
import time


# ipc call
import socket
import pickle

# Create a socket object
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Bind the socket to a specific address and port
server_address = ("localhost", 12345)
sock.bind(server_address)

# Listen for incoming connections
sock.listen(1)

print("Program B is waiting for a connection...")

# Accept a connection from Program A
connection, client_address = sock.accept()
print("Connection established with", client_address)

# Receive data from Program A
data_received = connection.recv(1024)
received_object = pickle.loads(data_received)
print("Program B received:", data_received)

# Close the connection
connection.close()


colorline = ["red", "green", "blue", "yellow", "black", "pink", "orange", "purple"]


def addDetectedTarget(agent, targetSearched, fakeTargets):
    targetsGet = getNewTarget(agent, targetSearched, fakeTargets)
    if len(targetsGet) > 0:
        for target in targetsGet:
            newTarget = target.copy()
            targetSearched.append(target)
            agent["targets"].append(newTarget)

        return True
    return False


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
                # print("before:", self.target["forbidenAgents"])
                self.target["forbidenAgents"].remove(self.agent["id"])
                # print("after:", self.target["forbidenAgents"])

    def start(self):
        with self.lock:
            self.target["forbidenAgents"].append(self.agent["id"])
        self.timer_thread.start()


def TargetWayPointFinished(agent, targetId):
    for targetWayPoint in agent["targetsWayPoints"]:
        if targetWayPoint["id"] == targetId:
            if len(targetWayPoint["targetPath"]) == 2:
                return True
    return False


def updateTargetRestCheck(agent, targetId, globalTargets):
    for target in globalTargets:
        if target["id"] == targetId:
            if target["restCheck"] > 0:
                target["restCheck"] -= 1
                restTime = target["restTime"] / 20
                restTimer = SetRestCheckTimer(target, agent, restTime)
                print("start a timer for", restTime)
                restTimer.start()
                target["allocated"] = False
            elif target["restCheck"] == 0:
                target["finished"] = True
                print("target", targetId, "finished")


def inCatchRange(agent, target):
    catchRange = 200
    if (
        np.linalg.norm(agent["position"] - target["position"])
        <= target["certainRadius"] + catchRange
    ):
        return True
    return False


def getControlPosition(agent, dt):
    position = agent["position"] + agent["newControl"] * dt
    return position


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

n_searcher = len(allAgents)
# get 5 biggest agent beaed on detectRadius
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


dt = 1
targetSearched = []
agents_count = len(allAgents)


allWayPointRemoved = False
while (
    not allWayPointRemoved
    and all(not target["finished"] for target in targetSearched)
    or len(targetSearched) > 0
):
    # Stage1 allocate wayPoints and TargetWayPoints to searcher and catcher
    for agent in detectAgents:
        if len(agent["wayPoints"]) == 0:
            agent["goals"] = []

        if len(agent["wayPoints"]) == 0 and len(agent["targetsWayPoints"]) == 0:
            detectAgents.remove(agent)
            catchAgents.append(agent)

    # catcher algorithem
    freeAgents = [agent for agent in catchAgents if len(agent["targetsWayPoints"]) == 0]
    freeTargets = [
        target
        for target in targetSearched
        if not target["allocated"] and not target["finished"]
    ]

    if len(freeTargets) > 0 and len(freeAgents) > 0:
        allocateTargetForSearcher(freeAgents, freeTargets)

    # Stage2 get and catch target
    for agent in allAgents:
        if addDetectedTarget(agent, targetSearched, fakeTargets):
            pass

        for target in agent["targets"]:
            # if inCatchRange(agent, target):
            if target["allocated"]:
                if TargetWayPointFinished(agent, target["id"]):
                    updateTargetRestCheck(agent, target["id"], targetSearched)
                    agent["targets"].remove(target)
                    agent["targetsWayPoints"] = [
                        targetWayPoint
                        for targetWayPoint in agent["targetsWayPoints"]
                        if targetWayPoint["id"] != target["id"]
                    ]

            else:
                updateCatchWayPoint(agent, [target], forbidArea, taskArea)
                if len(agent["goals"]) != 0:
                    # filter the goal in goals where distance between agent and goal is less than the distance between agent and taget's position
                    agent["goals"] = deleteGoalsBetweenPosAndAgent(
                        agent, target["position"]
                    )
                target["allocated"] = True

    # cooperate obstable avoidence
    for i in range(agents_count):
        obstacles_for_agent_i = []
        for j in range(agents_count):
            if i != j and inSensorRange(allAgents[i], allAgents[j]["position"]):
                if all(
                    allAgents[j]["id"] != obstacle["id"]
                    for obstacle in obstacles_for_agent_i
                ):
                    obstacles_for_agent_i.append(allAgents[j])

        for j in range(len(targetSearched)):
            if inSensorRange(allAgents[i], targetSearched[j]["position"]):
                if all(
                    targetSearched[j]["id"] != obstacle["id"]
                    for obstacle in obstacles_for_agent_i
                ):
                    obstacles_for_agent_i.append(targetSearched[j])
        if (
            len(allAgents[i]["goals"]) != 0
            or len(allAgents[i]["targetsWayPoints"]) != 0
        ):
            allAgents[i]["newControl"] = getControls(
                allAgents[i], obstacles_for_agent_i, dt, taskArea, forbidArea
            )
            # allAgents[i]["newControl"] = [0, 0]
        else:
            allAgents[i]["newControl"] = [0, 0]

    # Stage3 update allAgents position
    for agent in allAgents:
        controlPos = getControlPosition(agent, dt)
        agent["velocity"] = agent["newControl"]
        if len(agent["targetsWayPoints"]) != 0:
            targetWaypointObj = agent["targetsWayPoints"][0]
            if len(targetWaypointObj["targetPath"]) > 0:
                agent["position"] = targetWaypointObj["targetPath"][0]
                agent["targetsWayPoints"][0]["targetPath"] = agent["targetsWayPoints"][
                    0
                ]["targetPath"][1:]
        elif len(agent["goals"]) == 0:
            if len(agent["wayPoints"]) != 0:
                entryPoint = agent["wayPoints"][0]
                agent["goals"] = fillbetween(
                    agent["position"], entryPoint, forbidArea, taskArea
                )
                if not np.array_equal(agent["velocity"], np.array([0, 0])):
                    agent["goals"] = deleteGoalsBetweenPosAndAgent(agent, controlPos)
                agent["wayPoints"] = agent["wayPoints"][1:]
        elif len(agent["goals"]) > 0:
            agent["position"] = agent["goals"][0]
            agent["goals"] = agent["goals"][1:]
        agent["path"].append(agent["position"])
    plt.clf()
    plt.plot(taskArea[:, 0], taskArea[:, 1], "b")
    plt.fill(taskArea[:, 0], taskArea[:, 1], alpha=0.1, color="blue")
    for area in forbidArea:
        plt.plot(area[:, 0], area[:, 1], "r")
        plt.fill(area[:, 0], area[:, 1], alpha=0.1, color="red")

    plt.xlim([0, 7250])
    plt.ylim([0, 7250])
    # plot all wayPoints
    for i in range(len(detectAgents)):
        for wayPoint in detectAgents[i]["wayPoints"]:
            plt.plot(wayPoint[0], wayPoint[1], "o", color=colorline[i], markersize=2)
    for i in range(len(allAgents)):
        plt.plot(
            allAgents[i]["position"][0],
            allAgents[i]["position"][1],
            "*",
            color=colorline[i],
            markersize=10,
            label="Agents",
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
    for target in targetSearched:
        if not target["finished"]:
            plt.plot(target["position"][0], target["position"][1], "x", color="red")

    allWayPointRemoved = all(
        len(agent["wayPoints"]) == 0 and len(agent["targets"]) == 0
        for agent in allAgents
    )
    plt.pause(0.0001)
