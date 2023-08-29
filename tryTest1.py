import numpy as np
import warnings
import math

warnings.filterwarnings("ignore")
import matplotlib.pyplot as plt
from getControls import getControls
from inSensorRange import inSensorRange
from divdeStrip import divide_into_stripes, smoothAndAvoidObstacles
from fillBetween import fillbetween, distance

from addTargets import addTargets
from getInitTaskInfo import getInitTaskInfo
from getFakeTarget import getFakeTarget
from transCoordinate import transform_coordinates_for_plot, ecef2lla, lla2ecef
from updateCatchWayPoint import updateCatchWayPoint

from recieveAndSendInfo import getNewTarget
from transCoordinate import transform_coordinates_for_plot
from allocateTargetForSearcher import allocateTargetForSearcher
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


def interpolate_line_segment(point1, point2, max_distance=10.0):
    dis = distance(point1, point2)
    num_points = math.ceil(dis / max_distance)

    if num_points < 2:
        return np.array([point1, point2])

    x_values = np.linspace(point1[0], point2[0], num_points)
    y_values = np.linspace(point1[1], point2[1], num_points)

    return np.column_stack((x_values, y_values))


def turnForbidenAreaIntoPoints(forbidAreaRaw):
    forbidAreaPointsList = []

    for area in forbidAreaRaw:
        area = np.array(area)
        edge_points = []

        for i in range(len(area)):
            point1 = area[i]
            point2 = area[(i + 1) % len(area)]

            interpolated_points = interpolate_line_segment(point1, point2)
            edge_points.append(interpolated_points)

        edge_points = np.vstack(edge_points)
        forbidAreaPointsList.append(edge_points)

    return forbidAreaPointsList


def addDetectedTarget(agent, targetSearched, fakeTargets):
    targetsGet = getNewTarget(agent, targetSearched, fakeTargets)
    if len(targetsGet) > 0:
        for target in targetsGet:
            newTarget = target
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
                self.target["forbidenAgents"].remove(self.agent["id"])

    def start(self):
        with self.lock:
            if self.agent["id"] not in self.target["forbidenAgents"]:
                self.target["forbidenAgents"].append(self.agent["id"])
        self.timer_thread.start()


def TargetWayPointFinished(agent, targetId):
    for targetWayPoint in agent["targetsWayPoints"]:
        if targetWayPoint["id"] == targetId:
            if len(targetWayPoint["targetPath"]) < 2:
                return True
    return False


def updateTargetRestCheck(agent, targetId, globalTargets):
    for target in globalTargets:
        if target["id"] == targetId:
            if target["restCheck"] > 1:
                target["restCheck"] -= 1
                restTime = target["restTime"]
                restTimer = SetRestCheckTimer(target, agent, restTime)
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

forbidArea.append(
    np.array(
        [
            [600, 4000],
            [600, 8000],
            [630, 8000],
            [630, 4000],
            [600, 4000],
        ]
    )
)
for i in range(len(forbidArea)):
    if i == len(forbidArea) - 1:
        forbidArea[i][0], forbidArea[i][1] = (
            forbidArea[i][0] + 5,
            forbidArea[i][1] - 5,
        )

forbidAreaPoints = turnForbidenAreaIntoPoints(forbidArea)


centralPoints = calculate_central_points(forbidArea)

## paramters
n_searcher = len(allAgents) - 5
arrive_distance = 10
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

tmpforbiden = []
for area in forbidAreaPoints:
    for point in area:
        tmpforbiden.append(
            {
                "id": "das" + str(point[0]),
                "position": point,
                "velocity": [0, 0],
                "certainRadiusLimit": 80,
            }
        )

wayPoints = divide_into_stripes(p_taskArea, detectAgents, detectRadius, detectVelocity)

for i in range(len(detectAgents)):
    detectAgents[i]["wayPoints"] = smoothAndAvoidObstacles(
        wayPoints[i], detectAgents[i]["position"], forbidArea, p_taskArea
    )

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
dt = 4
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
    freeAgents = [agent for agent in catchAgents if len(agent["targetsWayPoints"]) == 0]
    # freeAgents = [agent for agent in catchAgents]
    freeTargets = [
        target
        for target in targetSearched
        if target["allocated"] > 0 and not target["finished"]
    ]

    if len(freeTargets) > 0 and len(freeAgents) > 0:
        allocateTargetForSearcher(freeAgents, freeTargets)

    # Stage2 get and catch target
    for agent in allAgents:
        if addDetectedTarget(agent, targetSearched, fakeTargets):
            pass

        agent["type"] = 0
        for target in agent["targets"]:
            if inCatchRange(agent, target):
                agent["type"] = 1
            # else:
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
                updateCatchWayPoint(
                    agent,
                    [target],
                    forbidArea,
                    taskArea,
                    arrive_distance + 100,
                    centralPoints,
                )
                if len(agent["goals"]) != 0:
                    agent["goals"] = deleteGoalsBetweenPosAndAgent(
                        agent, target["position"]
                    )
                target["allocated"] = True
                # break

    # cooperate obstacle avoidence
    for i in range(agents_count):
        # obstacles_for_agent_i = deep copy tmpforbiden
        # obstacles_for_agent_i = tmpforbiden
        obstacles_for_agent_i = []

        for k in range(agents_count):
            if i != k and inSensorRange(allAgents[i], allAgents[k]["position"]):
                if all(
                    allAgents[k]["id"] != obstacle["id"]
                    for obstacle in obstacles_for_agent_i
                ):
                    obstacles_for_agent_i.append(allAgents[k])

        for k in range(len(targetSearched)):
            if inSensorRange(allAgents[i], targetSearched[k]["position"]):
                if all(
                    targetSearched[k]["id"] != obstacle["id"]
                    for obstacle in obstacles_for_agent_i
                ):
                    obstacles_for_agent_i.append(targetSearched[k])
        if len(allAgents[i]["goals"]) > 0 or len(allAgents[i]["targetsWayPoints"]) > 0:
            # allAgents[i]["newControl"] = [0, 0]
            allAgents[i]["newControl"] = getControls(
                allAgents[i], obstacles_for_agent_i, dt, forbidArea
            )
        else:
            allAgents[i]["newControl"] = [0, 0]
        # delete obstacles_for_agent_i
        # del obstacles_for_agent_i

    # Stage3 update allAgents position
    for agent in allAgents:
        controlPos = getControlPosition(agent, dt)
        agent["velocity"] = agent["newControl"]
        if len(agent["targetsWayPoints"]) != 0:
            targetWaypointObj = agent["targetsWayPoints"][0]
            if len(targetWaypointObj["targetPath"]) > 0:
                # agent["position"] = targetWaypointObj["targetPath"][0]
                agent["position"] = controlPos
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
                    arrive_distance,
                )
                if not np.array_equal(agent["velocity"], np.array([0, 0])):
                    # agent["goals"] = deleteGoalsBetweenPosAndAgent(agent, controlPos)
                    pass
                agent["wayPoints"] = agent["wayPoints"][1:]
        elif len(agent["goals"]) > 0:
            # agent["position"] = agent["goals"][0]
            agent["position"] = controlPos
            if distance(agent["position"], agent["goals"][0]) < arrive_distance:
                agent["goals"] = agent["goals"][1:]
        agent["path"].append(agent["position"])
    plt.clf()
    plt.plot(taskArea[:, 0], taskArea[:, 1], "b")
    plt.fill(taskArea[:, 0], taskArea[:, 1], alpha=0.1, color="blue")
    for area in forbidArea:
        plt.plot(area[:, 0], area[:, 1], "r")
        plt.fill(area[:, 0], area[:, 1], alpha=0.1, color="red")

    # plt.xlim([0, 7250])
    # plt.ylim([0, 7250])
    # plot all wayPoints
    # for i in range(len(detectAgents)):
    #     for wayPoint in detectAgents[i]["wayPoints"]:
    #         plt.plot(wayPoint[0], wayPoint[1], "o", color=colorline[i], markersize=2)
    # plot forbidAreaPoints
    # for area in forbidAreaPoints:
    #     plt.plot(area[:, 0], area[:, 1], "*")
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
                markersize=5,
                alpha=0.99,
                label="goals",
            )
        if len(allAgents[i]["targetsWayPoints"]) > 0:
            plt.plot(
                np.array(allAgents[i]["targetsWayPoints"][0]["targetPath"])[:, 0],
                np.array(allAgents[i]["targetsWayPoints"][0]["targetPath"])[:, 1],
                ".",
                color=colorline[i],
                markersize=5,
                alpha=0.99,
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

        plt.text(
            allAgents[i]["position"][0] + 50,
            allAgents[i]["position"][1] + 150,
            "goals:" + str(len(allAgents[i]["goals"])),
            color=colorline[i],
        )
        plt.text(
            allAgents[i]["position"][0] + 50,
            allAgents[i]["position"][1] - 150,
            "t:" + str(len(allAgents[i]["targetsWayPoints"])),
            color=colorline[i],
        )
        if len(allAgents[i]["targetsWayPoints"]) > 0:
            plt.text(
                allAgents[i]["position"][0] + 50,
                allAgents[i]["position"][1] - 300,
                "twp:" + str(len(allAgents[i]["targetsWayPoints"][0]["targetPath"])),
                color=colorline[i],
            )
        # plt.plot(
        #     np.array(allAgents[i]["path"])[:, 0],
        #     np.array(allAgents[i]["path"])[:, 1],
        #     color=(0, 1, 0, 0.3),
        #     linewidth=allAgents[i]["detectRadius"] / 25,
        # )
    for target in targetSearched:
        if not target["finished"]:
            plt.plot(target["position"][0], target["position"][1], "x", color="red")
            # plt text target id and restime id be red restCheck be blue
            # plt.text(
            #     target["position"][0] + 50,
            #     target["position"][1] + 20,
            #     str(target["id"]),
            #     color="red",
            # )
            plt.text(
                target["position"][0],
                target["position"][1] - 50,
                str(target["restCheck"]),
                color="blue",
            )

    allWayPointRemoved = all(len(agent["wayPoints"]) == 0 for agent in allAgents)

    # plt all wayPoints's number
    wayPointsNumber = sum([len(agent["wayPoints"]) for agent in allAgents])
    plt.text(9, 6000, "wayPointsNumber: " + str(wayPointsNumber))
    allTargetFinished = all([target["finished"] for target in targetSearched])
    Finished = allTargetFinished and allWayPointRemoved
    # plt len of targetSearched and finished target put in top center

    plt.text(9, 7000, "targetSearched: " + str(len(targetSearched)))
    plt.text(
        9,
        6500,
        "finished: "
        + str(len([target for target in targetSearched if target["finished"]])),
    )
    if Finished:
        plt.text(0, 0, "Finished", color="red")
        # plt.close("all")
        # exit()

    plt.pause(0.0001)
