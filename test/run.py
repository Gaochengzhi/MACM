import numpy as np
import random
import warnings

warnings.filterwarnings("ignore")
# from GreedyCoalitionAuctionAlgorithm import lla2ecef
import matplotlib.pyplot as plt
from planPathForSatgeOne import findStripsForAgent
from fly2waypoints import fly2waypoints, inforbid
from getControls import getControls
from inSensorRange import inSensorRange
from futurePosition import futurePosition


# from matlibPy.lla_to_ecef import lla2ecef
from findStrips import findStrips
from addObs import addTarget

# from addObs import addObs
# from inSensorRange import inSensorRange
# from updateAgent import updateAgent
# from getControls import getControls
from getInitTaskInfo import getInitTaskInfo
from getFakeTarget import getFakeTarget


def updateAgent_test(agents, traj_points):
    for i in range(len(agents)):
        if len(traj_points[i]) != 0:
            # traj_points is shape 1,2,126, how to get 1,2
            agents[i]["goal"] = traj_points[i][0, :, 0]

            # agents[i]["goal"] = traj_points[i]
    return agents


def Transcoordinate(X, Y, devX, devY):
    X = X - devX
    Y = Y - devY
    return X, Y


def transform_coordinates(points, dx, dy):
    points[:, 0], points[:, 1] = Transcoordinate(points[:, 0], points[:, 1], dx, dy)
    return points


InitInfo = getInitTaskInfo()
taskArea = InitInfo["taskArea"]
agents = InitInfo["agents"]
forbidArea = InitInfo["forbidArea"]
p_targets = getFakeTarget()


p_agent = np.array([agent["position"] for agent in agents])
p_all = np.concatenate((taskArea, p_agent))
deviation_x = np.min(p_all[:, 0])
deviation_y = np.min(p_all[:, 1])

p_agent = transform_coordinates(p_agent, deviation_x, deviation_y)
# update all agent's position
for i in range(len(agents)):
    agents[i]["position"] = p_agent[i]

p_targets = transform_coordinates(p_targets, deviation_x, deviation_y)
p_taskArea = transform_coordinates(taskArea, deviation_x, deviation_y)
x = taskArea[:, 0]
y = taskArea[:, 1]

for area in forbidArea:
    transform_coordinates(area, deviation_x, deviation_y)
agent_number = len(agents)
minSensorRange = np.min([agent["detectRadius"] for agent in agents]) * 2
lmin, lmax, _, _ = findStrips(x, y, 0, minSensorRange, minSensorRange)

pos_waypoints = np.vstack((lmin, lmax))

# # plot pos_waypoints
# plt.plot(pos_waypoints[:, 0], pos_waypoints[:, 1], "o")

# plt.show()
lineNo = lmin.shape[0]
EachLine_agent = (lineNo // agent_number) * np.ones(agent_number, dtype=int)
remainedLineNo = lineNo % agent_number
tmp = np.zeros(agent_number, dtype=int)
tmp[:remainedLineNo] = 1
EachLine_agent += tmp

Waypoints_agent = [None] * agent_number

lmin_store = lmin.copy()
lmax_store = lmax.copy()
iterRounds = 3000
Waypoints_agent = [None] * agent_number
for i in range(agent_number):
    Waypoints_agent[i] = np.hstack(
        (
            lmax_store[0 : EachLine_agent[i], :],
            lmin_store[0 : EachLine_agent[i], :],
        )
    )

# plot Waypoints_agent
for i in range(agent_number):
    plt.plot(
        Waypoints_agent[i][:, 0],
        Waypoints_agent[i][:, 1],
    )
    plt.plot(
        Waypoints_agent[i][:, 2],
        Waypoints_agent[i][:, 3],
    )

X, xmax = fly2waypoints(agent_number, p_agent, Waypoints_agent, iterRounds)

# plot out Waypoints_agent
# for i in range(agent_number):
#     plt.plot(
#         Waypoints_agent[i][:, 0],
#         Waypoints_agent[i][:, 1],
#         linewidth=2,
#     )

# plot out point in X which is 1,2,n shape
for i in range(agent_number):
    plt.plot(
        X[0, i, :],
        X[1, i, :],
    )

traj_points = [None] * agent_number
for i in range(agent_number):
    traj = X[0][:2, :]
    for j in range(len(traj[0])):
        for k in range(len(forbidArea)):
            if inforbid(forbidArea[k], traj[:, j]):
                traj[j] = np.array([0, 0])
    traj = traj[np.all(traj != [0, 0], axis=0)]
    # Take one in traj every 20
    traj_points[i] = traj[::20]


# agent's add 'goal',traj_points{i}(:,1)'
for i in range(agent_number):
    agents[i]["goal"] = traj_points[i][0, :, 0].T
    agents[i]["velocity"] = [0, 0]
    agents[i]["path"] = []
    agents[i]["newControl"] = [0, 0]
dt = 3
obs = []

for i in range(len(p_targets)):
    obs.append(addTarget(str(i + 1), p_targets[i], [0, 0]))
targets_searched = []
maxIterations = 1500
counter = 0
colorline = ["r", "g", "b", "y", "c", "m", "k", "w"]
flag = 0

while flag == 0:
    for i in range(len(agents)):
        obstacles = []
        for j in range(len(agents)):
            if i != j and inSensorRange(agents[i], agents[j]):
                obstacles.append(agents[j])
                pass

        for j_ in range(len(obs)):
            if inSensorRange(agents[i], obs[j_]):
                obstacles.append(obs[j_])
                if all(
                    abs(obs[j_]["position"][0] - target[0]) >= 1e-3
                    for target in targets_searched
                ):
                    targets_searched.append(obs[j_]["position"])
        if (agents[i]["goal"]) != []:
            agents[i]["newControl"] = getControls(agents[i], obstacles, dt)
            pass
        else:
            agents[i]["newControl"] = [0, 0]

    for i in range(agent_number):
        agents[i]["path"].append(agents[i]["position"])
        agents[i]["position"] = futurePosition(agents[i], dt)
        p_agent[i] = agents[i]["position"]
        agents[i]["velocity"] = agents[i]["newControl"]
        if len(traj_points[i]) != 0:
            if np.linalg.norm(agents[i]["position"] - agents[i]["goal"]) <= 60:
                # delete traj_points[i][:, :, 0]
                traj_points[i] = np.delete(traj_points[i], 0, axis=0)
                agents = updateAgent_test(agents, traj_points)
        else:
            agents[i]["goal"] = []
    plt.clf()
    plt.plot(taskArea[:, 0], taskArea[:, 1], "b")
    plt.fill(taskArea[:, 0], taskArea[:, 1], alpha=0.1, color="blue")
    for area in forbidArea:
        plt.plot(area[:, 0], area[:, 1], "r")
        plt.fill(area[:, 0], area[:, 1], alpha=0.1, color="red")

    plt.xlim([0, 7250])
    plt.ylim([0, 7250])
    for i in range(len(agents)):
        plt.plot(
            agents[i]["position"][0],
            agents[i]["position"][1],
            "*",
            color=colorline[i],
            markersize=10,
            label="Agents",
        )
    # plt.plot(
    #     agents[i]["path"][0],
    #     agents[i]["path"][1],
    #     color=colorline[i],
    #     label="Path",
    # )
    # plt.text(agents[i]["position"][0], agents[i]["position"][1], agents[i]["id"])
    plt.pause(0.01)
    counter = counter + 1
    flag = 1
    for i in range(len(traj_points)):
        flag = flag and (traj_points is None or len(traj_points) == 0)
    # plt.xlim[0, 7000]


# plt.figure(figsize=(10, 10))
# # make x and y the same unit
# plt.axis("equal")

# for agent in p_agent:
#     plt.plot(agent[0], agent[1], "ro")
# for area in forbidArea:
#     plt.plot(area[:, 0], area[:, 1], "r")
#     plt.fill(area[:, 0], area[:, 1], alpha=0.1, color="red")

# # strip_areas = findStripsForAgent(taskArea[:, 0], taskArea[:, 1], agents)

# # for agent_id, path in strip_areas.items():
# #     path_x, path_y = zip(*path)
# #     plt.plot(path_x, path_y, label=f"Agent {agent_id}")

# plt.legend()
# plt.show()
# # print("taskArea: ", taskArea)
# # print("agents: ", agents)
# # print("forbidArea: ", forbidArea)


# # for i, region in enumerate(vor.regions):
# #     if not -1 in region and len(region) > 0:
# #         polygon = [vor.vertices[j] for j in region]
# #         polygon = Polygon(polygon)

# #         # Check if polygon centroid is within task area and not in forbidden area
#         centroid = polygon.centroid
#         if task_area_polygon.contains(centroid) and all(
#             not forbidden.contains(centroid) for forbidden in forbidArea
#         ):
#             valid_polygons.append(polygon)
