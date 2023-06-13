import numpy as np
import json
from pyproj import Proj
import os
import warnings

warnings.filterwarnings("ignore")

stage1_folder = "./stage1_json"
for file_name in os.listdir(stage1_folder):
    file_path = os.path.join(stage1_folder, file_name)
    if os.path.isfile(file_path):
        os.remove(file_path)

# Remove files in ./stage2_json folder
stage2_folder = "./stage2_json"
for file_name in os.listdir(stage2_folder):
    file_path = os.path.join(stage2_folder, file_name)
    if os.path.isfile(file_path):
        os.remove(file_path)

# from GreedyCoalitionAuctionAlgorithm import lla2ecef
import matplotlib.pyplot as plt
from matlibPy.lla_to_ecef import lla2ecef
from matlibPy.ecef_to_lla import ecef2lla
from findStrips import findStrips
from OptimalControlSolution_stage1 import OptimalControlSolution_stage1
from OptimalControlSolution_stage2 import OptimalControlSolution_stage2
from plotMapAllocation_stage1 import plotMapAllocation_stage1
from plotMapAllocation_stage2 import plotMapAllocation_stage2
from PlotTaskLoitering import PlotTaskLoitering
from GreedyCoalitionAuctionAlgorithm.GCAASolution_revised import GCAASolution_revised

# from legendUnq import legendUnq


# Stage 1: 区域覆盖及静目标搜索
# Initial parameter settings
uniform_agents = 0
uniform_tasks = 1  # Each task's hovering radius is consistent
plot_range = 1
na = 8
nt = 32
n_rounds = 2000

Lt = 1
ii = 0
nt_loiter = int(0 * nt)  # Coefficient of nt determines the number of circling agents
task_type = np.zeros((nt, 1))  # Which agents circle, which agents approach directly
task_type[0:nt_loiter] = 1
task_type1 = np.ones((nt, 1))
lambda_val = 1

map_width = 7250
comm_distance = 0.01 * map_width

simu_time = 10
time_step = 5
time_start = 0
max_speed = 1
max_speed_task = 0.1

vesPosition = np.array(
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
vesPosition_3 = np.zeros((vesPosition.shape[0], 3))
# vesPosition_3[:, 0:2] = vesPosition

# debug
vesPosition_3[:, 0:2] = vesPosition[:, [1, 0]]

p_agent = lla2ecef(vesPosition_3, "WGS84")
p_agent = p_agent[:, :2]

targets_pos_value = np.array(
    [
        [121.66111478389999, 38.805913105872833],
        [121.71429605931525, 38.816378517234796],
        [121.66970109072057, 38.852307347659718],
        [121.68604831104761, 38.845092037001393],
        [121.6707626024872, 38.776710579528583],
        [121.69921100930935, 38.780974165685052],
        [121.69708799459914, 38.803767989386017],
        [121.66078442629077, 38.827873681556639],
        [121.69814950636578, 38.831973290969444],
        [121.67840545897317, 38.816230794368096],
        [121.68243918339334, 38.791305184403939],
        [121.71322291874901, 38.794256902090751],
        [121.67522093249633, 38.835580942891085],
        [121.68286379339384, 38.782286047784794],
        [121.69475267930022, 38.821314311948186],
        [121.70112172343084, 38.849519613531612],
        [121.69117078844744, 38.772996503899321],
        [121.66524676197139, 38.790601531239759],
        [121.6795136756341, 38.803099747183978],
        [121.69726032670366, 38.793423708593934],
        [121.70839548464096, 38.838847345176987],
        [121.66663865781643, 38.816269912713494],
        [121.7002181004114, 38.812372621474879],
        [121.67812178420058, 38.826617900888834],
        [121.71204920406547, 38.80605631718123],
        [121.66502627357841, 38.84108488727361],
        [121.71622402252871, 38.825370693544812],
        [121.66338629520476, 38.784079916968153],
        [121.70796813066727, 38.782007407946821],
        [121.68732838777908, 38.810225433215066],
        [121.70404657828328, 38.824095299183966],
        [121.65781356356618, 38.821225675502149],
        [121.68774118369561, 38.83589266976162],
        [121.68487397874952, 38.853679222580809],
        [121.67142315889046, 38.798675571360093],
    ]
)

targets_pos_value_3 = np.zeros((targets_pos_value.shape[0], 3))
targets_pos_value_3[:, 0:2] = targets_pos_value[:, [1, 0]]
p_targets = lla2ecef(targets_pos_value_3, "WGS84")
p_targets = p_targets[:, :2]

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

taskArea_3 = np.zeros((taskArea.shape[0], 3))
taskArea_3[:, 0:2] = taskArea[:, [1, 0]]
p_task = lla2ecef(taskArea_3, "WGS84")
p_task = p_task[:, :2]

p_all = np.concatenate((p_agent, p_targets, p_task))
deviation_x = np.min(p_all[:, 0])
deviation_y = np.min(p_all[:, 1])

p_agent[:, 0] = p_agent[:, 0] - deviation_x
p_agent[:, 1] = p_agent[:, 1] - deviation_y
pos_a = p_agent

p_targets[:, 0] = p_targets[:, 0] - deviation_x
p_targets[:, 1] = p_targets[:, 1] - deviation_y
pos_t = p_targets

p_task = p_task.T
p_task[0, :] = p_task[0, :] - deviation_x
p_task[1, :] = p_task[1, :] - deviation_y


targets_angle = np.array(
    [
        60,
        210,
        105,
        135,
        30,
        240,
        120,
        165,
        15,
        45,
        75,
        330,
        60,
        255,
        180,
        225,
        285,
        120,
        75,
        150,
        300,
        90,
        270,
        285,
        240,
        180,
        255,
        60,
        240,
        150,
        300,
        90,
        270,
        285,
        240,
    ]
)

targets_restCheck = np.array(
    [
        3,
        4,
        4,
        5,
        4,
        3,
        3,
        4,
        5,
        4,
        3,
        4,
        5,
        4,
        3,
        5,
        4,
        5,
        4,
        4,
        5,
        4,
        5,
        4,
        5,
        3,
        4,
        5,
        4,
        4,
        5,
        4,
        5,
        4,
        5,
    ]
)

x = p_task[0, :]
y = p_task[1, :]
lmin, lmax, V, laneDist = findStrips(x, y, 0, 400, 400)
lmin = lmin[np.argsort(lmin[:, 0])]
lmax = lmax[np.argsort(lmax[:, 0])]
pos_waypoints = np.concatenate((lmin, lmax))

tf_t = simu_time * (
    1.95 + 0.05 * np.random.rand(nt, 1)
)  # tf_t will affect task allocation results
tloiter_t = simu_time * (0.2 + 0.05 * np.random.rand(nt, 1))
tloiter_t[task_type == 0] = 0

pos_t_initial = pos_waypoints

kdrag = 3 / simu_time


def remove_completed_tasks(pos_t, ind):
    pos_t_new = np.zeros((pos_t.shape[0] - len(ind), 2))
    k = 0
    for t in range(pos_t.shape[0]):
        if t not in ind:
            pos_t_new[k] = pos_t[t - 1]
            k = k + 1
    return pos_t_new


def output_json2(Xfinal, na, deviation_x, deviation_y, time_step, i_round):
    # Assuming Xfinal, na, deviation_x, deviation_y, time_step and i_round are defined previously
    pos_a_points = np.zeros((na, 2))
    points_temp = np.zeros((2, na))

    for j in range(na):
        if Xfinal[j].shape[1] >= 3:
            pos_a_points[j, :] = Xfinal[j][0:2, 2]
            points_temp[:, j] = Xfinal[j][0:2, 2] - Xfinal[j][0:2, 1]
        elif Xfinal[j].size != 0:
            pos_a_points[j, :] = Xfinal[j][0:2, -1]
            points_temp[:, j] = Xfinal[j][0:2, -1] - Xfinal[j][0:2, -1]

    pos_a_loop_rec = np.zeros_like(pos_a_loop)
    pos_a_loop_rec[:, 0] = pos_a_loop[:, 0] + deviation_x
    pos_a_loop_rec[:, 1] = pos_a_loop[:, 1] + deviation_y
    pos_a_loop3 = np.zeros([pos_a_loop_rec.shape[0], 3])
    pos_a_loop3[:, 0:2] = pos_a_loop_rec
    pos_a_loop3[:, 2] = 3.9765e06

    # Conversion from ECEF to LLA

    pos_a_lla = ecef2lla(pos_a_loop3, "WGS84")
    pos_a_output = pos_a_lla[:, [1, 0]]

    pos_a_points[:, 0] = pos_a_points[:, 0] + deviation_x
    pos_a_points[:, 1] = pos_a_points[:, 1] + deviation_y
    pos_a_points3 = np.zeros([pos_a_points.shape[0], 3])
    pos_a_points3[:, 0:2] = pos_a_points
    pos_a_points3[:, 2] = 3.9765e06

    # Conversion from ECEF to LLA
    pos_a_points_lla = ecef2lla(pos_a_points3, "WGS84")
    pos_a_points_output = pos_a_points_lla[:, [1, 0]]

    s = {"id": 2, "method": "stage_2"}
    agent_name = ["a1", "a2", "a3", "a4", "a5", "a6", "a7", "a8"]

    id = []
    spd_temp = [0, 0]

    for j in range(na):
        course = 0
        if Xfinal[j].shape[1] >= 2:
            spd_temp = Xfinal[j][0:2, 1] - Xfinal[j][0:2, 0]
        elif Xfinal[j].size != 0:
            spd_temp = Xfinal[j][0:2, -1] - Xfinal[j][0:2, -1]

        spd = np.linalg.norm(spd_temp) / (time_step * 0.51444)
        if np.sum(spd_temp) != 0:
            course = np.arctan2(spd_temp[1], spd_temp[0]) * 180 / np.pi

        if course > 90 and course < 180:
            course = 450 - course
        else:
            course = 90 - course

        spd_points = [
            np.linalg.norm(points_temp[:, j]) / (time_step * 0.51444)
            for j in range(points_temp.shape[1])
        ]

        for i in range(len(agent_name)):
            curr_pos = {
                "coord": pos_a_output[i, :].tolist(),
                "spd": spd,
                "course": course,
            }
            s.setdefault("content", {}).setdefault("arguments", {}).setdefault(
                "vesPostion", {}
            )[agent_name[i]] = curr_pos

            points = [
                {
                    "coord": pos_a_points_output[i, :].tolist(),
                    "spd": spd_points[i],
                }
            ]
            waypoint = {"shape": "LineString", "points": points}
            id.append({"id": agent_name[i], "path": waypoint})
            s["content"]["arguments"]["road"] = id

    t = json.dumps(s, indent=4)

    # Assuming the directory 'stage2_json' exists in the current directory
    filename = os.path.join(os.getcwd(), "stage2_json", f"{i_round:05d}.json")
    with open(filename, "w") as file:
        file.write(t)


def output_json(X, deviation_x, deviation_y, time_step, i_round):
    # Assuming X, deviation_x, deviation_y, time_step and i_round are defined previously
    pos_a_points = np.transpose(X[0:2, :, 2])

    pos_a_loop_rec = np.zeros_like(pos_a_loop)
    pos_a_loop_rec[:, 0] = pos_a_loop[:, 0] + deviation_x
    pos_a_loop_rec[:, 1] = pos_a_loop[:, 1] + deviation_y
    pos_a_loop3 = np.zeros([pos_a_loop_rec.shape[0], 3])
    pos_a_loop3[:, 0:2] = pos_a_loop_rec
    pos_a_loop3[:, 2] = 3.9765e06

    # Conversion from ECEF to LLA

    pos_a_lla = ecef2lla(pos_a_loop3, "WGS84")
    pos_a_output = pos_a_lla[:, [1, 0]]

    pos_a_points[:, 0] = pos_a_points[:, 0] + deviation_x
    pos_a_points[:, 1] = pos_a_points[:, 1] + deviation_y
    pos_a_points3 = np.zeros([pos_a_points.shape[0], 3])
    pos_a_points3[:, 0:2] = pos_a_points
    pos_a_points3[:, 2] = 3.9765e06

    # Conversion from ECEF to LLA
    pos_a_points_lla = ecef2lla(pos_a_points3, "WGS84")
    pos_a_points_output = pos_a_points_lla[:, [1, 0]]

    s = {"id": 1, "method": "stage_1"}
    agent_name = ["a1", "a2", "a3", "a4", "a5", "a6", "a7", "a8"]

    id = []

    spd_temp = X[0:2, :, 1] - X[0:2, :, 0]
    spd = [
        np.linalg.norm(spd_temp[:, j]) / (time_step * 0.51444)
        for j in range(spd_temp.shape[1])
    ]
    course = [
        np.arctan2(spd_temp[1, j], spd_temp[0, j]) * 180 / np.pi
        for j in range(spd_temp.shape[1])
    ]
    course = [450 - c if c > 0 else abs(c) + 90 for c in course]

    points_temp = X[0:2, :, 2] - X[0:2, :, 1]
    spd_points = [
        np.linalg.norm(points_temp[:, j]) / (time_step * 0.51444)
        for j in range(points_temp.shape[1])
    ]

    for i in range(len(agent_name)):
        curr_pos = {
            "coord": pos_a_output[i, :].tolist(),
            "spd": spd[i],
            "course": course[i],
        }
        s.setdefault("content", {}).setdefault("arguments", {}).setdefault(
            "vesPostion", {}
        )[agent_name[i]] = curr_pos

        points = [{"coord": pos_a_points_output[i, :].tolist(), "spd": spd_points[i]}]
        waypoint = {"shape": "LineString", "points": points}
        id.append({"id": agent_name[i], "path": waypoint})
        s["content"]["arguments"]["road"] = id

    t = json.dumps(s, indent=4)

    # Assuming the directory 'stage1_json' exists in the current directory
    filename = os.path.join(os.getcwd(), "stage1_json", f"{i_round:05d}.json")
    with open(filename, "w") as file:
        file.write(t)


def update_path(p, pos_a, pos_t, time_step, Agents, nt):
    ind_completed_tasks = []

    for i in range(pos_a.shape[0]):
        if len(p[i]) != 0 and p[i] != 0:
            d_a_t = pos_t[p[i][0]] - pos_a[i]
            if np.linalg.norm(d_a_t) < time_step * Agents["Speed"][i]:
                pos_a[i] = pos_t[p[i][0]]
                nt -= 1
                Agents["Lt"][i] -= 1
                ind_completed_tasks.append(p[i][0])

                p[i] = p[i][1:]
            else:
                pos_a[i] += (
                    d_a_t / np.linalg.norm(d_a_t) * time_step * Agents["Speed"][i]
                )

    return p, pos_a, ind_completed_tasks, nt, Agents


if uniform_agents:
    v_a = np.zeros((na, 2))
else:
    v_a = (2 * np.random.rand(na, 2) - 1) * max_speed

if uniform_tasks:
    v_t = np.zeros((nt, 2))
else:
    v_t = (2 * np.random.rand(nt, 2) - 1) * max_speed_task

R = 300  # Radius for task hovering
if uniform_tasks:
    radius_t = R * np.ones((nt, 1))
else:
    radius_t = (0.2 * np.random.rand(nt, 1) + 1) * R

R2 = 50
radius_t_2 = R2 * np.ones((nt, 1))

# Reward after task completion
r_nom = 0.2
if uniform_tasks:
    r_bar = r_nom * np.ones((nt, 1))
else:
    r_bar = r_nom * np.random.rand(nt, 1)
r_bar[task_type == 1] = 5 * r_bar[task_type == 1]

# Probability that agent i successfully completes task j
if uniform_agents:
    prob_a_t = 0.7 * np.ones((na, nt))
else:
    prob_a_t = np.random.rand(na, nt)

Tasks = {}
Tasks["r_bar"] = r_bar
Tasks["prob_a_t"] = prob_a_t
Tasks["task_type"] = task_type

Agents = {}
Agents["N"] = na
Agents["Lt"] = Lt * np.ones((1, na))
Agents["v_a"] = v_a
Agents["previous_task"] = np.zeros((na, 1))
Agents["previous_winnerBids"] = np.zeros((na, 1))
Agents["rin_task"] = np.zeros((na, 2))
Agents["vin_task"] = np.zeros((na, 2))
Agents["kdrag"] = kdrag

# Fully connected graph
G = np.logical_not(np.eye(Agents["N"]))

n_rounds_loop = n_rounds
simu_time_loop = simu_time
time_start_loop = time_start
tf_t_loop = tf_t
pos_a_loop = pos_a
v_a_loop = v_a

completed_tasks_round = []
completed_tasks = []

i_round = 0


p_GCAA = [[15], [13], [11], [9], [7], [5], [3], [1]]
ind_completed = []
targets_searched = []
dynamic_flag = np.zeros((na, 1))

colors = ["red", "green", "blue", "orange", "purple", "cyan", "magenta", "yellow"]

#####################################
# Core code for Stage 1

all_targets_searched = np.empty((0, 2), float)

while not all(sublist == [0] for sublist in p_GCAA):
    # break
    # mat_p_GCAA = np.array(p_GCAA)
    # mat_p_GCAA = mat_p_GCAA[mat_p_GCAA != [-1]]

    mat_p_GCAA = np.array(
        [sublist for sublist in p_GCAA if len(sublist) != 0 and sublist != [0]]
    )
    pos_waypoint = None
    if len(mat_p_GCAA) == 0:
        pos_waypoint = []
    else:
        mat_p_GCAA = [x - 1 for x in mat_p_GCAA]
        pos_waypoint = pos_waypoints[mat_p_GCAA, :]
    pos_waypoint = np.squeeze(pos_waypoint)

    plt.clf()
    plt.xlim([0, map_width])
    plt.ylim([0, map_width])
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.title(
        "Stage 1: Area Coverage and Targets Searched: "
        + str(len(all_targets_searched)),
        fontsize=13,
    )

    targets_to_plot = []
    targets_searched = np.empty((0, 2), float)

    for i in range(na):
        if i == 0:  # 只绘制第一个代理
            plt.plot(
                pos_a_loop[i, 0],
                pos_a_loop[i, 1],
                "*",
                color=colors[i],
                markersize=10,
                label="Agent",
            )
        else:
            plt.plot(
                pos_a_loop[i, 0], pos_a_loop[i, 1], "*", color=colors[i], markersize=10
            )
        for j in range(len(p_targets)):
            if np.linalg.norm(pos_a_loop[i, :] - p_targets[j, :]) <= 300:
                targets_to_plot.append(p_targets[j, :])
                if not np.any(np.all(targets_searched == p_targets[j, :], axis=1)):
                    targets_searched = np.append(
                        targets_searched, [p_targets[j, :]], axis=0
                    )

    all_targets_searched = np.append(all_targets_searched, targets_searched, axis=0)
    all_targets_searched = np.unique(all_targets_searched, axis=0)
    if len(all_targets_searched) > 0:
        plt.plot(
            all_targets_searched[:, 0],
            all_targets_searched[:, 1],
            "rs",
            markersize=10,
            label="Targets",
            markerfacecolor=[1, 0.6, 0.6],
        )

    for w in range(len(pos_waypoint)):
        if w + 1 not in completed_tasks:
            if pos_waypoint.ndim != 2:
                break
            if w == 0:
                plt.plot(
                    pos_waypoint[w, 0],
                    pos_waypoint[w, 1],
                    "ko",
                    linewidth=6,
                    markersize=15,
                    markeredgewidth=2,
                    markerfacecolor="none",
                    label="Waypoints",
                )
            else:
                plt.plot(
                    pos_waypoint[w, 0],
                    pos_waypoint[w, 1],
                    "ko",
                    linewidth=6,
                    markersize=15,
                    markeredgewidth=2,
                    markerfacecolor="none",
                )

    plt.plot(p_task[0, :], p_task[1, :], "k", label="Boundary", linewidth=2)

    Agents["Pos"] = pos_a_loop
    Agents["v_a"] = v_a_loop

    Tasks["Pos"] = pos_waypoints
    Tasks["Speed"] = v_t
    Tasks["N"] = nt
    Tasks["tf"] = tf_t_loop
    Tasks["lambda"] = lambda_val
    Tasks["task_type"] = task_type
    Tasks["tloiter"] = tloiter_t
    Tasks["radius"] = radius_t
    Tasks["angle"] = targets_angle
    Tasks["restCheck"] = targets_restCheck

    X, completed_tasks_round = OptimalControlSolution_stage1(
        pos_a_loop,
        v_a_loop,
        pos_waypoints,
        v_t,
        radius_t,
        p_GCAA,
        Agents,
        tf_t_loop,
        tloiter_t,
        time_step,
        n_rounds_loop,
        na,
        kdrag,
    )

    plotMapAllocation_stage1(X, n_rounds_loop, na, colors, "Planned Path")

    plt.legend(loc="upper right")

    plt.draw()
    plt.pause(0.0001)

    # Update position and velocity of each agent
    pos_a_loop = X[:2, :, 1].T
    v_a_loop = X[2:4, :, 1].T

    simu_time_loop -= time_step
    time_start_loop += time_step
    n_rounds_loop -= 1
    tf_t_loop -= time_step

    if completed_tasks_round:
        for k in range(len(completed_tasks_round)):
            ind_completed = p_GCAA.index(completed_tasks_round[k])
            if dynamic_flag[ind_completed] == 0:
                # debug
                p_GCAA[ind_completed][0] += len(lmin)
                dynamic_flag[ind_completed] += 1
            elif dynamic_flag[ind_completed] == 1:
                p_GCAA[ind_completed][0] += 1
                dynamic_flag[ind_completed] += 1
            elif dynamic_flag[ind_completed] == 2:
                p_GCAA[ind_completed][0] -= len(lmin)
                dynamic_flag[ind_completed] += 1
            elif dynamic_flag[ind_completed] == 3:
                p_GCAA[ind_completed] = [0]

        Agents["N"] = na
        Agents["Lt"] = Lt * np.ones((1, na))
        Agents["previous_task"] = np.zeros((na, 1))
        Agents["previous_winnerBids"] = np.zeros((na, 1))
        Agents["rin_task"] = np.zeros((na, 2))
        Agents["vin_task"] = np.zeros((na, 2))
        Agents["kdrag"] = kdrag

        n_rounds_loop = n_rounds
    output_json(X, deviation_x, deviation_y, time_step, i_round)

    i_round += 1

plt.clf()
plt.xlim([0, map_width])
plt.ylim([0, map_width])
plt.xlabel("x [m]")
plt.ylabel("y [m]")
# for i in range(na):
#     if i == 0:
#         plt.plot(
#             pos_a_loop[i, 0],
#             pos_a_loop[i, 1],
#             "*",
#             color=colors[i],
#             markersize=10,
#             label="Agents",
#         )
#     else:
#         plt.plot(
#             pos_a_loop[i, 0],
#             pos_a_loop[i, 1],
#             "*",
#             color=colors[i],
#             markersize=10,
#         )

plt.plot(p_task[0, :], p_task[1, :], "k", label="Boundary", linewidth=2)
# plt.title("Stage 2: Static Target Exploration", fontsize=15)
# plt.legend(loc="upper right")

# Stage 2: Static Target Exploration

# Initial parameter settings
na = 8
nt = 35
nt_ini = nt
n_rounds = 5000

targets_angle = np.array(
    [
        60,
        210,
        105,
        135,
        30,
        240,
        120,
        165,
        15,
        45,
        75,
        330,
        60,
        255,
        180,
        225,
        285,
        120,
        75,
        150,
        300,
        90,
        270,
        285,
        240,
        180,
        255,
        60,
        240,
        150,
        300,
        90,
        270,
        285,
        240,
    ]
)

targets_restCheck = np.array(
    [
        3,
        4,
        4,
        5,
        4,
        3,
        3,
        4,
        5,
        4,
        3,
        4,
        5,
        4,
        3,
        5,
        4,
        5,
        4,
        4,
        5,
        4,
        5,
        4,
        5,
        3,
        4,
        5,
        4,
        4,
        5,
        4,
        5,
        4,
        5,
    ]
)

changeTargets = np.array(
    [
        [5, 10, 15, 20, 25, 30, 35],
        [150, 225, 180, 300, 120, 345, 330],
        [3, 2, 3, 5, 4, 1, 4],
        [2, 1, 2, 2, 2, 1, 3],
    ]
)


max_index = np.max(changeTargets[0])
# targets_restCheck.resize(
#     max_index
# )  # Resize targets_restCheck array to match maximum index value

targets_restCheck[changeTargets[0] - 1] = changeTargets[3]


pos_t_initial = np.copy(pos_t)
pos_t_initial = np.hstack((pos_t_initial, np.arange(1, nt + 1).reshape(-1, 1)))

R = 150  # task outer orbit radius
R2 = 50  # threat zone radius
radius_t = R * np.ones(nt)
radius_t_2 = R2 * np.ones(nt)

Lt = 1
task_type = np.zeros(nt)
task_type1 = np.ones(nt)
lambda_val = 1

map_width = 7250
comm_distance = 0.01 * map_width

simu_time = 10
time_step = 1
time_start = 0
tf_t = simu_time * (1.95 + 0.05 * np.random.rand(nt))
tloiter_t = simu_time * (0.2 + 0.05 * np.random.rand(nt))
tloiter_t[task_type == 0] = 0

uniform_agents = 0
uniform_tasks = 1
plot_range = 1

kdrag = 3 / simu_time

max_speed = 1
if uniform_agents:
    v_a = np.zeros((na, 2))
else:
    v_a = (2 * np.random.rand(na, 2) - 1) * max_speed

max_speed_task = 0.1
if uniform_tasks:
    v_t = np.zeros((nt, 2))
else:
    v_t = (2 * np.random.rand(nt, 2) - 1) * max_speed_task

r_nom = 0.2
if uniform_tasks:
    r_bar = r_nom * np.ones(nt)
else:
    r_bar = r_nom * np.random.rand(nt)
r_bar[task_type == 1] = 5 * r_bar[task_type == 1]

prob_a_t = 0.7 * np.ones((na, nt))
if uniform_agents:
    prob_a_t = 0.7 * np.ones((na, nt))
else:
    prob_a_t = np.random.rand(na, nt)

Agents = {}
Agents["N"] = na
Agents["Lt"] = Lt * np.ones(na)
Agents["v_a"] = v_a
Agents["previous_task"] = np.zeros(na)
Agents["previous_winnerBids"] = np.zeros(na)
Agents["rin_task"] = np.zeros((na, 2))
Agents["vin_task"] = np.zeros((na, 2))
Agents["kdrag"] = kdrag

costs = np.zeros((na, nt))
utility = np.zeros((na, nt))
rewards = np.zeros((na, nt))

G = np.logical_not(np.eye(Agents["N"]))


n_rounds_loop = n_rounds
simu_time_loop = simu_time
time_start_loop = time_start
tf_t_loop = tf_t
v_a_loop = v_a

U_next_tot = np.zeros(n_rounds)
U_tot = np.zeros(n_rounds)
U_completed_tot = 0

completed_tasks_round = []
completed_tasks = []
rt_completed = 0

X_full_simu = [0] * n_rounds
p_GCAA_full_simu = [0] * n_rounds
S_GCAA_ALL_full_simu = np.zeros((n_rounds, nt))
rt_full_simu = np.zeros((n_rounds, nt))
J = np.zeros((n_rounds, na))
J_to_completion_target = np.zeros((n_rounds, na))

flag = 1
flag_changeTargets = np.ones(nt)
flag_changeTargets[changeTargets[0] - 1] += 1


Agents["Pos"] = pos_a_loop
Agents["v_a"] = v_a_loop

pos_t = np.hstack((pos_t, np.arange(1, nt + 1).reshape(-1, 1)))
Tasks = {
    "r_bar": r_bar,
    "prob_a_t": prob_a_t,
    "task_type": task_type,
    "Pos": pos_t,
    "Speed": v_t,
    "N": nt,
    "tf": tf_t_loop,
    "lambda": lambda_val,
    "task_type": task_type,
    "task_type1": task_type1,
    "tloiter": tloiter_t,
    "radius": radius_t,
    "radius_t_2": radius_t_2,
    "angle": targets_angle,
    "restCheck": targets_restCheck,
    "completed": np.zeros(nt),
    "flag_changeTargets": flag_changeTargets,
}

completed_agents = []
assigned_tasks = []
Xfinal = [None] * na
Tasks_initial = Tasks.copy()
completed_tasks_Store = []


plt.figure()
for i_round in range(0, n_rounds):
    completed_tasks_round = []
    # Initial Plot Code here
    plt.clf()
    plt.xlim([0, map_width])
    plt.ylim([0, map_width])
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.title("Stage 2: Static Target Exploration", fontsize=15)
    for i in range(na):
        if i == 0:
            plt.plot(
                pos_a_loop[i, 0],
                pos_a_loop[i, 1],
                "*",
                color=colors[i],
                markersize=10,
                label="Agents",
            )
        else:
            plt.plot(
                pos_a_loop[i, 0], pos_a_loop[i, 1], "*", color=colors[i], markersize=10
            )

    if i_round == 0:
        plt.plot(
            Tasks_initial["Pos"][:, 0],
            Tasks_initial["Pos"][:, 1],
            "rs",
            markersize=10,
            label="Targets",
            markerfacecolor=[1, 0.6, 0.6],
        )
        PlotTaskLoitering(
            Tasks_initial["Pos"][:, 0:2],
            Tasks_initial["radius"],
            Tasks_initial["task_type1"],
            "b",
            "Exploration Range",
        )
        PlotTaskLoitering(
            Tasks_initial["Pos"][:, 0:2],
            Tasks_initial["radius_t_2"],
            Tasks_initial["task_type1"],
            "r",
            "Threaten Range",
        )
        plt.plot(p_task[0, :], p_task[1, :], "k", label="Boundary", linewidth=2)
    else:
        plt.plot(
            Tasks_initial["Pos"][:, 0],
            Tasks_initial["Pos"][:, 1],
            "rs",
            markersize=10,
            markerfacecolor=[1, 0.6, 0.6],
        )
        PlotTaskLoitering(
            Tasks_initial["Pos"][:, 0:2],
            Tasks_initial["radius"],
            Tasks_initial["task_type1"],
            "b",
        )
        PlotTaskLoitering(
            Tasks_initial["Pos"][:, 0:2],
            Tasks_initial["radius_t_2"],
            Tasks_initial["task_type1"],
            "r",
        )
        plt.plot(p_task[0, :], p_task[1, :], "k", linewidth=2)

    for i in range(np.size(Tasks_initial["Pos"], 0)):
        plt.text(
            Tasks_initial["Pos"][i, 0] + 100,
            Tasks_initial["Pos"][i, 1] + 100,
            str(int(Tasks_initial["Pos"][i, 2])),
            fontsize=15,
        )

    if flag != 0 and nt != 0:
        p_GCAA_tmp, taskInd, Agents = GCAASolution_revised(Agents, G, Tasks)
        assigned_tasks = list(map(int, p_GCAA_tmp))
        (
            Xmed,
            _,
            _,
            _,
        ) = OptimalControlSolution_stage2(
            Agents, Tasks, p_GCAA_tmp, time_step, n_rounds_loop, kdrag
        )

        if i_round == 0:
            Xfinal = Xmed
            p_GCAA = list(map(int, taskInd))
        else:
            mm = 1
            Xmed = [i for i in Xmed if i is not None]
            for j in range(len(Xmed)):
                Xfinal[completed_agents[j]] = Xmed[j]
                if not p_GCAA_tmp[j]:
                    p_GCAA[completed_agents[j]] = []
                else:
                    p_GCAA[completed_agents[j]] = taskInd[mm - 1]
                    mm += 1

        flag = 0
        completed_agents = []

        for task in taskInd:
            flag_changeTargets[int(task) - 1] -= 1

        # Indices of tasks to keep
        keep_tasks = [i for i in range(nt) if i not in assigned_tasks]

        assigned_tasks1 = assigned_tasks.copy()
        assigned_tasks1 = [i - 1 for i in assigned_tasks1]
        pos_t = np.delete(pos_t, assigned_tasks1, axis=0)
        v_t = np.delete(v_t, assigned_tasks1, axis=0)
        nt -= len(assigned_tasks)

        tf_t = np.delete(tf_t, assigned_tasks1, axis=0)
        tf_t_loop = tf_t
        task_type = np.delete(task_type, assigned_tasks1, axis=0)
        task_type1 = np.delete(task_type1, assigned_tasks1, axis=0)
        tloiter_t = np.delete(tloiter_t, assigned_tasks1, axis=0)
        radius_t = np.delete(radius_t, assigned_tasks1, axis=0)
        radius_t_2 = np.delete(radius_t_2, assigned_tasks1, axis=0)
        Tasks["r_bar"] = np.delete(Tasks["r_bar"], assigned_tasks1, axis=0)
        Tasks["prob_a_t"] = np.delete(Tasks["prob_a_t"], assigned_tasks1, axis=1)
        Tasks["Pos"] = pos_t
        Tasks["Speed"] = v_t
        Tasks["N"] = nt
        Tasks["tf"] = tf_t_loop
        Tasks["lambda"] = lambda_val
        Tasks["task_type"] = task_type
        Tasks["tloiter"] = tloiter_t
        Tasks["radius"] = radius_t
        Tasks["angle"] = np.delete(Tasks["angle"], assigned_tasks1)
        Tasks["restCheck"] = np.delete(Tasks["restCheck"], assigned_tasks1)
        Tasks["flag_changeTargets"] = np.delete(
            Tasks["flag_changeTargets"], assigned_tasks1
        )

        assigned_tasks = []

    plotMapAllocation_stage2(Xfinal, na, colors, "Planned Path")
    # plt.legend(legendUnq(plt.gca()))
    plt.draw()
    plt.pause(0.00001)
    output_json2(Xfinal, na, deviation_x, deviation_y, time_step, i_round)

    # Update position and velocity of each agent
    for i in range(na):
        if Xfinal[i] is None or len(Xfinal[i][0]) == 0:
            tmp = int(p_GCAA[i])
            if tmp not in completed_tasks_Store and tmp:
                completed_agents.append(i)
                completed_tasks_round.append(tmp - 1)
                flag = 1
                if int(flag_changeTargets[tmp - 1]) == 0:
                    completed_tasks_Store.append(tmp)
        else:
            if len(Xfinal[i]) == 0:
                break
            pos_a_loop[i] = Xfinal[i][0:2, 0]
            v_a_loop[i] = Xfinal[i][2:4, 0]
            Xfinal[i] = np.delete(Xfinal[i], 0, 1)

    n_rounds_loop -= 1
    simu_time_loop -= time_step
    time_start_loop += time_step
    tf_t_loop -= time_step

    # Code for ~isempty(completed_tasks_round)
    # ...
    # ...
    if len(completed_tasks_round) != 0:
        # for Plot Targets
        tmp1 = Tasks_initial["Pos"][:, 2]
        ind1 = [i for i, j in enumerate(tmp1) if j in completed_tasks_round]
        for j in range(len(ind1)):
            if ind1[j] != -1 and flag_changeTargets[completed_tasks_round[j]] == 0:
                # pos_t_tmp = np.delete(Tasks_initial["Pos"], ind1[j] + 1, axis=0)
                # radius_t_tmp = np.delete(Tasks_initial["radius"], ind1[j] + 1, axis=0)
                # radius_t_2_tmp = np.delete(
                #     Tasks_initial["radius_t_2"], ind1[j] + 1, axis=0
                # )
                # task_type1_tmp = np.delete(
                #     Tasks_initial["task_type1"], ind1[j] + 1, axis=0
                # )
                # Tasks_initial["Pos"] = pos_t_tmp
                # Tasks_initial["radius"] = radius_t_tmp
                # Tasks_initial["radius_t_2"] = radius_t_2_tmp
                # Tasks_initial["task_type1"] = task_type1_tmp
                pass

            elif flag_changeTargets[completed_tasks_round[j]] != 0:
                ind2 = completed_tasks_round[j]
                ind3 = np.where(changeTargets[0] == ind2 + 1)
                pos_t = np.vstack((pos_t, pos_t_initial[ind2]))
                v_t = np.vstack((v_t, [0, 0]))
                nt += 1
                tf_t = np.append(tf_t, simu_time * (1.95 + 0.05 * np.random.rand()))
                tf_t_loop = np.copy(tf_t)
                task_type = np.append(task_type, 0)
                task_type1 = np.append(task_type1, 1)
                tloiter_t = np.append(tloiter_t, 0)
                radius_t = np.append(radius_t, R)
                radius_t_2 = np.append(radius_t_2, R2)

                Tasks["r_bar"] = np.append(Tasks["r_bar"], r_nom)
                Tasks["prob_a_t"] = np.hstack(
                    (Tasks["prob_a_t"], np.random.rand(Tasks["prob_a_t"].shape[0], 1))
                )

                Tasks["Pos"] = pos_t
                Tasks["Speed"] = v_t
                Tasks["N"] = nt
                Tasks["tf"] = tf_t_loop
                Tasks["lambda"] = lambda_val
                Tasks["task_type"] = task_type
                Tasks["tloiter"] = tloiter_t
                Tasks["radius"] = radius_t
                Tasks["radius_t_2"] = radius_t_2
                Tasks["angle"] = np.append(Tasks["angle"], changeTargets[1, ind3])
                Tasks["restCheck"] = np.append(
                    Tasks["restCheck"], changeTargets[2, ind3]
                )
                Tasks["flag_changeTargets"] = np.append(
                    Tasks["flag_changeTargets"], flag_changeTargets[ind2]
                )
        # for Update Agents
        na_new = len(completed_agents)
        v_a_loop = np.zeros((8, 2))
        Agents["N"] = na_new
        Agents["Pos"] = np.array(pos_a_loop)[completed_agents, :]
        Agents["Lt"] = [Lt] * na_new
        Agents["previous_task"] = np.zeros(na_new)
        Agents["previous_winnerBids"] = np.zeros(na_new)
        Agents["rin_task"] = np.zeros((na_new, 2))
        Agents["vin_task"] = np.zeros((na_new, 2))
        Agents["kdrag"] = kdrag
        Agents["v_a"] = v_a_loop

        Tasks["prob_a_t"] = np.random.rand(na_new, nt)

        n_rounds_loop = n_rounds

    if len(completed_tasks_Store) == nt_ini:
        break

plt.clf()
plt.xlim([0, map_width])
plt.ylim([0, map_width])
plt.xlabel("x [m]")
plt.ylabel("y [m]")
plt.title("Stage 2: Static Target Exploration", fontsize=15)
for i in range(na):
    plt.plot(
        pos_a_loop[i, 0],
        pos_a_loop[i, 1],
        "*",
        color=colors[i],
        markersize=10,
        label="Agents",
    )

plt.plot(
    pos_t[:, 0],
    pos_t[:, 1],
    "rs",
    markersize=8,
    label="Targets",
    markerfacecolor=[1, 0.6, 0.6],
)
plt.plot(p_task[0, :], p_task[1, :], "k", label="Boundary", linewidth=2)
plt.legend(loc="upper right")
plt.draw()
plt.pause(0.001)
# plt.show()
