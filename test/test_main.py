import numpy as np
import warnings

warnings.filterwarnings("ignore")

import matplotlib.pyplot as plt
from matlibPy.lla_to_ecef import lla2ecef
from findStrips import findStrips
from addObs import addTarget
from addAgent import addAgent
from inSensorRange import inSensorRange
from updateAgent import updateAgent
from getControls import getControls
from OptimalControlSolution_stage2 import OptimalControlSolution_stage2
from plotMapAllocation_stage2 import plotMapAllocation_stage2
from PlotTaskLoitering import PlotTaskLoitering
from GreedyCoalitionAuctionAlgorithm.GCAASolution_revised import GCAASolution_revised


def lines(n):
    cmap = plt.get_cmap("tab10")
    return [cmap(i) for i in np.linspace(0, 1, n)]


def futurePosition(agent, dt):
    position = np.array(agent["position"]) + np.array(agent["newControl"]) * dt
    return position


agent_number = 8
target_number = 34

map_width = 7000

agentInitPostion = np.array(
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
p_agent = lla2ecef(agentInitPostion)

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
p_targets = lla2ecef(targets_pos_value)

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
p_task = lla2ecef(taskArea, "WGS84")
# p_task = taskArea_3

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
    ]
)

targets_restCheck = np.array(
    [3, 4, 4, 5, 4, 3, 3, 4, 5, 4, 3, 4, 5, 4, 3, 5, 4, 5, 4, 4, 5, 4, 5, 4, 5]
)

x = p_task[0, :]
y = p_task[1, :]


p_GCAA = [[15], [13], [11], [9], [7], [5], [3], [1]]
ind_completed = []
targets_searched = []


mat_p_GCAA = np.array(p_GCAA).flatten()
mat_p_GCAA = mat_p_GCAA[mat_p_GCAA != 0]
senseorangeList = np.random.randint(200, 300, 8)
dt = 3
maxIterations = 1500
counter = 0
lmin, lmax, V, laneDist = findStrips(x, y, 0, 400, 400)

pos_waypoints = np.concatenate((lmin, lmax))
pos_waypoint = pos_waypoints[mat_p_GCAA]
agents = addAgent(pos_a, pos_waypoint, senseorangeList)
colors = lines(len(agents))
dynamic_flag = np.zeros(len(agents))
obs = [addTarget(str(i), pos_t[i], [0, 0], 3) for i in range(0, len(pos_t))]

selected_targets = np.array(
    [
        targets_pos_value[10, :],
        targets_pos_value[22, :],
        targets_pos_value[8, :],
        targets_pos_value[26, :],
        targets_pos_value[13, :],
        targets_pos_value[24, :],
        targets_pos_value[10, :],
    ]
)

# Calculate forbidden area points
forbidArea = (selected_targets[1:, :] + selected_targets[:-1, :]) / 2
forbidArea = np.vstack((forbidArea, forbidArea[0, :]))

# Swap columns for longitude and latitude
forbidArea_3 = np.zeros((forbidArea.shape[0], 3))
forbidArea_3[:, :2] = forbidArea[:, [1, 0]]
p_forbidArea = lla2ecef(forbidArea_3, "WGS84")
p_forbidArea = p_forbidArea[:, :2]


def remove_completed_tasks(pos_t, ind):
    pos_t_new = np.zeros((pos_t.shape[0] - len(ind), 2))
    k = 0
    for t in range(pos_t.shape[0]):
        if t not in ind:
            pos_t_new[k] = pos_t[t - 1]
            k = k + 1
    return pos_t_new


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


pos_a_loop = pos_a

plt.figure(figsize=(10, 10))
while len(mat_p_GCAA) > 0:
    completed_tasks = []
    maxDistFromGoal = 0

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
        agents[i]["newControl"] = getControls(agents[i], obstacles, dt)

    for i in range(len(agents)):
        if len(agents[i]["path"]) == 0:
            agents[i]["path"] = np.array([agents[i]["position"]])
        else:
            agents[i]["path"] = np.vstack([agents[i]["path"], agents[i]["position"]])
        agents[i]["position"] = futurePosition(agents[i], dt)
        maxDistFromGoal = max(
            maxDistFromGoal, np.sum((agents[i]["position"] - agents[i]["goal"]) ** 2)
        )

        if (
            np.linalg.norm(agents[i]["position"] - agents[i]["goal"]) <= 20
            and p_GCAA[i] not in completed_tasks
        ):
            completed_tasks.append(p_GCAA[i])

    if completed_tasks and len(pos_waypoint) == len(agents):
        for k in range(len(completed_tasks)):
            ind_completed = np.where(np.array(p_GCAA) == completed_tasks[k])[0]
            if ind_completed.size > 0:
                if dynamic_flag[ind_completed[0]] == 0:
                    p_GCAA[ind_completed[0]][0] += len(lmin)
                    dynamic_flag[ind_completed[0]] += 1
                elif dynamic_flag[ind_completed[0]] == 1:
                    p_GCAA[ind_completed[0]][0] += 1
                    dynamic_flag[ind_completed[0]] += 1
                elif dynamic_flag[ind_completed[0]] == 2:
                    p_GCAA[ind_completed[0]][0] -= len(lmin)
                    dynamic_flag[ind_completed[0]] += 1

    if completed_tasks:
        for k in range(len(completed_tasks)):
            ind_completed = np.where(p_GCAA == completed_tasks[k])
            if dynamic_flag[ind_completed[0]] == 3:
                p_GCAA[ind_completed[0]] = 0

    mat_p_GCAA = np.array(p_GCAA).flatten()
    indices = [int(index) - 1 for index in mat_p_GCAA]
    pos_waypoint = pos_waypoints[indices, :]

    if len(pos_waypoint) == len(agents):
        agents = updateAgent(agents, pos_waypoint)
    plt.clf()
    plt.plot(p_forbidArea[:, 0], p_forbidArea[:, 1], "k", label="Boundary", linewidth=2)
    plt.plot(p_task[0, :], p_task[1, :], "k", label="Boundary", linewidth=2)
    plt.plot(
        pos_waypoint[:, 0],
        pos_waypoint[:, 1],
        "ko",
        markersize=15,
        label="Waypoints",
        markerfacecolor="none",
        markeredgewidth=2,
    )

    for i, agent in enumerate(agents):
        faceColor = np.array([0, 170, 255]) / 255
        lineColor = np.array([135, 135, 135]) / 255
        plt.plot(
            agent["position"][0],
            agent["position"][1],
            "*",
            color=colors[i],
            markersize=10,
            label="Agents",
        )
        # PlotTaskLoitering(
        #     agent["position"], agent["radius"], 1, "g", "Agent Threaten Range"
        # )
        plt.quiver(
            agent["position"][0],
            agent["position"][1],
            agent["velocity"][0],
            agent["velocity"][1],
            color="black",
        )
        plt.plot(
            agent["path"][:, 0],
            agent["path"][:, 1],
            color=(0, 1, 0, 0.3),
            linewidth=41.379,
        )
        plt.plot(
            agent["path"][:, 0],
            agent["path"][:, 1],
            color=(1, 0, 0, 1),
            linewidth="1",
        )

        plt.text(agent["position"][0], agent["position"][1], agent["name"])

    if len(targets_searched) > 0:
        for pos in targets_searched:
            # PlotTaskLoitering(pos, 50, 1, "r", "Target Threaten Range")
            plt.plot(
                pos[0],
                pos[1],
                "rs",
                markersize=5,
                markerfacecolor=[1, 0.6, 0.6],
                label="Targets",
            )

    plt.title(
        "Stage 1: Area Coverage and Targets Searched: " + str(len(targets_searched)),
        fontsize=20,
    )
    # plt.legend(legendUnq(plt.gca()))
    plt.xlabel("X [m]")
    plt.ylabel("Y [m]")
    plt.xlim([0, 7250])
    plt.ylim([0, 7250])
    # plt.hold(False)
    plt.draw()
    plt.pause(0.01)

    counter += 1

    if maxDistFromGoal < 0.1:
        break


plt.clf()
plt.xlim([0, map_width])
plt.ylim([0, map_width])
plt.xlabel("x [m]")
plt.ylabel("y [m]")
for i in range(agent_number):
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
            pos_a_loop[i, 0],
            pos_a_loop[i, 1],
            "*",
            color=colors[i],
            markersize=10,
        )

# plt.plot(p_task[0, :], p_task[1, :], "k", label="Boundary", linewidth=2)
# plt.title("Stage 2: Static Target Exploration", fontsize=15)
plt.legend(loc="upper right")

# Stage 2: Static Target Exploration

# Initial parameter settings
agent_number = 8
target_number = 25
nt_ini = target_number
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
    ]
)
targets_restCheck = np.array(
    [3, 4, 4, 5, 4, 3, 3, 4, 5, 4, 3, 4, 5, 4, 3, 5, 4, 5, 4, 4, 5, 4, 5, 4, 5]
)
changeTargetsNo = 5
changeTargets = np.array(
    [[5, 10, 15, 20, 25], [45, 60, 180, 135, 120], [4, 4, 3, 4, 5], [2, 1, 2, 1, 1]]
)

max_index = np.max(changeTargets[0])
targets_restCheck.resize(
    max_index
)  # Resize targets_restCheck array to match maximum index value

targets_restCheck[changeTargets[0] - 1] = changeTargets[3]


pos_t_initial = np.copy(pos_t)
pos_t_initial = np.hstack(
    (pos_t_initial, np.arange(1, target_number + 1).reshape(-1, 1))
)

R = 300  # task outer orbit radius
R2 = 50  # threat zone radius
radius_t = R * np.ones(target_number)
radius_t_2 = R2 * np.ones(target_number)

Lt = 1
task_type = np.zeros(target_number)
task_type1 = np.ones(target_number)
lambda_val = 1

map_width = 7000
comm_distance = 0.01 * map_width

simu_time = 10
time_step = 0.05
time_start = 0
tf_t = simu_time * (1.95 + 0.05 * np.random.rand(target_number))
tloiter_t = simu_time * (0.2 + 0.05 * np.random.rand(target_number))
tloiter_t[task_type == 0] = 0

uniform_agents = 0
uniform_tasks = 1
plot_range = 1

kdrag = 3 / simu_time

max_speed = 1
if uniform_agents:
    v_a = np.zeros((agent_number, 2))
else:
    v_a = (2 * np.random.rand(agent_number, 2) - 1) * max_speed

max_speed_task = 0.1
if uniform_tasks:
    v_t = np.zeros((target_number, 2))
else:
    v_t = (2 * np.random.rand(target_number, 2) - 1) * max_speed_task

r_nom = 0.2
if uniform_tasks:
    r_bar = r_nom * np.ones(target_number)
else:
    r_bar = r_nom * np.random.rand(target_number)
r_bar[task_type == 1] = 5 * r_bar[task_type == 1]

prob_a_t = 0.7 * np.ones((agent_number, target_number))
if uniform_agents:
    prob_a_t = 0.7 * np.ones((agent_number, target_number))
else:
    prob_a_t = np.random.rand(agent_number, target_number)

Agents = {}
Agents["N"] = agent_number
Agents["Lt"] = Lt * np.ones(agent_number)
Agents["v_a"] = v_a
Agents["previous_task"] = np.zeros(agent_number)
Agents["previous_winnerBids"] = np.zeros(agent_number)
Agents["rin_task"] = np.zeros((agent_number, 2))
Agents["vin_task"] = np.zeros((agent_number, 2))
Agents["kdrag"] = kdrag

costs = np.zeros((agent_number, target_number))
utility = np.zeros((agent_number, target_number))
rewards = np.zeros((agent_number, target_number))

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
S_GCAA_ALL_full_simu = np.zeros((n_rounds, target_number))
rt_full_simu = np.zeros((n_rounds, target_number))
J = np.zeros((n_rounds, agent_number))
J_to_completion_target = np.zeros((n_rounds, agent_number))

flag = 1
flag_changeTargets = np.ones(target_number)
flag_changeTargets[changeTargets[0] - 1] += 1


Agents["Pos"] = pos_a_loop
Agents["v_a"] = v_a_loop

pos_t = np.hstack((pos_t, np.arange(1, target_number + 1).reshape(-1, 1)))
Tasks = {
    "r_bar": r_bar,
    "prob_a_t": prob_a_t,
    "task_type": task_type,
    "Pos": pos_t,
    "Speed": v_t,
    "N": target_number,
    "tf": tf_t_loop,
    "lambda": lambda_val,
    "task_type": task_type,
    "task_type1": task_type1,
    "tloiter": tloiter_t,
    "radius": radius_t,
    "radius_t_2": radius_t_2,
    "angle": targets_angle,
    "restCheck": targets_restCheck,
    "completed": np.zeros(target_number),
    "flag_changeTargets": flag_changeTargets,
}

completed_agents = []
assigned_tasks = []
Xfinal = [None] * agent_number
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
    for i in range(agent_number):
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

    if flag and target_number != 0:
        _, p_GCAA_tmp, taskInd, _, _, Agents = GCAASolution_revised(Agents, G, Tasks)
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
        keep_tasks = [i for i in range(target_number) if i not in assigned_tasks]

        assigned_tasks1 = [x - 1 for x in assigned_tasks]
        pos_t = np.delete(pos_t, assigned_tasks1, axis=0)
        v_t = np.delete(v_t, assigned_tasks1, axis=0)
        target_number -= len(assigned_tasks)

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
        Tasks["N"] = target_number
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

    plotMapAllocation_stage2(Xfinal, agent_number, colors, "Planned Path")
    # plt.legend(legendUnq(plt.gca()))
    plt.draw()
    plt.pause(0.00001)

    # Update position and velocity of each agent
    for i in range(agent_number):
        if Xfinal[i] is None or len(Xfinal[i][0]) == 0:
            tmp = int(p_GCAA[i] - 1)
            if tmp not in completed_tasks_Store and tmp:
                completed_agents.append(i)
                completed_tasks_round.append(tmp)
                flag = 1
                if int(flag_changeTargets[tmp]) == 0:
                    completed_tasks_Store.append(tmp)
        else:
            if Xfinal[i] == []:
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
            if ind1[j] != 0 and flag_changeTargets[completed_tasks_round[j]] == 0:
                pos_t_tmp = np.delete(Tasks_initial["Pos"], ind1[j], axis=0)
                radius_t_tmp = np.delete(Tasks_initial["radius"], ind1[j], axis=0)
                radius_t_2_tmp = np.delete(Tasks_initial["radius_t_2"], ind1[j], axis=0)
                task_type1_tmp = np.delete(Tasks_initial["task_type1"], ind1[j], axis=0)
                Tasks_initial["Pos"] = pos_t_tmp
                Tasks_initial["radius"] = radius_t_tmp
                Tasks_initial["radius_t_2"] = radius_t_2_tmp
                Tasks_initial["task_type1"] = task_type1_tmp

            elif flag_changeTargets[completed_tasks_round[j]] != 0:
                ind2 = completed_tasks_round[j]
                ind3 = np.where(changeTargets[0] == ind2)
                pos_t = np.vstack((pos_t, pos_t_initial[ind2]))
                pos_t = np.vstack((pos_t, pos_t_initial[ind2]))
                v_t = np.vstack((v_t, [0, 0]))
                target_number += 1
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
                Tasks["N"] = target_number
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

        Tasks["prob_a_t"] = np.random.rand(na_new, target_number)

        n_rounds_loop = n_rounds

    if len(completed_tasks_Store) == nt_ini:
        break

plt.clf()
plt.xlim([0, map_width])
plt.ylim([0, map_width])
plt.xlabel("x [m]")
plt.ylabel("y [m]")
plt.title("Stage 2: Static Target Exploration", fontsize=15)
for i in range(agent_number):
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
plt.show()
