
import numpy as np
from GreedyCoalitionAuctionAlgorithm import lla2ecef
import matplotlib.pyplot as plt

# Stage 1: 区域覆盖及静目标搜索
# Initial parameter settings
uniform_agents = 0
uniform_tasks = 1  # Each task's hovering radius is consistent
plot_range = 1
na = 8
nt = 34
n_rounds = 2000

Lt = 1
nt_loiter = int(0 * nt)  # Coefficient of nt determines the number of circling agents
task_type = np.zeros((nt, 1))  # Which agents circle, which agents approach directly
task_type[0:nt_loiter] = 1
task_type1 = np.ones((nt, 1))
lambda_val = 1

map_width = 7000
comm_distance = 0.01 * map_width

simu_time = 10
time_step = 0.05
time_start = 0

vesPosition = np.array([[121.64994868, 38.82776073],
                        [121.64994868, 38.82775873],
                        [121.64994868, 38.82775673],
                        [121.64994868, 38.82775473],
                        [121.64994868, 38.82775273],
                        [121.64994868, 38.82775073],
                        [121.64994868, 38.82774873],
                        [121.64994868, 38.82774673]])
vesPosition_3 = np.zeros((vesPosition.shape[0], 3))
vesPosition_3[:, 0:2] = vesPosition
p_agent = lla2ecef(vesPosition_3, 'WGS84')
p_agent = p_agent[:, :2]

targets_pos_value = np.array([[121.66111478389999, 38.805913105872833],
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
                              [121.71204920406547, 38.80605631718123]])
targets_pos_value_3 = np.zeros((targets_pos_value.shape[0], 3))
targets_pos_value_3[:, 0:2] = targets_pos_value
p_targets = lla2ecef(targets_pos_value_3, 'WGS84')
p_targets = p_targets[:, :2]

taskArea = np.array([[121.66473222, 38.85573388],
                     [121.71089122, 38.85573388],
                     [121.71940282, 38.82943606],
                     [121.71940282, 38.79479375],
                     [121.70958175, 38.76925451],
                     [121.66505958, 38.76925451],
                     [121.65572957, 38.79441445],
                     [121.65572957, 38.82791887],
                     [121.66473222, 38.85573388]])
taskArea_3 = np.zeros((taskArea.shape[0], 3))
taskArea_3[:, 0:2] = taskArea
p_task = lla2ecef(taskArea_3, 'WGS84')
p_task = p_task[:, :2]

p_all = np.concatenate((p_agent, p_targets, p_task))

p_agent[:, 0] = p_agent[:, 0] - np.min(p_all[:, 0])
p_agent[:, 1] = p_agent[:, 1] - np.min(p_all[:, 1])
pos_a = p_agent

p_targets[:, 0] = p_targets[:, 0] - np.min(p_all[:, 0])
p_targets[:, 1] = p_targets[:, 1] - np.min(p_all[:, 1])
pos_t = p_targets

p_task = p_task.T
p_task[0, :] = p_task[0, :] - np.min(p_all[:, 0])
p_task[1, :] = p_task[1, :] - np.min(p_all[:, 1])

targets_angle = np.array([60, 210, 105, 135, 30, 240, 120, 165, 15, 45, 75, 330, 60, 255, 180, 225, 285, 120, 75, 150, 300, 90, 270, 285, 240])

targets_restCheck = np.array([3, 4, 4, 5, 4, 3, 3, 4, 5, 4, 3, 4, 5, 4, 3, 5, 4, 5, 4, 4, 5, 4, 5, 4, 5])

x = p_task[0, :]
y = p_task[1, :]
lmin, lmax, V, laneDist = findStrips(x, y, 0, 300, 300)
lmin = lmin[np.argsort(lmin[:, 0])]
lmax = lmax[np.argsort(lmax[:, 0])]
pos_waypoints = np.concatenate((lmin, lmax))

tf_t = simu_time * (1.95 + 0.05 * np.random.rand(nt, 1))  # tf_t will affect task allocation results
tloiter_t = simu_time * (0.2 + 0.05 * np.random.rand(nt, 1))
tloiter_t[task_type == 0] = 0

pos_t_initial = pos_waypoints

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
Tasks['r_bar'] = r_bar
Tasks['prob_a_t'] = prob_a_t
Tasks['task_type'] = task_type

Agents = {}
Agents['N'] = na
Agents['Lt'] = Lt * np.ones((1, na))
Agents['v_a'] = v_a
Agents['previous_task'] = np.zeros((na, 1))
Agents['previous_winnerBids'] = np.zeros((na, 1))
Agents['rin_task'] = np.zeros((na, 2))
Agents['vin_task'] = np.zeros((na, 2))
Agents['kdrag'] = kdrag

# Fully connected graph
G = np.logical_not(np.eye(Agents['N']))

n_rounds_loop = n_rounds
simu_time_loop = simu_time
time_start_loop = time_start
tf_t_loop = tf_t
pos_a_loop = pos_a
v_a_loop = v_a

completed_tasks_round = []
completed_tasks = []

i_round = 1

p_GCAA = [1, 3, 5, 7, 9, 11, 13, 15]
ind_completed = []
targets_searched = []
dynamic_flag = np.zeros((na, 1))

# Core code for Stage 1

while np.sum(np.array(p_GCAA)) != 0:
    mat_p_GCAA = np.array(p_GCAA)
    mat_p_GCAA = mat_p_GCAA[mat_p_GCAA != 0]
    pos_waypoint = pos_waypoints[mat_p_GCAA-1, :]

    plt.clf()
    plt.xlim([0, map_width])
    plt.ylim([0, map_width])
    plt.xlabel('x [m]')
    plt.ylabel('y [m]')
    plt.title('Stage 1: Area Coverage and Targets Searched: ' + str(targets_searched.shape[0]), fontsize=20)

    for i in range(na):
        plt.plot(pos_a_loop[i, 0], pos_a_loop[i, 1], '*', color=colors[i], markersize=10, label='Agents')
        for j in range(len(p_targets)):
            if np.linalg.norm(pos_a_loop[i, :] - p_targets[j, :]) <= 300:
                plt.plot(p_targets[j, 0], p_targets[j, 1], 'rs', markersize=10, label='Targets', markerfacecolor=[1, .6, .6])
                if not np.any(np.all(targets_searched == p_targets[j, :], axis=1)):
                    targets_searched = np.vstack((targets_searched, p_targets[j, :]))

    if targets_searched.size > 0:
        plt.plot(targets_searched[:, 0], targets_searched[:, 1], 'rs', markersize=10, label='Targets', markerfacecolor=[1, .6, .6])

    for w in range(len(pos_waypoint)):
        if w+1 not in completed_tasks:
            plt.plot(pos_waypoint[w, 0], pos_waypoint[w, 1], 'ko', linewidth=3, markersize=15, label='Waypoints')

    plt.plot(p_task[0, :], p_task[1, :], 'k', label='Boundary', linewidth=2)

    Agents['Pos'] = pos_a_loop
    Agents['v_a'] = v_a_loop

    Tasks['Pos'] = pos_waypoints
    Tasks['Speed'] = v_t
    Tasks['N'] = nt
    Tasks['tf'] = tf_t_loop
    Tasks['lambda'] = lambda_val
    Tasks['task_type'] = task_type
    Tasks['tloiter'] = tloiter_t
    Tasks['radius'] = radius_t
    Tasks['angle'] = targets_angle
    Tasks['restCheck'] = targets_restCheck

    X, completed_tasks_round = OptimalControlSolution_stage1(pos_a_loop, v_a_loop, pos_waypoints, v_t, radius_t, p_GCAA, Agents, tf_t_loop, tloiter_t, time_step, n_rounds_loop, na, kdrag)

    plotMapAllocation_stage1(X, n_rounds_loop, na, colors, 'Planned Path')

    plt.legend(loc='upper right')

    plt.draw()
    plt.pause(0.001)

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
                p_GCAA[ind_completed] += 17
                dynamic_flag[ind_completed] += 1
            elif dynamic_flag[ind_completed] == 1:
                p_GCAA[ind_completed] += 1
                dynamic_flag[ind_completed] += 1
            elif dynamic_flag[ind_completed] == 2:
                p_GCAA[ind_completed] -= 17
                dynamic_flag[ind_completed] += 1
            elif dynamic_flag[ind_completed] == 3:
                p_GCAA[ind_completed] = 0

        Agents['N'] = na
        Agents['Lt'] = Lt * np.ones((1, na))
        Agents['previous_task'] = np.zeros((na, 1))
        Agents['previous_winnerBids'] = np.zeros((na, 1))
        Agents['rin_task'] = np.zeros((na, 2))
        Agents['vin_task'] = np.zeros((na, 2))
        Agents['kdrag'] = kdrag

        n_rounds_loop = n_rounds

    i_round += 1
plt.clf()
plt.xlim([0, map_width])
plt.ylim([0, map_width])
plt.xlabel('x [m]')
plt.ylabel('y [m]')
for i in range(na):
    plt.plot(pos_a_loop[i, 0], pos_a_loop[i, 1], '*', color=colors[i], markersize=10, label='Agents')

plt.plot(p_task[0, :], p_task[1, :], 'k', label='Boundary', linewidth=2)
plt.title('Stage 2: Static Target Exploration', fontsize=20)
plt.legend(loc='upper right')
plt.draw()
plt.pause(0.001)

# Stage 2: Static Target Exploration

# Initial parameter settings
na = 8
nt = 25
nt_ini = nt
n_rounds = 5000

targets_angle = np.array([60, 210, 105, 135, 30, 240, 120, 165, 15, 45, 75, 330, 60, 255, 180, 225, 285, 120, 75, 150, 300, 90, 270, 285, 240])
targets_restCheck = np.array([3, 4, 4, 5, 4, 3, 3, 4, 5, 4, 3, 4, 5, 4, 3, 5, 4, 5, 4, 4, 5, 4, 5, 4, 5])
changeTargetsNo = 5
changeTargets = np.array([[5, 10, 15, 20, 25],
                          [45, 60, 180, 135, 120],
                          [4, 4, 3, 4, 5],
                          [2, 1, 2, 1, 1]]).T  # column: id, angle, restCheck, changeCheck

targets_restCheck[changeTargets[:, 0] - 1] = changeTargets[:, 3]

pos_t_initial = np.copy(pos_t)
pos_t_initial = np.hstack((pos_t_initial, np.arange(1, nt+1).reshape(-1, 1)))

R = 300  # task outer orbit radius
R2 = 50  # threat zone radius
radius_t = R * np.ones(nt)
radius_t_2 = R2 * np.ones(nt)

Lt = 1
task_type = np.zeros(nt)
task_type1 = np.ones(nt)
lambda_val = 1

map_width = 7000
comm_distance = 0.01 * map_width

simu_time = 10
time_step = 0.05
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
if not uniform_agents:
    prob_a_t = np.random.rand(na, nt)

Tasks = {}
Tasks['r_bar'] = r_bar
Tasks['prob_a_t'] = prob_a_t
Tasks['task_type'] = task_type

Agents = {}
Agents['N'] = na
Agents['Lt'] = Lt * np.ones(na)
Agents['v_a'] = v_a
Agents['previous_task'] = np.zeros(na)
Agents['previous_winnerBids'] = np.zeros(na)
Agents['rin_task'] = np.zeros((na, 2))
Agents['vin_task'] = np.zeros((na, 2))
Agents['kdrag'] = kdrag

costs = np.zeros((na, nt))
utility = np.zeros((na, nt))
rewards = np.zeros((na, nt))

G = np.logical_not(np.eye(Agents['N']))

colors = plt.cm.lines(np.arange(na))

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
import numpy as np

flag = 1
nt = len(changeTargets[0])
flag_changeTargets = np.ones(nt)
flag_changeTargets[changeTargets[0]] += 1

Agents = {'Pos': pos_a_loop, 'v_a': v_a_loop}

pos_t = np.hstack((pos_t, np.arange(1, nt+1).reshape(-1, 1)))
Tasks = {'Pos': pos_t, 'Speed': v_t, 'N': nt, 'tf': tf_t_loop, 'lambda': lambda_, 
         'task_type': task_type, 'task_type1': task_type1, 'tloiter': tloiter_t,
         'radius': radius_t, 'radius_t_2': radius_t_2, 'angle': targets_angle, 
         'restCheck': targets_restCheck, 'completed': np.zeros(nt), 'flag_changeTargets': flag_changeTargets}

completed_agents = []
assigned_tasks = []
Xfinal = [None] * na
Tasks_initial = Tasks.copy()
completed_tasks_Store = []

for i_round in range(1, n_rounds+1):
    completed_tasks_round=[]
    # Initial Plot Code here
    # ...
    # ...

    if flag and nt != 0:
        # GCAASolution_revised, OptimalControlSolution_stage2 related code here
        # ...
        # ...

        flag = 0
        completed_agents = []

        flag_changeTargets[taskInd] -= 1

        mask = np.ones(nt, dtype=bool)
        mask[assigned_tasks] = False
        pos_t = pos_t[mask]
        v_t = v_t[mask]
        nt -= len(assigned_tasks)
        tf_t = tf_t[mask]
        tf_t_loop = tf_t
        task_type = task_type[mask]
        task_type1 = task_type1[mask]
        tloiter_t = tloiter_t[mask]
        radius_t = radius_t[mask]
        radius_t_2 = radius_t_2[mask]
        Tasks['r_bar'] = Tasks['r_bar'][mask]
        Tasks['prob_a_t'] = Tasks['prob_a_t'][:, mask]
        Tasks.update({'Pos': pos_t, 'Speed': v_t, 'N': nt, 'tf': tf_t_loop, 'lambda': lambda_, 
                     'task_type': task_type, 'tloiter': tloiter_t, 'radius': radius_t, 
                     'angle': Tasks['angle'][mask], 'restCheck': Tasks['restCheck'][mask], 
                     'flag_changeTargets': flag_changeTargets[mask]})

        assigned_tasks = []

    # plotMapAllocation_stage2, legendUnq related code here
    # ...
    # ...

    # Update position and velocity of each agent
    for i in range(na):
        if Xfinal[i] is None:
            tmp = p_GCAA[i]
            if tmp not in completed_tasks_Store and tmp: 
                completed_agents.append(i)
                completed_tasks_round.append(tmp)
                flag = 1
                if flag_changeTargets[tmp] == 0:
                    completed_tasks_Store.append(tmp)
        else:
            pos_a_loop[i] = Xfinal[i][0:2,0]
            v_a_loop[i]   = Xfinal[i][2:4,0]
            Xfinal[i] = np.delete(Xfinal[i], 0, 1)

    n_rounds_loop -= 1
    simu_time_loop -= time_step
    time_start_loop += time_step
    tf_t_loop -= time_step

    # Code for ~isempty(completed_tasks_round)
    # ...
    # ...

    if len(completed_tasks_Store) == nt_ini:
        break
plt.clf()
plt.xlim([0, map_width])
plt.ylim([0, map_width])
plt.xlabel('x [m]')
plt.ylabel('y [m]')
plt.title('Stage 2: Static Target Exploration', fontsize=20)
for i in range(na):
    plt.plot(pos_a_loop[i, 0], pos_a_loop[i, 1], '*', color=colors[i], markersize=10, label='Agents')

plt.plot(pos_t[:, 0], pos_t[:, 1], 'rs', markersize=8, label='Targets', markerfacecolor=[1, 0.6, 0.6])
plt.plot(p_task[0, :], p_task[1, :], 'k', label='Boundary', linewidth=2)
plt.legend(loc='upper right')
plt.draw()
plt.pause(0.001)

def remove_completed_tasks(pos_t, ind):
    pos_t_new = np.delete(pos_t, ind, axis=0)
    return pos_t_new

def update_path(p, pos_a, pos_t, time_step, Agents, nt):
    ind_completed_tasks = []

    for i in range(pos_a.shape[0]):
        if len(p[i]) != 0:
            d_a_t = pos_t[p[i][0]] - pos_a[i]
            if np.linalg.norm(d_a_t) < time_step * Agents['Speed'][i]:
                pos_a[i] = pos_t[p[i][0]]
                nt -= 1
                Agents['Lt'][i] -= 1
                ind_completed_tasks.append(p[i][0])
                
                p[i] = p[i][1:]
                # if len(p[i]) != 0:
                #     time_step_remaining = time_step - np.linalg.norm(d_a_t) / Agents['Speed'][i]
                #     d_a_next_t = pos_t[p[i][0]] - pos_a[i]
                #     pos_a[i] += d_a_next_t / np.linalg.norm(d_a_next_t) * time_step_remaining * Agents['Speed'][i]
            else:
                pos_a[i] += d_a_t / np.linalg.norm(d_a_t) * time_step * Agents['Speed'][i]
    
    return p, pos_a, ind_completed_tasks, nt, Agents

