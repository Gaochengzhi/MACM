import numpy as np
import matplotlib.pyplot as plt

# Set random seed
np.random.seed(1)

N = 3
M = 8
Lt = 3

WORLD = {"XMAX": 100, "YMAX": 100, "XMIN": 0, "YMIN": 0, "CLR": np.random.rand(100, 3)}

# Define agent and task types
GCAA_Params = GCAA_Init(0, 0)
GCAA_Params.MAX_DEPTH = Lt

# Define agent default fields
agent_default = {
    "id": 0,
    "type": 0,
    "avail": 0,
    "clr": [],
    "x": 0,
    "y": 0,
    "z": 0,
    "nom_vel": 0,
    "fuel": 0,
}

# Define task default fields
task_default = {
    "id": 0,
    "type": 0,
    "value": 0,
    "start": 0,
    "end": 0,
    "duration": 0,
    "lambda": 0.95,
    "x": 0,
    "y": 0,
    "z": 0,
}

# Create default agents
agent_quad = agent_default.copy()
agent_quad["type"] = GCAA_Params["AGENT_TYPES"]["QUAD"]
agent_quad["nom_vel"] = 1
agent_quad["fuel"] = 1

agents = []
for n in range(N):
    agent = agent_quad.copy()
    agent["id"] = n
    agent["x"] = np.random.rand() * WORLD["XMAX"]
    agent["y"] = np.random.rand() * WORLD["YMAX"]
    agent["clr"] = WORLD["CLR"][n]
    agents.append(agent)

# Create default tasks
task_track = task_default.copy()
task_track["type"] = GCAA_Params["TASK_TYPES"]["TRACK"]
task_track["value"] = 1

tasks = []
for m in range(M):
    task = task_track.copy()
    task["id"] = m
    task["x"] = np.random.rand() * WORLD["XMAX"]
    task["y"] = np.random.rand() * WORLD["YMAX"]
    tasks.append(task)

# Define communication graph
Graph = np.ones((N, N), dtype=bool)
np.fill_diagonal(Graph, False)

# Run GCAA
GCAA_Assignments, Total_Score = GCAA_Main(agents, tasks, Graph)

# Print assignments and total score
for agent_assignment in GCAA_Assignments:
    agent_id = agent_assignment["agentID"]
    assigned_task = agent_assignment["path"]
    print(f"Agent {agent_id} assigned to task {assigned_task}")

print(f"Total Score: {Total_Score}")

# Plot assignments
for agent_assignment in GCAA_Assignments:
    agent_id = agent_assignment["agentID"]
    assigned_task = agent_assignment["path"]
    agent_color = agent_assignment["clr"]
    agent_x = agents[agent_id]["x"]
    agent_y = agents[agent_id]["y"]
    task_x = tasks[assigned_task]["x"]
    task_y = tasks[assigned_task]["y"]
    plt.plot([agent_x, task_x], [agent_y, task_y], color=agent_color)
    plt.plot(
        agent_x,
        agent_y,
        marker="o",
        markersize=8,
        color=agent_color,
        label=f"Agent {agent_id}",
    )
    plt.plot(
        task_x,
        task_y,
        marker="s",
        markersize=8,
        color="red",
        label=f"Task {assigned_task}",
    )
plt.legend()
plt.xlim(WORLD["XMIN"], WORLD["XMAX"])
plt.ylim(WORLD["YMIN"], WORLD["YMAX"])
