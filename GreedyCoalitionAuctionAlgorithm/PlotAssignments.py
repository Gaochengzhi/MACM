import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def PlotAssignments(WORLD, GCAA_Assignments, agents, tasks, figureID):
    # Set plotting parameters
    plt.rcParams['font.size'] = 12
    plt.rcParams['axes.labelsize'] = 10
    plt.rcParams['axes.titlesize'] = 10
    plt.rcParams['font.family'] = 'arial'
    plt.rcParams['lines.linewidth'] = 2
    plt.rcParams['lines.markersize'] = 10

    # Plot X and Y agent and task positions vs. time
    offset = (WORLD['XMAX'] - WORLD['XMIN']) / 100
    fig = plt.figure(figureID)
    ax = fig.add_subplot(111, projection='3d')
    Cmap = plt.cm.get_cmap('tab10')

    # Plot tasks
    for m, task in enumerate(tasks):
        ax.plot([task['x'], task['x']], [task['y'], task['y']], [task['start'], task['end']], 'x:', color=Cmap(task['type']), linewidth=3)
        ax.text(task['x'] + offset, task['y'] + offset, task['start'], f'T{m}')

    # Plot agents
    for n, agent in enumerate(agents):
        ax.plot([agent['x']], [agent['y']], [0], 'o', color=Cmap(agent['type']))
        ax.text(agent['x'] + offset, agent['y'] + offset, 0.1, f'A{n}')
        path = GCAA_Assignments[n]['path']
        if path[0] != -1:
            taskPrev = lookupTask(tasks, path[0])
            X = [agent['x'], taskPrev['x']]
            Y = [agent['y'], taskPrev['y']]
            T = [0, GCAA_Assignments[n]['times'][0]]
            ax.plot(X, Y, T, '-', color=Cmap(agent['type']))
            ax.plot([X[-1]], [Y[-1]], [T[1]], '-^', color=Cmap(agent['type']))
            agent['time'] = round(GCAA_Assignments[n]['times'][0] / 100)
            agent['path'] = [[taskPrev['x'], taskPrev['y'], agent['z'], taskPrev['duration'] / 100, taskPrev['id']]]
            for m in range(1, len(path)):
                if path[m] != -1:
                    taskNext = lookupTask(tasks, path[m])
                    X = [taskPrev['x'], taskNext['x']]
                    Y = [taskPrev['y'], taskNext['y']]
                    T = [GCAA_Assignments[n]['times'][m - 1] + taskPrev['duration'], GCAA_Assignments[n]['times'][m]]
                    ax.plot(X, Y, T, '-^', color=Cmap(agent['type']))
                    ax.plot([X[-1]], [Y[-1]], [T[1]], '-^', color=Cmap(agent['type']))
                    agent['path'].append([taskNext['x'], taskNext['y'], agent['z'], taskNext['duration'] / 100, taskNext['id']])
                    taskPrev = taskNext
                else:
                    break

    ax.set_title('Agent Paths with Time Windows')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Time')
    ax.grid(True)

    # Plot agent schedules
    fig2, axes = plt.subplots(len(agents), 1, figsize=(8, 6), sharex=True, sharey=True)
    fig2.suptitle('Agent Schedules')
    for n, agent in enumerate(agents):
        axes[n].set_ylabel(f'A{agent["id"]}')
        axes[n].grid(True)
        axes[n].set_xlim(0, 1600)
        axes[n].set_ylim(0, 2)
        for m, taskID in enumerate(GCAA_Assignments[n]['path']):
            if taskID != -1:
                taskCurr = lookupTask(tasks, taskID)
                axes[n].plot([GCAA_Assignments[n]['times'][m], GCAA_Assignments[n]['times'][m] + taskCurr['duration']], [1, 1], '-', color=Cmap(agent['type']), linewidth=10)
                axes[n].plot([taskCurr['start'], taskCurr['end']], [1, 1], '--', color=Cmap(agent['type']))
            else:
                break

    axes[-1].set_xlabel('Time')

    return fig, fig2


def lookupTask(tasks, taskID):
    for task in tasks:
        if task['id'] == taskID:
            return task
    print(f'Task with index={taskID} not found')
    return None

