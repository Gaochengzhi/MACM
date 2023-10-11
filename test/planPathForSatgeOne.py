import numpy as np
from scipy.spatial import ConvexHull
from matplotlib.path import Path


def findStripsForAgent(x, y, agents):
    points = np.column_stack((x, y))
    hull = ConvexHull(points)
    x = x[hull.vertices]
    y = y[hull.vertices]
    x = np.append(x, x[0])
    y = np.append(y, y[0])

    agents = sorted(agents, key=lambda k: k["id"])
    agentDetectRanges = [agent["certainRadius"] for agent in agents]

    # Assuming the areaWidth is to be calculated the same way
    areaWidth = max(x) - min(x)
    thetamin = 0
    for i in range(1, 361):
        theta = i * 2 * np.pi / 360
        R = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
        aux = np.matmul(R, np.column_stack((x, y)).T)
        if (max(aux[0, :]) - min(aux[0, :])) < areaWidth:
            areaWidth = max(aux[0, :]) - min(aux[0, :])
            thetamin = theta

    thetamin = thetamin + np.pi / 2
    R = np.array(
        [[np.cos(thetamin), -np.sin(thetamin)], [np.sin(thetamin), np.cos(thetamin)]]
    )
    aux = np.matmul(R, np.column_stack((x, y)).T)

    x = aux[0, :]
    x = np.append(x[1:], x[1])
    y = aux[1, :]
    y = np.append(y[1:], y[1])

    areaWidth = max(x) - min(x)
    areaLength = max(y) - min(y)

    # Calculating the number of lanes based on the agentDetectRanges
    numberOfLanes = len(agentDetectRanges)

    # Determine paths for each agent
    agentPaths = {}
    for index, agentRange in enumerate(agentDetectRanges):
        laneDist = areaWidth / (numberOfLanes - index)

        xi = min(x) + laneDist - laneDist / 2
        delta = areaLength / agentRange

        k = 0
        miny = min(y) + k * delta
        while not Path(
            np.column_stack((x[hull.vertices], y[hull.vertices]))
        ).contains_point((xi, miny)):
            miny = min(y) + k * delta
            k += 1

        k = 0
        maxy = max(y) - k * delta
        while not Path(
            np.column_stack((x[hull.vertices], y[hull.vertices]))
        ).contains_point((xi, maxy)):
            maxy = max(y) - k * delta
            k += 1

        agentPaths[agents[index]["id"]] = [(xi, miny), (xi, maxy)]

    return agentPaths
