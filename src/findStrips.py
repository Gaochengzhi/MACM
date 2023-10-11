import numpy as np
from scipy.spatial import ConvexHull
from matplotlib.path import Path


def findStrips(x, y, sidelap, imageWidth, imageLength):
    points = np.column_stack((x, y))
    hull = ConvexHull(points)
    x = x[hull.vertices]
    y = y[hull.vertices]
    x = np.append(x, x[0])
    y = np.append(y, y[0])

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
    numberOfLanes = np.ceil(areaWidth / (imageWidth * (1 - sidelap)))
    laneDist = areaWidth / numberOfLanes

    lanemin = []
    lanemax = []
    # debug here

    for i in range(1, int(numberOfLanes) + 1):
        xi = min(x) + laneDist * i - laneDist / 2
        delta = areaLength / imageLength

        k = 0
        miny = min(y) + k * delta
        while not Path(
            np.column_stack((x[hull.vertices], y[hull.vertices]))
        ).contains_point((xi, miny)):
            miny = min(y) + k * delta
            k = k + 1

        k = 0
        maxy = max(y) - k * delta
        while not Path(
            np.column_stack((x[hull.vertices], y[hull.vertices]))
        ).contains_point((xi, maxy)):
            maxy = max(y) - k * delta
            k = k + 1

        lanemin.append([xi, miny])
        lanemax.append([xi, maxy])

    lanemin = np.array(lanemin)
    lanemax = np.array(lanemax)

    lmin = np.matmul(lanemin, R)
    lmax = np.matmul(lanemax, R)

    V = np.zeros((int(numberOfLanes * 2 + 1), 2))
    for i in range(int(numberOfLanes * 2)):
        if i == 0:
            V[i, :] = [0, 0]
        elif (i + 1) % 2 == 0:
            V[i, :] = lmin[i // 2 - 1, :]
        else:
            V[i, :] = lmax[(i - 1) // 2 - 1, :]
    lmin = lmin[np.argsort(lmin[:, 0])]
    lmax = lmax[np.argsort(lmax[:, 0])]
    return lmin, lmax, V, laneDist
