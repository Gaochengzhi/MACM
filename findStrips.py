import numpy as np
from scipy.spatial import ConvexHull, Delaunay

def findStrips(x, y, sidelap, imageWidth, imageLength):
    # Compute the convex hull of the region to be investigated
    hull = ConvexHull(np.vstack([x, y]).T)
    # Only the points that define the convex hull matter from this point
    x, y = x[hull.vertices], y[hull.vertices]
    # Rotate the region of interest to find the scanning angle that uses the smallest number of lines
    areaWidth = max(x) - min(x)
    thetamin = 0
    for i in range(1, 361):
        theta = i * 2 * np.pi / 360
        R = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
        aux = R @ np.vstack([x, y])
        if max(aux[0, :]) - min(aux[0, :]) < areaWidth:
            areaWidth = max(aux[0, :]) - min(aux[0, :])
            thetamin = theta
    # Rotate the region to the angle chosen in the previous step to facilitate subsequent calculations
    R = np.array([[np.cos(thetamin), -np.sin(thetamin)], [np.sin(thetamin), np.cos(thetamin)]])
    aux = R @ np.vstack([x, y])
    x, y = aux[0, :], aux[1, :]
    areaWidth = max(x) - min(x)
    areaLength = max(y) - min(y)
    numberOfLanes = np.ceil(areaWidth / (imageWidth * (1 - sidelap)))
    laneDist = areaWidth / numberOfLanes
    lanemin, lanemax = [], []
    for i in range(int(numberOfLanes)):
        xi = min(x) + laneDist * (i + 1) - laneDist / 2
        delta = areaLength / imageLength
        k = 0
        miny = min(y) + k * delta
        # Using Delaunay for inpolygon
        while not Delaunay(np.vstack([x, y]).T).find_simplex([xi, miny]) >= 0:
            miny = min(y) + k * delta
            k += 1
        k = 0
        maxy = max(y) - k * delta
        while not Delaunay(np.vstack([x, y]).T).find_simplex([xi, maxy]) >= 0:
            maxy = max(y) - k * delta
            k += 1
        lanemin.append([xi, miny])
        lanemax.append([xi, maxy])
    lmin = (R.T @ np.array(lanemin).T).T
    lmax = (R.T @ np.array(lanemax).T).T
    # Vertex construction
    V = np.zeros((int(numberOfLanes * 2 + 1), 2))
    for i in range(int(numberOfLanes * 2 + 1)):
        if i == 0:
            V[i, :] = [0, 0]
        elif i % 2 == 0:
            V[i, :] = lmin[i // 2-1, :]
        else:
            V[i, :] = lmax[(i - 1) // 2, :]
    return lmin, lmax, V, laneDist

