import numpy as np
from scipy.spatial import ConvexHull
from shapely.geometry import Polygon

def findStrips(x, y, sidelap, imageWidth, imageLength):
    # Calculating the convex hull of the region of interest
    points = np.column_stack((x, y))
    hull = ConvexHull(points)

    # Only consider the points that define the convex hull
    x = points[hull.vertices, 0]
    y = points[hull.vertices, 1]

    # Rotating the region to find the scanning angle that uses the minimum number of lines
    areaWidth = np.max(x) - np.min(x)
    thetamin = 0
    for i in range(360):
        theta = i * 2 * np.pi / 360
        R = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
        rotated_points = np.dot(R, points.T).T
        rotated_x = rotated_points[hull.vertices, 0]
        rotated_y = rotated_points[hull.vertices, 1]
        rotated_area_width = np.max(rotated_x) - np.min(rotated_x)
        if rotated_area_width < areaWidth:
            areaWidth = rotated_area_width
            thetamin = theta

    # Rotate the region to the chosen angle
    R = np.array([[np.cos(thetamin), -np.sin(thetamin)], [np.sin(thetamin), np.cos(thetamin)]])
    rotated_points = np.dot(R, points.T).T
    x = rotated_points[hull.vertices, 0]
    y = rotated_points[hull.vertices, 1]

    areaWidth = np.max(x) - np.min(x)
    areaLength = np.max(y) - np.min(y)
    numberOfLanes = int(np.ceil(areaWidth / (imageWidth * (1 - sidelap))))
    laneDist = areaWidth / numberOfLanes
    lanemin = np.zeros((numberOfLanes, 2))
    lanemax = np.zeros((numberOfLanes, 2))

    for i in range(numberOfLanes):
        xi = np.min(x) + laneDist * i - laneDist / 2
        delta = areaLength / imageLength
        k = 0
        miny = np.min(y) + k * delta
        while not Polygon(np.column_stack((x, y))).contains(Point(xi, miny)):
            miny = np.min(y) + k * delta
            k = k + 1

        k = 0
        maxy = np.max(y) - k * delta
        while not Polygon(np.column_stack((x, y))).contains(Point(xi, maxy)):
            maxy = np.max(y) - k * delta
            k = k + 1

        lanemin[i, :] = [xi, miny]
        lanemax[i, :] = [xi, maxy]

    lmin = np.dot(R.T, lanemin.T).T
    lmax = np.dot(R.T, lanemax.T).T

    # Constructing the vertices
    V = np.zeros((numberOfLanes * 2 + 1, 2))
    for i in range(numberOfLanes * 2 + 1):
        if i == 0:
            V[i, :] = [0, 0]
        elif i % 2 == 0:
            V[i, :] = lmin[i // 2, :]
        else:
            V[i, :] = lmax[(i - 1) // 2, :]

    return lmin, lmax, V, laneDist

