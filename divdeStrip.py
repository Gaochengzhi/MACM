# convexWdith along axisCut
import numpy as np
from shapely.geometry import Polygon, Point, box
from scipy.optimize import minimize
from shapely import affinity
from scipy.interpolate import splprep, splev


def is_inside_polygon(polygon, point, margin=0):
    poly = Polygon(polygon)
    p = Point(point)
    return poly.buffer(margin).contains(p)


def filter_waypoints(interpolated_waypoints, forbiddenAreas, taskArea):
    filtered_waypoints = []
    for point in interpolated_waypoints:
        in_forbidden_area = any(
            is_inside_polygon(area, point, 20) for area in forbiddenAreas
        )
        in_task_area = is_inside_polygon(taskArea, point)
        if not in_forbidden_area and in_task_area:
            filtered_waypoints.append(point)
    return np.array(filtered_waypoints)


def smoothAndAvoidObstacles(wayPoints, agentPos, forbiddenAreas, taskArea):
    # First, interpolate the waypoints
    tck, u = splprep(wayPoints.T, u=None, s=0, k=2)
    u_new = np.linspace(u.min(), u.max(), 20)
    x_new, y_new = splev(u_new, tck)
    interpolated_waypoints = np.column_stack([x_new, y_new])
    filtered = filter_waypoints(interpolated_waypoints, forbiddenAreas, taskArea)
    return minimal_turning_path(filtered, agentPos)
    # return minimal_turning_path(interpolated_waypoints, agentPos)


def computeAngle(areaCenter, agentCenter):
    return np.arctan2(
        abs(agentCenter[1] - areaCenter[1]), abs(agentCenter[0] - areaCenter[0])
    )


def rotate_points(points, center, angle, clockwise=True):
    rotation_matrix = np.array(
        [[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]]
    )
    centered_points = points - center
    if not clockwise:
        rotation_matrix = np.array(
            [[np.cos(-angle), -np.sin(-angle)], [np.sin(-angle), np.cos(-angle)]]
        )
    rotated_points = centered_points.dot(rotation_matrix.T)
    final_points = rotated_points + center
    return final_points


from shapely.geometry import LineString, Point


def getIntersectionPoint(
    Ybroundary,
    rotated_taskArea,
    bias,
    min_x=-80000,
    max_x=80000,
):
    line = LineString([(x, Ybroundary) for x in range(min_x, max_x)])
    points = []

    for i in range(len(rotated_taskArea)):
        edge_start = rotated_taskArea[i]
        edge_end = rotated_taskArea[(i + 1) % len(rotated_taskArea)]
        edge = LineString([edge_start, edge_end])

        intersection = line.intersection(edge)

        if intersection.is_empty:
            continue
        elif isinstance(intersection, Point):
            if intersection.y - Ybroundary < 1e-10:
                points.append([float(intersection.x) + bias, Ybroundary])
        elif isinstance(intersection, LineString):
            for point in intersection:
                if point.y - Ybroundary < 1e-10:
                    points.append([float(point.x) + bias, Ybroundary])

    return points


def find_central_point(convex_area):
    if len(convex_area) == 0:
        return None
    centroid = np.mean(convex_area, axis=0)
    return centroid


def minimal_squares_centers(points, detectRadius):
    poly = Polygon(points)
    min_squares = float("inf")
    optimal_centers = []

    def count_squares(angle):
        nonlocal optimal_centers, min_squares
        rotated_poly = affinity.rotate(poly, angle, origin="centroid")
        minx, miny, maxx, maxy = rotated_poly.bounds
        edge_size = 2 * detectRadius
        centers = []
        for x in np.arange(minx, maxx, edge_size):
            for y in np.arange(miny, maxy, edge_size):
                center = Point(x + detectRadius, y + detectRadius)
                # if rotated_poly.contains(center):
                centers.append([center.x, center.y])
                # else move point to the inside of the polygon and squere overlap
                # else:
                #     center = rotated_poly.buffer(0).intersection(
                #         box(x, y, x + edge_size / 2, y + edge_size / 2).buffer(0)
                #     )
                #     if center:
                #         centers.append([center.centroid.x, center.centroid.y])
        num_squares = len(centers)
        if num_squares < min_squares:
            min_squares = num_squares
            optimal_centers = centers
        return num_squares

    result = minimize(count_squares, x0=0, bounds=[(0, 359)])
    optimal_angle = result.x[0]
    rotated_optimal_centers = [
        affinity.rotate(Point(*center), -optimal_angle, origin="centroid")
        for center in optimal_centers
    ]
    return np.array([[p.x, p.y] for p in rotated_optimal_centers])


#


# Get the centers of the minimal squares


def turning_angle(p1, p2, p3):
    a = np.array(p1)
    b = np.array(p2)
    c = np.array(p3)
    ba = a - b
    bc = c - b
    cosine_angle = np.dot(ba, bc) / (np.linalg.norm(ba) * np.linalg.norm(bc))
    angle = np.arccos(np.clip(cosine_angle, -1.0, 1.0))
    return angle


def minimal_turning_path(waypoints, agent_position, alpha=0.7):
    waypoints = [np.array(waypoint) for waypoint in waypoints]
    sorted_waypoints = [np.array(agent_position)]
    remaining_waypoints = waypoints.copy()

    while remaining_waypoints:
        if len(sorted_waypoints) > 1:
            next_waypoint = min(
                remaining_waypoints,
                key=lambda waypoint: alpha
                * turning_angle(sorted_waypoints[-1], sorted_waypoints[-2], waypoint)
                + (1 - alpha) * np.linalg.norm(waypoint - sorted_waypoints[-1]),
            )
        else:
            next_waypoint = min(
                remaining_waypoints,
                key=lambda waypoint: np.linalg.norm(waypoint - sorted_waypoints[-1]),
            )
        sorted_waypoints.append(next_waypoint)
        # Find the correct index to remove
        for i, waypoint in enumerate(remaining_waypoints):
            if np.array_equal(waypoint, next_waypoint):
                del remaining_waypoints[i]
                break

    return sorted_waypoints[1:]


def pathPlan(StripsVex, agents, detectRadius):
    agentPath = []
    for i in range(len(StripsVex)):
        tmp = minimal_squares_centers(StripsVex[i], detectRadius[i])
        sorted_waypoints = minimal_turning_path(tmp, agents[i]["position"])
        agentPath.append(sorted_waypoints)

    return agentPath


def divide_into_stripes(p_taskArea, agents, detectRadius, detectVelocity):
    p_agent = [agent["position"] for agent in agents]
    areaCenter = find_central_point(p_taskArea)
    agentCenter = find_central_point(p_agent)
    angles = computeAngle(areaCenter, agentCenter)
    rotated_taskArea = rotate_points(p_taskArea, areaCenter, angles, clockwise=False)
    topVex = np.max(rotated_taskArea[:, 1])
    bottomVex = np.min(rotated_taskArea[:, 1])
    convexHeight = topVex - bottomVex

    stripHeight = detectRadius / np.sum(detectRadius) * convexHeight
    stripHeight = stripHeight / np.mean(detectVelocity) * detectVelocity

    # topPoint = np.max(rotated_taskArea, axis=0)
    topPoint = np.array(rotated_taskArea[np.argmax(rotated_taskArea[:, 1])])
    # StripsVex = len(stripHeight),n, 2 list
    StripsVex = [[] for i in range(len(stripHeight))]
    for i in range(len(stripHeight)):
        bias = 0.2 * stripHeight[i]
        topAxis = None
        # if topPoint's shape is (2,)
        if topPoint.shape == (2,):
            StripsVex[i].append(topPoint)
            topAxis = topPoint[1] - bias
        else:
            for pointi in topPoint:
                StripsVex[i].append(pointi)
                topAxis = pointi[1] - bias
        if i < len(stripHeight) - 1:
            convexPoints = rotated_taskArea[
                np.where(
                    np.logical_and(
                        rotated_taskArea[:, 1] <= topAxis,
                        rotated_taskArea[:, 1] >= topAxis - stripHeight[i] + bias,
                    )
                )
            ]
            for point in convexPoints:
                StripsVex[i].append(point)
            intersectionPoints = getIntersectionPoint(
                topAxis - stripHeight[i] + bias, rotated_taskArea, bias=0.2 * bias
            )
            for point in intersectionPoints:
                StripsVex[i].append(point)
            topPoint = np.array(intersectionPoints)
        else:
            convexPoints = rotated_taskArea[
                np.where(
                    np.logical_and(
                        rotated_taskArea[:, 1] <= topAxis,
                        rotated_taskArea[:, 1] >= topAxis - stripHeight[i] - 100,
                    )
                )
            ]
            for point in convexPoints:
                StripsVex[i].append(point)
    rawres = pathPlan(StripsVex, agents, detectRadius)
    res = []
    for Strips in rawres:
        res.append(rotate_points(np.array(Strips), areaCenter, angles, clockwise=True))
    return res
