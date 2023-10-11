import numpy as np
from scipy.spatial import ConvexHull, convex_hull_plot_2d
import matplotlib.pyplot as plt
from shapely.geometry import Point, LineString, Polygon


def distance(a, b):
    return np.linalg.norm(np.array(a) - np.array(b))


def vertexsToConvex(forbiddenAreas):
    concevHulls = []
    for area in forbiddenAreas:
        concevHulls.append(ConvexHull(area))
    return concevHulls


def isInsideConvexHull(point, polygon):
    length = len(polygon)
    intersections = 0

    dx2 = point[0] - polygon[0][0]
    dy2 = point[1] - polygon[0][1]
    ii = 0
    jj = 1

    while jj < length:
        dx = dx2
        dy = dy2
        dx2 = point[0] - polygon[jj][0]
        dy2 = point[1] - polygon[jj][1]

        F = (dx - dx2) * dy - dx * (dy - dy2)
        if 0.0 == F and dx * dx2 <= 0 and dy * dy2 <= 0:
            return 2

        if (dy >= 0 and dy2 < 0) or (dy2 >= 0 and dy < 0):
            if F > 0:
                intersections += 1
            elif F < 0:
                intersections -= 1

        ii = jj
        jj += 1
    return intersections != 0


def orientation(p, q, r):
    val = (q[1] - p[1]) * (r[0] - q[0]) - (q[0] - p[0]) * (r[1] - q[1])
    if val == 0:
        return 0  # Collinear
    return 1 if val > 0 else 2  # Clockwise or Counterclockwise


def on_segment(p, q, r):
    return (
        q[0] <= max(p[0], r[0])
        and q[0] >= min(p[0], r[0])
        and q[1] <= max(p[1], r[1])
        and q[1] >= min(p[1], r[1])
    )


def do_intersect(p1, q1, p2, q2):
    o1 = orientation(p1, q1, p2)
    o2 = orientation(p1, q1, q2)
    o3 = orientation(p2, q2, p1)
    o4 = orientation(p2, q2, q1)

    if (
        (o1 != o2 and o3 != o4)
        or (o1 == 0 and on_segment(p1, p2, q1))
        or (o2 == 0 and on_segment(p1, q2, q1))
        or (o3 == 0 and on_segment(p2, p1, q2))
        or (o4 == 0 and on_segment(p2, q1, q2))
    ):
        return True

    return False


def islinePolygonIntersect(A, B, polygon_vertices):
    if isInsideConvexHull(A, polygon_vertices) or isInsideConvexHull(
        B, polygon_vertices
    ):
        return True
    for i in range(len(polygon_vertices)):
        j = (i + 1) % len(polygon_vertices)
        if do_intersect(A, B, polygon_vertices[i], polygon_vertices[j]):
            return True
    return False


def calculate_central_points(
    forbiddenAreas,
):
    central_points = []
    for forbidArea in forbiddenAreas:
        central_point = np.mean(forbidArea, axis=0)
        central_points.append(central_point)
    return np.array(central_points)


def fillbetween(start, end, forbiddenAreas, forbidArea_center, mini_distance=0.8):
    push_distance = 0.5
    if distance(start, end) < mini_distance:
        return [end]

    num_points = int(distance(start, end) / mini_distance)
    points = np.linspace(start, end, num_points, endpoint=False)

    for i, forbidArea in enumerate(forbiddenAreas):
        forbidArea = np.array(forbidArea).tolist()
        interplot_points = [start]
        a = None
        b = None
        previous_point = start
        for point in points:
            if islinePolygonIntersect(point, previous_point, forbidArea):
                a = previous_point
                break
            else:
                interplot_points.append(point)
                previous_point = point

        for point in points:
            if not isInsideConvexHull(point, forbidArea) and islinePolygonIntersect(
                point, previous_point, forbidArea
            ):
                b = point
                break
            else:
                previous_point = point
        if a is not None and b is not None:
            next_point = min(forbidArea, key=lambda x: distance(x, a))
            adjusted_next_point = pushPoint(
                forbidArea_center, push_distance, i, next_point
            )
            interplot_points.append(adjusted_next_point)
            # next_point = forbidArea[
            #     (forbidArea.index(closest_vertex) - 1) % len(forbidArea)
            # ]
            # _next_point = pushPoint(forbidArea_center, push_distance, i, next_point)
            # interplot_points.append(_next_point)
            while islinePolygonIntersect(
                adjusted_next_point, b, forbidArea
            ) or islinePolygonIntersect(adjusted_next_point, b, forbidArea):
                interplot_points.append(adjusted_next_point)
                next_point = forbidArea[
                    (forbidArea.index(next_point) - 1) % len(forbidArea)
                ]
                adjusted_next_point = pushPoint(
                    forbidArea_center, push_distance, i, next_point
                )
            adjusted_next_point = pushPoint(
                forbidArea_center, push_distance, i, next_point
            )
            interplot_points.append(adjusted_next_point)

            interplot_points.append(b)
            next_point_index = None
            for i in range(len(points)):
                if distance(points[i], b) < 1e-3:
                    next_point_index = i
                    break
            for i in range(next_point_index + 1, len(points)):
                interplot_points.append(points[i])
            # del points
            points = interplot_points

    return interplot_points


def pushPoint(forbidArea_center, push_distance, i, closest_vertex):
    vectorCenter = closest_vertex - forbidArea_center[i]
    adjusted_next_point = closest_vertex + (
        push_distance * vectorCenter / np.linalg.norm(vectorCenter)
    )

    return adjusted_next_point


if __name__ == "__main__":
    start = [-5, 7.5]
    end = [10, 7.5]
    forbiddenAreas = np.array(
        [
            [
                [5, 4],
                [5, 8],
                [8, 8],
                [8, 4],
                [5.01, 4.001],
            ],
        ]
    )
    centralPoints = calculate_central_points(forbiddenAreas)

    path = fillbetween(start, end, forbiddenAreas, centralPoints)
    plt.xlim(-5, 15)
    plt.ylim(-5, 15)
    for area in forbiddenAreas:
        area = np.array(area)
        plt.fill(area[:, 0], area[:, 1], "g", alpha=0.3)
    for point in path:
        plt.plot(point[0], point[1], "ro")
        # plt.pause(0.1)
        # plt.clf()

    plt.show()

# import matplotlib.pyplot as plt
# from matplotlib.patches import Polygon


# def plot_line_polygon(line_start, line_end, polygon_vertices):
#     intersects = islinePolygonIntersect(line_start, line_end, polygon_vertices)

#     plt.figure()
#     plt.plot([line_start[0], line_end[0]], [line_start[1], line_end[1]], "bo-")

#     polygon = Polygon(polygon_vertices, fill=None, edgecolor="r")
#     plt.gca().add_patch(polygon)

#     plt.scatter(*zip(*polygon_vertices), c="r", marker="o")

#     if intersects:
#         plt.title("Line intersects polygon")
#     else:
#         plt.title("Line does not intersect polygon")

#     plt.xlim(
#         min(line_start[0], line_end[0], min(polygon_vertices, key=lambda x: x[0])[0])
#         - 1,
#         max(line_start[0], line_end[0], max(polygon_vertices, key=lambda x: x[0])[0])
#         + 1,
#     )
#     plt.ylim(
#         min(line_start[1], line_end[1], min(polygon_vertices, key=lambda x: x[1])[1])
#         - 1,
#         max(line_start[1], line_end[1], max(polygon_vertices, key=lambda x: x[1])[1])
#         + 1,
#     )

#     plt.gca().set_aspect("equal", adjustable="box")
#     plt.grid()
#     plt.show()


# # Test cases
# line_start1 = (-3, -3)
# line_end1 = (-4, -5)
# polygon1 = [(2, 2), (2, 6), (6, 6), (6, 2)]
# plot_line_polygon(line_start1, line_end1, polygon1)

# line_start2 = (1, 1)
# line_end2 = (7, 7)
# polygon2 = [(2, 2), (2, 6), (6, 6), (6, 2)]
# plot_line_polygon(line_start2, line_end2, polygon2)

# line_start3 = (2, 4)
# line_end3 = (5, 4)
# polygon3 = [(2, 2), (2, 6), (6, 6), (6, 2)]
# plot_line_polygon(line_start3, line_end3, polygon3)

# line_start4 = (1, 3)
# line_end4 = (5, 7)
# polygon4 = [(2, 2), (2, 6), (6, 6), (6, 2)]
# plot_line_polygon(line_start4, line_end4, polygon4)

# line_start5 = (1, 3)
# line_end5 = (3, 5)
# polygon5 = [(2, 2), (2, 6), (6, 6), (6, 2)]
# plot_line_polygon(line_start5, line_end5, polygon5)
