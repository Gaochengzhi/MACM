import numpy as np


def distance(p1, p2):
    return np.linalg.norm(p1 - p2)


def isInsideConvex(point, polygon):
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
            return True

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
    if isInsideConvex(A, polygon_vertices) or isInsideConvex(B, polygon_vertices):
        return True
    for i in range(len(polygon_vertices)):
        j = (i + 1) % len(polygon_vertices)
        if do_intersect(A, B, polygon_vertices[i], polygon_vertices[j]):
            return True
    return False


def fillbetween(
    start,
    end,
    forbiddenAreas,
    taskArea,
    forbidArea_center,
    mini_distance=200,
    push_distance=150,
):
    max_iterations = 30  # Set a maximum number of iterations for the while loop
    iteration_count = 0
    if distance(start, end) < mini_distance:
        return [end]
    # return [end]
    num_points = int(distance(start, end) / mini_distance)
    points = np.linspace(start, end, num_points, endpoint=False)
    interplot_points = [end]

    for i, forbidArea in enumerate(forbiddenAreas):
        forbidArea = np.array(forbidArea).tolist()
        lenForbidArea = len(forbidArea)
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
            if not isInsideConvex(point, forbidArea) and islinePolygonIntersect(
                point, previous_point, forbidArea
            ):
                b = point
                break
            else:
                previous_point = point
        if a is not None and b is not None:
            next_point = min(forbidArea, key=lambda x: distance(x, a))
            adjusted_next_point = pushPoint(
                forbidArea_center[i], push_distance, next_point
            )
            interplot_points.append(adjusted_next_point)
            distance_init = distance(a, adjusted_next_point)
            # clockwise loop & counterclockwise loop
            interplot_points_cw, interplot_points_ccw = (
                interplot_points.copy(),
                interplot_points.copy(),
            )
            next_point_cw, next_point_ccw = next_point, next_point
            adjusted_next_point_cw, adjusted_next_point_ccw = (
                adjusted_next_point,
                adjusted_next_point,
            )
            distance_cw, distance_ccw = distance_init, distance_init
            while islinePolygonIntersect(adjusted_next_point_cw, b, forbidArea):
                iteration_count += 1
                if iteration_count > max_iterations:
                    print("Max iterations reached1")
                    distance_cw = 1e20
                    break
                interplot_points_cw.append(adjusted_next_point_cw)
                next_point_cw = forbidArea[
                    (forbidArea.index(next_point_cw) + 1) % lenForbidArea
                ]
                distance_cw += distance(adjusted_next_point_cw, next_point_cw)
                adjusted_next_point_cw = pushPoint(
                    forbidArea_center[i], push_distance, next_point_cw
                )
            adjusted_next_point_cw = pushPoint(
                forbidArea_center[i], push_distance, next_point_cw
            )

            interplot_points_cw.append(adjusted_next_point_cw)
            iteration_count = 0

            while islinePolygonIntersect(adjusted_next_point_ccw, b, forbidArea):
                iteration_count += 1
                if iteration_count > max_iterations:
                    print("Max iterations reached2")
                    distance_ccw = 1e20
                    break
                interplot_points_ccw.append(adjusted_next_point_ccw)
                next_point_ccw = forbidArea[
                    (forbidArea.index(next_point_ccw) - 1) % lenForbidArea
                ]
                distance_ccw += distance(adjusted_next_point_ccw, next_point_ccw)
                adjusted_next_point_ccw = pushPoint(
                    forbidArea_center[i], push_distance, next_point_ccw
                )
            adjusted_next_point_ccw = pushPoint(
                forbidArea_center[i], push_distance, next_point_ccw
            )
            interplot_points_ccw.append(adjusted_next_point_ccw)
            if distance_cw < distance_ccw:
                interplot_points = interplot_points_cw
            else:
                interplot_points = interplot_points_ccw
            interplot_points.append(b)
            next_point_index = None
            for i in range(len(points)):
                if distance(points[i], b) < 1e-5:
                    next_point_index = i
                    break
            for i in range(next_point_index + 1, len(points)):
                interplot_points.append(points[i])
            # del points
            points = interplot_points

    return interplot_points


def pushPoint(forbidArea_center, push_distance, closest_vertex):
    vectorCenter = closest_vertex - forbidArea_center
    adjusted_next_point = closest_vertex + (
        push_distance * vectorCenter / np.linalg.norm(vectorCenter)
    )

    return adjusted_next_point
