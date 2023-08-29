import numpy as np
from scipy.spatial.distance import euclidean
from fillBetween import fillbetween


def generate_points_on_circumference(center, radius, num_points, angle_range):
    angles = np.linspace(
        np.radians(angle_range[0]), np.radians(angle_range[1]), num_points
    )
    points = np.array(
        [
            [center[0] + radius * np.cos(angle), center[1] + radius * np.sin(angle)]
            for angle in angles
        ]
    )
    return points


def calculate_path_length(points):
    return np.sum([euclidean(points[i], points[i + 1]) for i in range(len(points) - 1)])


def updateCatchWayPoint(
    agent, targets, forbiddenAreas, taskArea, arrive_distance, centralPoints
):
    # agent["targetsWayPoints"] = []

    for target in targets:
        center = target["position"]
        radius = target["certainRadius"] - arrive_distance
        angle_range = target["angle"]
        num_points = 6

        points = generate_points_on_circumference(
            center, radius, num_points, angle_range
        )

        best_path = None
        min_path_length = float("inf")

        for i in range(num_points):
            candidate_points = np.roll(
                points, -i, axis=0
            )  # Rotate the points to find the optimal starting point
            path_length = calculate_path_length(candidate_points)

            if path_length < min_path_length:
                min_path_length = path_length
                best_path = candidate_points

        combined_array = best_path
        interplotLineStart = fillbetween(
            agent["position"],
            best_path[0],
            forbiddenAreas,
            taskArea,
            centralPoints,
            # arrive_distance,
        )
        interplotLineEnd = []

        if len(agent["goals"]) != 0:
            interplotLineEnd = fillbetween(
                best_path[-1],
                agent["position"],
                forbiddenAreas,
                taskArea,
                centralPoints,
                # arrive_distance,
            )

        if len(interplotLineStart) > 0 or len(interplotLineEnd) > 0:
            if len(interplotLineStart) > 0 and len(interplotLineEnd) > 0:
                combined_array = np.concatenate(
                    (
                        interplotLineStart,
                        best_path,
                        interplotLineEnd,
                    ),
                    axis=0,
                )
            elif len(interplotLineStart) > 0:
                combined_array = np.concatenate(
                    (
                        interplotLineStart,
                        best_path,
                    ),
                    axis=0,
                )
            else:
                combined_array = np.concatenate(
                    (
                        best_path,
                        interplotLineEnd,
                    ),
                    axis=0,
                )
        else:
            combined_array = best_path
        agent["targetsWayPoints"].insert(
            0, {"id": target["id"], "targetPath": combined_array}
        )
