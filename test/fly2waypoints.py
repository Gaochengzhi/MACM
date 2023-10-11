import numpy as np


def polyarea(x, y, dim=1):
    # if len(x.shape) == 1:
    #     raise ValueError("Input arrays must have at least two dimensions.")

    if x.shape != y.shape:
        raise ValueError("Input arrays x and y must have the same shape.")

    if dim is None:
        dim = 0

    if dim < 0 or dim >= int(x.shape[0]):
        raise ValueError("Invalid value for dim.")

    perm = [dim] + [i for i in range(len(x.shape)) if i != dim]
    x = np.transpose(x, axes=perm)
    y = np.transpose(y, axes=perm)

    siz = x.shape
    if np.any(siz):
        area = np.abs(
            np.sum(
                (x[np.roll(np.arange(siz[0]), -1)], y[np.roll(np.arange(siz[0]), -1)])
                * (x, y),
                axis=0,
            )
            / 2
        ).reshape((1,) + siz[1:])
    else:
        area = np.sum(x)  # SUM produces the right value for all empty cases

    if dim != 0:
        area = np.transpose(area, axes=perm)

    return area


from shapely.geometry import Polygon, Point


def inforbid(p_forbidArea, point):
    forbid_polygon = Polygon(p_forbidArea)
    point_obj = Point(point)
    is_inside = point_obj.within(forbid_polygon)
    return is_inside


import numpy as np


def fly2waypoints(na, pos_a, waypoints, n_rounds):
    X = np.zeros((4, na, n_rounds + 1))
    xmax = 0
    organizedWaypoints = {}

    for i in range(na):
        wayP = waypoints[i]
        LineNo, _ = wayP.shape
        dis_left = np.linalg.norm(pos_a[i] - wayP[0, 0:2])
        dis_right = np.linalg.norm(pos_a[i] - wayP[0, 3:4])
        startfromLeft = (dis_left - dis_right) < 0

        tmp0 = wayP.copy()
        tmp1 = wayP[0::2, :]  # Odd rows
        tmp2 = wayP[1::2, :]  # Even rows
        tmp1[:, [0, 2]], tmp1[:, [1, 3]] = tmp1[:, [2, 0]], tmp1[:, [3, 1]]
        tmp2[:, [0, 2]], tmp2[:, [1, 3]] = tmp2[:, [2, 0]], tmp2[:, [3, 1]]

        if startfromLeft:
            tmp0[1::2, :] = tmp2
        else:
            tmp0[0::2, :] = tmp1

        reshapedTmp0 = tmp0.reshape(-1, 2)
        organizedWaypoints[i] = np.vstack([pos_a[i], tmp0.ravel().reshape(-1, 2)])
        oWp = organizedWaypoints[i]
        X[0:2, i, 0] = oWp[0, :]
        K = np.zeros(oWp.shape[0] - 1)

        # v = shape 4*4s
        # v = np.zeros((4, oWp.shape[0] - 1))
        # for j in range(len(oWp) - 2):
        #     dis = oWp[j + 1, :] - oWp[j, :]
        #     d = np.linalg.norm(dis)
        #     new_values = np.array(dis) / d * 20
        #     v[:, j] = np.concatenate([new_values, [0, 0]])
        #     K[j] = max(int(dis[0] / v[0, j]), int(dis[1] / v[1, j]))

        # for m in range(len(K)):
        #     iter0 = 0 if m == 0 else int(iter0 + K[m - 1])
        #     iterEnd = int(iter0 + K[m] - 1)

        #     for k in range(iter0, iterEnd):
        #         X[:, i, k + 1] = X[:, i, k] + v[:, m]

        # tmp = X[:, i, :]
        # tmp = tmp[:, ~np.all(tmp == 0, axis=0)]
        # organizedWaypoints[i] = tmp
        # # remove all negative number in organizedWaypoints
        # organizedWaypoints[i] = organizedWaypoints[i][
        #     :, np.all(organizedWaypoints[i] >= 0, axis=0)
        # ]

    # return organizedWaypoints, xmax
    return X, 0
