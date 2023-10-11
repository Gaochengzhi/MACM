def transform_coordinates_for_plot(points, dx, dy, reverse=True):
    if reverse:
        dx = -dx
        dy = -dy
    points[:, 0] += dx
    points[:, 1] += dy
    return points


import numpy as np
import pyproj


def lla2ecef(lla):
    transformer = pyproj.Transformer.from_crs(4326, 3857)
    x, y = transformer.transform(lla[:, 1], lla[:, 0])
    return np.column_stack([x, y])


def ecef2lla(merc_coords):
    transformer = pyproj.Transformer.from_crs(4326, 3857)
    lon, lat = transformer.transform(
        merc_coords[:, 0],
        merc_coords[:, 1],
        direction=pyproj.enums.TransformDirection.INVERSE,
    )
    return np.column_stack([lat, lon])  # Constants for WGS84 model


# merc_proj = pyproj.Transformer.from_crs(4326, 3857)
# ## test
# targets_pos_value = np.array(
#     [[121.66111478389999, 38.805913105872833]],
#     # [121.71429605931525, 38.816378517234796],
#     # [121.66970109072057, 38.852307347659718],
#     # [121.68604831104761, 38.845092037001393],
# )
# targetPostion = transform_coordinates_for_plot(lla2ecef(targets_pos_value), 1, 1)
# targetPostions = transform_coordinates_for_plot(lla2ecef(targets_pos_value), 1, 1)
# orignal = ecef2lla(targetPostion)[0]
# # print(orignal)
# atest = np.array([[121.66111478389999, 38.805913105872833]])

# a = ecef2lla(transform_coordinates_for_plot(atest, 1, 1))
# print(a)
