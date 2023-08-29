import numpy as np


def ecef2lla(position, model="WGS84"):
    """
    Convert ECEF coordinates to geodetic coordinates (latitude, longitude).

    Parameters:
        position (numpy.ndarray): Array of ECEF coordinates (x, y).
        model (str): Ellipsoid model. Default is 'WGS84'.

    Returns:
        numpy.ndarray: Array of geodetic coordinates (latitude, longitude).
    """
    if model == "WGS84":
        a = 6378137.0  # Semi-major axis (equatorial radius) in meters
        f = 1 / 298.257223563  # Flattening
    else:
        a = 0.0
        f = 0.0

    x, y = position[:, 0], position[:, 1]

    lon = np.arctan2(y, x)

    p = np.sqrt(x**2 + y**2)
    e = np.sqrt((a**2 - (a * f) ** 2) / a**2)
    theta = np.arctan2(a * y, p)

    lat = np.arctan2(a * 1000000, p * (1 - e**2))

    lat = np.degrees(lat)
    lon = np.degrees(lon)

    return np.column_stack((lat, lon))


def lla2ecef(lla, model="WGS84"):
    """
    Convert geodetic coordinates (latitude, longitude, altitude) to ECEF coordinates.

    Parameters:
        lla (numpy.ndarray): Array of geodetic coordinates (latitude, longitude, altitude).
        model (str): Ellipsoid model. Default is 'WGS84'.

    Returns:
        numpy.ndarray: Array of ECEF coordinates.
    """
    lla = np.hstack(
        (
            lla[:, 1].reshape(-1, 1),
            lla[:, 0].reshape(-1, 1),
            np.zeros((lla.shape[0], 1)),
        )
    )
    if model == "WGS84":
        a = 6378137.0  # Semi-major axis (equatorial radius) in meters
        f = 1 / 298.257223563  # Flattening
    else:
        a = 0.0
        f = 0.0

    lat, lon, alt = np.radians(lla[:, 0]), np.radians(lla[:, 1]), lla[:, 2]

    b = (1 - f) * a
    e = np.sqrt((a**2 - b**2) / a**2)
    N = a / np.sqrt(1 - e**2 * np.sin(lat) ** 2)

    x = (N + alt) * np.cos(lat) * np.cos(lon)
    y = (N + alt) * np.cos(lat) * np.sin(lon)
    z = (b**2 / a**2 * N + alt) * np.sin(lat)
    return np.column_stack((x, y, z))[:, :2]
