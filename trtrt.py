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
    e = np.sqrt((a**2 - (a * f)**2) / a**2)
    theta = np.arctan2(a * y, p)

    lat = np.arctan2(a * 1000000, p * (1 - e**2))
    
    lat = np.degrees(lat)
    lon = np.degrees(lon)
    
    return np.column_stack((lat, lon))

# Example usage
ecef_position = np.array([[5000000.0, 5000000.0], [6000000.0, 6000000.0]])
geodetic_coords = ecef2lla(ecef_position)
print(geodetic_coords)

