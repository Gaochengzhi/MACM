import numpy as np
def lla2ecef(lla, model='WGS84'):
    """
    Convert geodetic coordinates (latitude, longitude, altitude) to ECEF coordinates.
    
    Parameters:
        lla (numpy.ndarray): Array of geodetic coordinates (latitude, longitude, altitude).
        model (str): Ellipsoid model. Default is 'WGS84'.
    
    Returns:
        numpy.ndarray: Array of ECEF coordinates.
    """
    if model == 'WGS84':
        a = 6378137.0  # Semi-major axis (equatorial radius) in meters
        f = 1 / 298.257223563  # Flattening
    else:
        # Custom ellipsoid parameters
        # Update these values based on your custom ellipsoid model
        a = 0.0
        f = 0.0
    
    # Convert latitude, longitude, altitude to radians
    lat, lon, alt = np.radians(lla[:, 0]), np.radians(lla[:, 1]), lla[:, 2]
    
    # Calculate auxiliary values
    b = (1 - f) * a
    e = np.sqrt((a**2 - b**2) / a**2)
    N = a / np.sqrt(1 - e**2 * np.sin(lat)**2)
    
    # Calculate ECEF coordinates
    x = (N + alt) * np.cos(lat) * np.cos(lon)
    y = (N + alt) * np.cos(lat) * np.sin(lon)
    z = (b**2 / a**2 * N + alt) * np.sin(lat)
    
    return np.column_stack((x, y, z))

# lla = np.array([[37,-122,0]])  # Single set of geodetic coordinates
# ecef = lla2ecef(lla)
# print(ecef)

