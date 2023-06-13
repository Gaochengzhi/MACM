import numpy as np

# from pyproj import Proj, transform
from pyproj import CRS, Transformer

# def ecef2lla(ecef_, model="WSG84"):
#     # define two coordinate systems
#     ecef = Proj(proj='geocent', ellps='WGS84', datum='WGS84')
#     lla = Proj(proj='latlong', ellps='WGS84', datum='WGS84')
#     x,y,z  = ecef_

#     lon, lat, alt = transform(ecef, lla, x, y, z, radians=False)

#     return np.column_stack((lon,lat,alt))


def ecef2lla(pos, model="WGS84"):
    # Define the coordinate systems
    wgs84 = CRS("EPSG:4326")  # WGS84 geographic coordinate system
    ecef = CRS("EPSG:4978")  # ECEF coordinate system

    # Define the transformation
    transformer = Transformer.from_crs(ecef, wgs84, always_xy=True)

    # Preallocate output array
    lla = np.zeros_like(pos)

    # Convert each position
    for i in range(pos.shape[0]):
        lon, lat, alt = transformer.transform(*pos[i, :])
        lla[i, :] = [lat, lon, alt]

    return lla
