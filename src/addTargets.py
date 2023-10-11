import random


def addTargets(
    id,
    lon,
    lat,
    # angle,
    # restcheck,
    # resttime,
    # certainradiuslimit,
    # certainradius,
):
    # "lon": position[0],
    # "lat": position[1],
    restChecks = random.randint(3, 5)
    obs = {
        "id": id + "sdas",
        "position": lon,
        "velocity": [0, 0],
        "angle": sorted([random.randint(0, 180), random.randint(0, 180)]),
        "restCheck": restChecks,
        "restTime": random.randint(90, 120),
        "allocated": False,
        "allocatedTime": restChecks,
        "finished": False,
        # !!! change the name of threaten rangess
        "certainRadiusLimit": random.randint(50, 80),
        "certainRadius": 150,
        "forbidenAgents": [],
    }
    return obs
