import random


def addTargets(id, position, velocity, restcheck=4):
    # "lon": position[0],
    # "lat": position[1],
    obs = {
        "id": id + "sdas",
        "position": position,
        "velocity": [0, 0],
        "angle": sorted([random.randint(0, 180), random.randint(0, 180)]),
        "restCheck": random.randint(3, 5),
        "restTime": random.randint(90, 120),
        "allocated": False,
        "finished": False,
        # !!! change the name of threaten rangess
        "certainRadiusLimit": random.randint(50, 80),
        "certainRadius": random.randint(120, 150),
        "forbidenAgents": [],
    }
    return obs
