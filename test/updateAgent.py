def updateAgent(agents, pos_waypoint):
    for i in range(len(agents)):
        agents[i]['goal'] = pos_waypoint[i]
    return agents
