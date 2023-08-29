import numpy as np
from scipy.optimize import fmin_slsqp
from fillBetween import isInsideConvex, islinePolygonIntersect


def desiredVelocity(agent):
    agentGoal = None
    if len(agent["targetsWayPoints"]) != 0:
        agentGoal = agent["targetsWayPoints"][0]["targetPath"][0]
    elif len(agent["goals"]) > 0:
        agentGoal = agent["goals"][0]
    velocity = np.array(agentGoal) - agent["position"]
    return velocity


def desiredVelocityCost(agent, control, forbidens):
    dv_cost = np.sum((desiredVelocity(agent) - control) ** 2)
    c_value = 0
    for area in forbidens:
        futurePos = agent["position"] + control
        if isInsideConvex(futurePos, area) or islinePolygonIntersect(
            agent["position"], agent["position"] + control, area
        ):
            c_value -= 500

    return dv_cost + c_value


def get_constraints(agent, obstacles, control, dt, forbidens):
    c = []
    marginNumber = 100
    for obs in obstacles:
        c_value = np.linalg.norm(
            np.array(agent["position"]) + control - np.array(obs["position"])
        ) + max(agent["certainRadiusLimit"], obs["certainRadiusLimit"])
        #

        c.append(c_value)
    # for area in forbidens:
    #     futurePos = agent["position"] + control
    #     if isInsideConvex(futurePos, area) or islinePolygonIntersect(
    #         agent["position"], agent["position"] + control, area
    #     ):
    #         c_value = -5000
    #         c.append(c_value)
    #     else:
    #         c_value = 1
    #         c.append(c_value)

    return np.array(c)


def clip_controls(controls, maxSpeed):
    original_norm = np.sqrt(controls[0] ** 2 + controls[1] ** 2)
    if original_norm <= maxSpeed:
        return controls
    scaling_factor = maxSpeed / original_norm
    clipped_controls = [controls[0] * scaling_factor, controls[1] * scaling_factor]
    return clipped_controls


def getControls(agent, obstacles, dt, forbidens):
    def cost(u):
        return desiredVelocityCost(agent, u, forbidens)

    maxSpeed = None
    maxdetectSpeed = agent["sailViteLimit"]
    maxCatachSpeed = agent["certainRadiusLimit"]
    if agent["type"] == 0:
        maxSpeed = maxdetectSpeed * 100
    else:
        maxSpeed = maxCatachSpeed
    # Initial guess
    init = desiredVelocity(agent)
    # lb = [-maxSpeed, -maxSpeed]
    # ub = [maxSpeed, maxSpeed]

    lb = [-300, -300]
    ub = [300, 300]
    # Bounds

    # Optimization using fmin_slsqp
    controls = fmin_slsqp(
        func=cost,
        x0=init,
        bounds=list(zip(lb, ub)),
        f_ieqcons=lambda u: get_constraints(agent, obstacles, u, dt, forbidens),
        iter=100,
        acc=1e-3,
        disp=False,
    )

    # clip the controls making controls's length is less than maxSpeed
    controls = clip_controls(controls, maxSpeed)
    return controls
