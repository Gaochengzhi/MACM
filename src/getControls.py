import numpy as np
from scipy.optimize import fmin_slsqp
from fillBetween import isInsideConvex, islinePolygonIntersect
import sys


def desiredVelocity(agent, agentGoal):
    velocity = np.array(agentGoal) - agent["position"]
    return velocity


def getControls(agent, obstacles, dt, forbidens, taskArea, agentGoal):
    maxSpeed = None
    maxdetectSpeed = agent["sailViteLimit"]
    maxCatachSpeed = agent["certainRadiusLimit"]
    if agent["type"] == 0:
        maxSpeed = maxdetectSpeed * 5
    else:
        maxSpeed = maxCatachSpeed

    # Initial guess
    def desiredVelocityCost(agent, control, forbidens, taskArea, agentGoal, maxSpeed):
        realControl = clip_controls(control, maxSpeed)
        c_value = 0
        dv_cost = np.sum((desiredVelocity(agent, agentGoal) - control) ** 2)
        for area in forbidens:
            futurePos = agent["position"] + realControl
            if islinePolygonIntersect(
                agent["position"], futurePos, area
            ) or isInsideConvex(futurePos, area):
                # print("insid")
                c_value += 9e21
            else:
                c_value = 0.0

        futurePos = agent["position"] + control
        if isInsideConvex(futurePos, taskArea):
            c_value += 9e21
        return dv_cost + c_value

    def cost(u):
        return desiredVelocityCost(agent, u, forbidens, taskArea, agentGoal, maxSpeed)

    init = desiredVelocity(agent, agentGoal)
    lb = [-maxSpeed, -maxSpeed]
    ub = [maxSpeed, maxSpeed]

    # lb = [-9000, -9000]
    # ub = [9000, 9000]
    # Bounds

    def get_constraints(agent, obstacles, control, dt, forbidens, maxSpeed):
        c = []
        realControl = clip_controls(control, maxSpeed)
        # realControl = control
        marginNumber = 32
        # for obs in obstacles:
        #     c_value = np.linalg.norm(
        #         agent["position"] + realControl - obs["position"]
        #     ) - (
        #         max(agent["certainRadiusLimit"], obs["certainRadiusLimit"])
        #         + marginNumber
        #     )
        #     c.append(c_value)

        for area in forbidens:
            futurePos = agent["position"] + realControl
            if islinePolygonIntersect(
                agent["position"], futurePos, area
            ) or isInsideConvex(futurePos, area):
                c_value = -210.0
            else:
                c_value = 3.0
            c.append(c_value)
        return c

    def clip_controls(controls, maxSpeed):
        original_norm = np.sqrt(controls[0] ** 2 + controls[1] ** 2)
        if original_norm <= maxSpeed:
            return controls
        scaling_factor = maxSpeed / original_norm
        clipped_controls = [controls[0] * scaling_factor, controls[1] * scaling_factor]
        return clipped_controls

    # Optimization using fmin_slsqp
    controls = fmin_slsqp(
        func=cost,
        x0=init,
        bounds=list(zip(lb, ub)),
        f_ieqcons=lambda u: get_constraints(
            agent, obstacles, u, dt, forbidens, maxSpeed
        ),
        iter=50,
        acc=1,
        disp=False,
    )

    # clip the controls making controls's length is less than maxSpeed
    controls = clip_controls(controls, maxSpeed)
    return controls
