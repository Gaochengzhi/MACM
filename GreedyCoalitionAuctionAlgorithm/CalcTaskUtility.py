import numpy as np
from ComputeCommandParamsWithVelocity import ComputeCommandParamsWithVelocity


def calc_task_utility(
    agent_pos,
    agent_va,
    task_pos,
    task_vt,
    task_tf,
    task_value,
    j,
    prob_a_t,
    winners,
    lambda_val,
    kdrag,
):
    prod_all = np.prod(1 - winners[:, j] * prob_a_t[:, j])
    rt = task_value * (1 - prod_all)

    Rt = 0

    assigned_agents = np.where(winners[:, j] == 1)[0]
    if len(assigned_agents) == 0:
        U = 0
    else:
        for i in assigned_agents:
            _, _, _, _, rho = ComputeCommandParamsWithVelocity(
                agent_pos[i], agent_va[i], task_pos, task_vt, task_tf, None, kdrag
            )
            Rt += rho
        U = rt - lambda_val * Rt

    return U
