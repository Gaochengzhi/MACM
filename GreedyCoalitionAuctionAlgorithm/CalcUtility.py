from ComputeCommandParamsWithVelocity import ComputeCommandParamsWithVelocity
import numpy as np
import numpy as np
from scipy.spatial.distance import pdist
from math import sin, cos, pi


def compute_dU(
    allocations,
    j,
    agent_pos,
    agent_va,
    task_pos,
    task_v,
    task_type,
    task_radius,
    task_tloiter,
    task_tf,
    task_value,
    i,
    prob_a_t,
    N,
    lambda_,
    old_tf,
    kdrag,
):
    j = j - 1
    prod_others = np.prod(
        1
        - np.array(allocations)[list(range(0, i)) + list(range(i + 1, N)), j]
        * np.array(prob_a_t)[list(range(0, i)) + list(range(i + 1, N)), j]
    )

    r_without_a = task_value[j] * (1 - prod_others)
    r_with_a = task_value[j] * (1 - prod_others * (1 - prob_a_t[i, j]))

    tf = task_tf[j] - old_tf
    old_tf = old_tf + tf
    if task_type[j] == 0:
        # ComputeCommandParamsWithVelocity needs to be translated
        # Assuming it returns 5 values
        _, _, _, _, rho = ComputeCommandParamsWithVelocity(
            agent_pos, agent_va, task_pos[j][:], task_v[j][:], tf, [], kdrag
        )

        rin = task_pos[j][:]
        vt = task_v[j][:]
    else:
        rin, vt, rho = MinimumCostAlongLoitering(
            agent_pos, agent_va, task_pos, task_radius, task_tloiter, task_tf, j, kdrag
        )

    dU = (r_with_a - r_without_a) - lambda_ * rho
    return rin, vt, dU


def calc_utility(
    agent_pos,
    agent_va,
    task_pos,
    task_v,
    task_type,
    task_radius,
    task_tloiter,
    task_tf,
    task_value,
    b,
    i,
    prob_a_t,
    N,
    winners,
    lambda_,
    kdrag,
):
    U = 0
    old_tf = 0

    # No assignment if same end time for two tasks in the sequence
    # print(task_tf[b])
    # if len(set(task_tf[b])) != len(task_tf[b]):
    #     return None, None, None

    rin, vt, dU = compute_dU(
        winners,
        b,
        agent_pos,
        agent_va,
        task_pos,
        task_v,
        task_type,
        task_radius,
        task_tloiter,
        task_tf,
        task_value,
        i,
        prob_a_t,
        N,
        lambda_,
        old_tf,
        kdrag,
    )
    U = U + dU

    return rin, vt, U


def MinimumCostAlongLoitering(
    agent_pos, agent_va, task_pos, task_radius, task_tloiter, task_tf, j, kdrag
):
    norm_vt = 2 * pi * task_radius[j] / task_tloiter[j]
    rho = 1e16

    for theta in np.linspace(0.05, 2 * pi * 0.9, 10):
        rin_new = task_pos[j][:] + task_radius[j] * np.array([cos(theta), sin(theta)])
        for rot_turn in [-1, 1]:
            vt_new = (
                rot_turn
                * np.array([[0, 1], [-1, 0]])
                @ (rin_new - task_pos[j][:])
                * norm_vt
                / pdist([rin_new, task_pos[j][:]], "euclidean")
            )
            # ComputeCommandParamsWithVelocity needs to be translated
            # Assuming it returns 5 values
            _, _, _, _, rho_new = ComputeCommandParamsWithVelocity(
                agent_pos,
                agent_va,
                rin_new,
                vt_new,
                task_tf[j] - task_tloiter[j],
                [],
                kdrag,
            )
            if rho_new < rho:
                rho = rho_new
                rin = rin_new
                vt = vt_new
                # u_opt = u

    norm_a = np.linalg.norm(vt) ** 2 / task_radius[j]
    rho = rho + 1 / 2 * (norm_a) ** 2 * task_tloiter[j]
    return rin, vt, rho
