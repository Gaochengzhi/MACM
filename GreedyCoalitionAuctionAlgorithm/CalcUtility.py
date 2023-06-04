from ComputeCommandParamsWithVelocity import ComputeCommandParamsWithVelocity
import numpy as np


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
    rin = None
    vt = None
    U = 0
    old_tf = 0

    for j in range(1):
        if j != 0 and task_tf[b[j]] == task_tf[b[j - 1]]:
            return

    j = b
    rin, vt, dU = compute_dU(
        winners,
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
    )
    U = U + dU

    return rin, vt, U


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
    rin = None
    vt = None

    prod_others = np.prod(
        1 - allocations[np.r_[0:i, i + 1 : N], j] * prob_a_t[np.r_[0:i, i + 1 : N], j]
    )
    r_without_a = task_value[j] * (1 - prod_others)
    r_with_a = task_value[j] * (1 - prod_others * (1 - prob_a_t[i, j]))

    tf = task_tf[j] - old_tf
    old_tf = old_tf + tf

    if task_type[j] == 0:
        # The function ComputeCommandParamsWithVelocity is not included in the provided code.
        # Please replace this with appropriate code.
        agent_pos = np.array(agent_pos)
        agent_va = np.array(agent_va)
        task_pos = np.array(task_pos)
        task_v = np.array(task_v)
        kdrag = np.array(kdrag)
        _, _, _, _, rho = ComputeCommandParamsWithVelocity(
            agent_pos.T, agent_va.T, task_pos[j, :].T, task_v[j, :].T, tf, None, kdrag
        )
        rin = task_pos[j, :].T
        vt = task_v[j, :].T
    else:
        rin, vt, rho = minimum_cost_along_loitering(
            agent_pos, agent_va, task_pos, task_radius, task_tloiter, task_tf, j, kdrag
        )

    dU = (r_with_a - r_without_a) - lambda_ * rho
    return rin, vt, dU


def minimum_cost_along_loitering(
    agent_pos, agent_va, task_pos, task_radius, task_tloiter, task_tf, j, kdrag
):
    norm_vt = 2 * np.pi * task_radius[j] / task_tloiter[j]
    rho = 1e16
    rin = None
    rho_new = None
    vt = None

    for theta in np.linspace(0.05, 2 * np.pi * 0.9, 10):
        rin_new = task_pos[j, :].T + task_radius[j] * np.array(
            [np.cos(theta), np.sin(theta)]
        )
        for rot_turn in [-1, 1]:
            vt_new = (
                rot_turn
                * np.array([[0, 1], [-1, 0]])
                @ (
                    (rin_new - task_pos[j, :].T)
                    * norm_vt
                    / np.linalg.norm(rin_new - task_pos[j, :].T)
                )
            )
            # The function ComputeCommandParamsWithVelocity is not included in the provided code.
            # P_e_s_ replace this with appropriate code.
            u, _, _, _, rho_new = ComputeCommandParamsWithVelocity(
                agent_pos.T,
                agent_va.T,
                rin_new,
                vt_new,
                task_tf[j] - task_tloiter[j],
                None,
                kdrag,
            )

            if rho_new < rho:
                rho = rho_new
                rin = rin_new
                vt = vt_new

    norm_a = np.linalg.norm(vt) ** 2 / task_radius[j]
    rho = rho + 0.5 * norm_a**2 * task_tloiter[j]
    return rin, vt, rho
