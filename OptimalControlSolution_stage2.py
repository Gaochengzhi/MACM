import numpy as np


def remove_consecutive_zeros(tmp):
    tmp = tmp[:, ~np.all(tmp == 0.0, axis=0)]
    return tmp


def OptimalControlSolution_stage2(Agents, Tasks, p_GCAA, time_step, n_rounds, kdrag):
    pos_a = Agents["Pos"]
    v_a = Agents["v_a"]
    pos_t = Tasks["Pos"][:, 0:2]
    v_t = Tasks["Speed"]
    radius_t = Tasks["radius"]
    tf_t = Tasks["tf"]
    tloiter_t = Tasks["tloiter"]
    na = Agents["N"]
    targets_angle = Tasks["angle"]
    targets_restCheck = Tasks["restCheck"]
    flag_changeTargets = Tasks["flag_changeTargets"]

    pos_t_curr = []
    pos_a_curr = []
    Xfinal = [None] * na
    X = np.zeros((4, na, n_rounds + 1))
    J = np.zeros(na)
    J_to_completion_target = np.zeros(na)
    A = np.block(
        [[np.zeros((2, 2)), np.eye(2, 2)], [np.zeros((2, 2)), np.zeros((2, 2))]]
    )
    B = np.block([np.zeros((2, 2)), np.eye(2, 2)]).T
    completed_tasks = []
    for i in range(na):
        tmp = 0
        v_a[i, :] = np.zeros_like(v_a[i, :])
        X[:, i, 0] = np.concatenate([pos_a[i, :], v_a[i, :]], axis=0)
        if not p_GCAA[i] or p_GCAA[i] == 0:
            p_GCAA[i] = []
            for k in range(X.shape[2] - 2):
                if k == 0:
                    u = -kdrag * X[2:4, i, k]
                    X[:, i, k + 1] = X[:, i, k] + time_step * (
                        np.dot(A, X[:, i, k]) + np.dot(B, u)
                    )
                else:
                    X[:, i, k + 1] = X[:, i, k]
            continue

        for j in range(1):
            k = 0
            ind_task = p_GCAA[i] - 1
            tf = tf_t[ind_task]
            if j > 0:
                tf = tf - tf_t[p_GCAA[i] - 1]
            pos_t_curr = pos_t[ind_task, :]
            v_t_curr = v_t[ind_task, :]
            pos_a_curr = X[0:2, i, k]
            v_a_curr = X[2:4, i, k]

            J_to_completion_target_curr = 0

            J_to_completion_target[i] = J_to_completion_target_curr
            R = radius_t[ind_task]
            norm_vt = 2 * np.pi * R / tloiter_t[ind_task]
            norm_a = norm_vt**2 / R
            if ind_task >= len(targets_angle):
                exp_angle = targets_angle[ind_task - 1]
            else:
                exp_angle = targets_angle[ind_task]

            if tloiter_t[ind_task] > 0 and tf > 0:
                J_to_completion_target[i] += (
                    0.5 * norm_a**2 * min(tloiter_t[ind_task], tf)
                )

            t = 0

            pos_t_curr_app = np.array(
                [
                    pos_t_curr[0] + R * np.cos(np.radians(exp_angle)),
                    pos_t_curr[1] + R * np.sin(np.radians(exp_angle)),
                ]
            )

            while np.linalg.norm(pos_t_curr - X[0:2, i, k]) > 53:
                u = 0
                angle = pos_t_curr - X[0:2, i, k]
                if (
                    np.linalg.norm(pos_t_curr - X[0:2, i, k]) < R + 3
                    and abs(
                        np.degrees(np.arctan2(angle[1], angle[0])) - (exp_angle - 180)
                    )
                    < 5
                ):
                    r_target_circle = pos_t_curr - X[0:2, i, k]
                    d = np.linalg.norm(r_target_circle)
                    curr_velocity = np.zeros_like(X[:, i, k])
                    curr_velocity[0:2] = r_target_circle / d
                    X[:, i, k + 1] = X[:, i, k] + time_step * curr_velocity * 500
                elif np.linalg.norm(pos_t_curr_app - X[0:2, i, k]) > 3:
                    r_target_circle = pos_t_curr_app - X[0:2, i, k]
                    d = np.linalg.norm(r_target_circle)
                    curr_velocity = np.zeros_like(X[:, i, k])
                    curr_velocity[0:2] = r_target_circle / d
                    X[:, i, k + 1] = X[:, i, k] + time_step * curr_velocity * 500
                    tmp = k

                t += time_step
                k += 1

                if k == 0:
                    J[i] = 0.5 * np.linalg.norm(u) ** 2 * time_step
        if len(pos_a_curr) == 0 or len(pos_t_curr) == 0:
            pos_t_curr = 0
            pos_a_curr = 0

        if np.linalg.norm(pos_t_curr - pos_a_curr) <= 53:
            completed_tasks.append(p_GCAA[i])
        if flag_changeTargets[ind_task] <= 1 and targets_restCheck[ind_task] <= 1:
            tmp = np.copy(X[:, :, i])
            # tmp = tmp[:, ~np.all(tmp == 0.0, axis=0)]
            trimmed_tmp = remove_consecutive_zeros(tmp)
            Xfinal[i] = trimmed_tmp
        elif flag_changeTargets[ind_task] > 1 and targets_restCheck[ind_task] <= 1:
            lengTh = k - tmp + 1
            Xtmp = X[:, i, tmp:k]
            Xtmp = np.expand_dims(Xtmp, axis=1)
            Xtmp1 = np.flip(Xtmp[:, :, : lengTh - 1], axis=2)
            X[:, i, k : k + 1 + lengTh - 2] = Xtmp1[:, 0, :]
            tmp = np.copy(X[:, :, i])
            tmp = tmp[:, ~np.all(tmp == 0.0, axis=0)]
            trimmed_tmp = remove_consecutive_zeros(tmp)
            Xfinal[i] = trimmed_tmp
        elif flag_changeTargets[ind_task] <= 1 and targets_restCheck[ind_task] > 1:
            lengTh = k - tmp + 1
            Xtmp = X[:, i, tmp - 1 : k]
            Xtmp = np.expand_dims(Xtmp, axis=1)
            Xtmp1 = np.flip(Xtmp[:, :, :lengTh], axis=2)
            Xtmp2 = np.concatenate((Xtmp1, Xtmp[:, :, 1:lengTh]), axis=2)
            Xtmp3 = np.tile(Xtmp2, (1, 1, targets_restCheck[ind_task] - 1))
            Xtmp3 = np.squeeze(Xtmp3)
            X[
                :,
                i,
                k + 1 : k + 1 + (targets_restCheck[ind_task] - 1) * (2 * lengTh - 1),
            ] = Xtmp3
            tmp = np.copy(X[:, i, :])
            # tmp = tmp[:, ~np.all(tmp == 0.0, axis=0)]
            trimmed_tmp = remove_consecutive_zeros(tmp)
            Xfinal[i] = trimmed_tmp
        elif flag_changeTargets[ind_task] > 1 and targets_restCheck[ind_task] > 1:
            lengTh = k - tmp + 1
            Xtmp = X[:, i, tmp - 1 : k]
            Xtmp = np.expand_dims(Xtmp, axis=1)
            Xtmp1 = np.flip(Xtmp[:, :, : lengTh - 1], axis=2)
            Xtmp2 = np.concatenate((Xtmp1, Xtmp[:, :, 1:lengTh]), axis=2)
            Xtmp3 = np.tile(Xtmp2, (1, 1, targets_restCheck[ind_task] - 1))
            Xtmp4 = np.concatenate((Xtmp3, Xtmp1), axis=2)
            Xtmp4 = np.squeeze(Xtmp4)

            X[
                :,
                i,
                k
                + 1 : k
                + 1
                + (targets_restCheck[ind_task] - 1) * (2 * lengTh - 2)
                + lengTh
                - 1,
            ] = Xtmp4

            tmp = np.copy(X[:, :, i])
            trimmed_tmp = remove_consecutive_zeros(tmp)
            Xfinal[i] = trimmed_tmp

    return Xfinal, completed_tasks, J, J_to_completion_target
