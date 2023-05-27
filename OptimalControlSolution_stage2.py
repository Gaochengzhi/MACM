import numpy as np

def OptimalControlSolution_stage2(Agents, Tasks, p_GCAA, time_step, n_rounds, kdrag):
    pos_a = Agents.Pos
    v_a = Agents.v_a
    pos_t = Tasks.Pos[:, :2]
    v_t = Tasks.Speed
    radius_t = Tasks.radius
    tf_t = Tasks.tf
    tloiter_t = Tasks.tloiter
    na = Agents.N
    targets_angle = Tasks.angle
    targets_restCheck = Tasks.restCheck
    flag_changeTargets = Tasks.flag_changeTargets

    pos_t_curr = []
    pos_a_curr = []
    Xfinal = [None] * na
    X = np.zeros((4, na, n_rounds + 1))
    J = np.zeros((1, na))
    J_to_completion_target = np.zeros((1, na))
    A = np.block([[np.zeros((2, 2)), np.eye(2, 2)], [np.zeros((2, 2)), np.zeros((2, 2))]])
    B = np.block([[np.zeros((2, 2))], [np.eye(2, 2)]])

    completed_tasks = []

    for i in range(na):
        v_a[i, :] = np.zeros(v_a[i, :].shape)
        X[:, i, 0] = np.concatenate((pos_a[i, :], v_a[i, :]), axis=None)

        if (len(p_GCAA[i]) == 0) or (p_GCAA[i] == 0):
            p_GCAA[i] = []
            for k in range(n_rounds):
                if k == 0:
                    u = -kdrag * X[2:4, i, k]
                    X[:, i, k + 1] = X[:, i, k] + time_step * (A @ X[:, i, k] + B @ u)
                else:
                    X[:, i, k + 1] = X[:, i, k]
            continue

        for j in range(len(p_GCAA[i])):
            k = 0
            ind_task = p_GCAA[i][j]
            tf = tf_t[ind_task]
            if j > 0:
                tf = tf - tf_t[p_GCAA[i][j - 1]]
            pos_t_curr = pos_t[ind_task, :].reshape((-1, 1))
            v_t_curr = v_t[ind_task, :].reshape((-1, 1))
            pos_a_curr = X[0:2, i, k + 1].reshape((-1, 1))
            v_a_curr = X[2:4, i, k + 1].reshape((-1, 1))

            J_to_completion_target_curr = 0
            J_to_completion_target[i] = J_to_completion_target_curr
            R = radius_t[ind_task]
            norm_vt = 2 * np.pi * R / tloiter_t[ind_task]
            norm_a = norm_vt ** 2 / R
            exp_angle = targets_angle[ind_task]

            if tloiter_t[ind_task] > 0 and tf > 0:
                J_to_completion_target[i] = J_to_completion_target[i] + 1 / 2 * norm_a ** 2 * min(tloiter_t[ind_task], tf)

            t = 0

            pos_t_curr_app = np.zeros((2, 1))
            pos_t_curr_app[0, 0] = pos_t_curr[0, 0] + R * np.cos(np.deg2rad(exp_angle))
            pos_t_curr_app[1, 0] = pos_t_curr[1, 0] + R * np.sin(np.deg2rad(exp_angle))

            while np.linalg.norm(pos_t_curr - X[0:2, i, k + 1]) > 53:
                u = 0
                angle = pos_t_curr - X[0:2, i, k + 1]
                if np.linalg.norm(pos_t_curr - X[0:2, i, k + 1]) < R + 3 and abs(np.rad2deg(np.arctan2(angle[1, 0], angle[0, 0])) - (exp_angle - 180)) < 5:
                    r_target_circle = pos_t_curr - X[0:2, i, k + 1]
                    d = np.linalg.norm(r_target_circle)
                    curr_velocity = np.zeros(X[:, i, k + 1].shape)
                    curr_velocity[0:2] = r_target_circle / d
                    X[:, i, k + 2] = X[:, i, k + 1] + time_step * curr_velocity * 500
                elif np.linalg.norm(pos_t_curr_app - X[0:2, i, k + 1]) > 3:
                    r_target_circle = pos_t_curr_app - X[0:2, i, k + 1]
                    d = np.linalg.norm(r_target_circle)
                    curr_velocity = np.zeros(X[:, i, k + 1].shape)
                    curr_velocity[0:2] = r_target_circle / d
                    X[:, i, k + 2] = X[:, i, k + 1] + time_step * curr_velocity * 500

                t = t + time_step
                k = k + 1

                if k == 0:
                    J[0, i] = 1 / 2 * np.linalg.norm(u) ** 2 * time_step

            if np.linalg.norm(pos_t_curr - pos_a_curr) <= 53:
                completed_tasks.append(p_GCAA[i])

            if flag_changeTargets[ind_task] <= 1 and targets_restCheck[ind_task] <= 1:
                tmp = X[:, i, :]
                tmp = np.delete(tmp, np.where(~tmp.any(axis=0))[0], axis=1)
                Xfinal[i] = tmp
            elif flag_changeTargets[ind_task] > 1 and targets_restCheck[ind_task] <= 1:
                len = k + 1 - tmp + 1
                Xtmp = X[:, i, tmp:k + 1]
                Xtmp1 = np.flip(Xtmp[:, :, :-1], axis=2)
                X[:, i, k + 2:k + 2 + len - 2] = Xtmp1

                tmp = X[:, i, :]
                tmp = np.delete(tmp, np.where(~tmp.any(axis=0))[0], axis=1)
                Xfinal[i] = tmp
            elif flag_changeTargets[ind_task] <= 1 and targets_restCheck[ind_task] > 1:
                len = k + 1 - tmp + 1
                Xtmp = X[:, i, tmp:k + 1]
                Xtmp1 = np.flip(Xtmp[:, :, :-1], axis=2)
                Xtmp2 = np.concatenate((Xtmp1, Xtmp[:, :, 1:]), axis=2)
                Xtmp3 = np.tile(Xtmp2, (1, 1, targets_restCheck[ind_task] - 1))
                X[:, i, k + 2:k + 1 + (targets_restCheck[ind_task] - 1) * (2 * len - 2)] = Xtmp3

                tmp = X[:, i, :]
                tmp = np.delete(tmp, np.where(~tmp.any(axis=0))[0], axis=1)
                Xfinal[i] = tmp
            elif flag_changeTargets[ind_task] > 1 and targets_restCheck[ind_task] > 1:
                len = k + 1 - tmp + 1
                Xtmp = X[:, i, tmp:k + 1]
                Xtmp1 = np.flip(Xtmp[:, :, :-1], axis=2)
                Xtmp2 = np.concatenate((Xtmp1, Xtmp[:, :, 1:]), axis=2)
                Xtmp3 = np.tile(Xtmp2, (1, 1, targets_restCheck[ind_task] - 1))
                X[:, i, k + 2:k + 1 + (targets_restCheck[ind_task] - 1) * (2 * len - 2) + len - 1] = np.concatenate((Xtmp3, Xtmp1), axis=2)

                tmp = X[:, i, :]
                tmp = np.delete(tmp, np.where(~tmp.any(axis=0))[0], axis=1)
                Xfinal[i] = tmp

    return Xfinal, completed_tasks, J, J_to_completion_target

