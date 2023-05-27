import numpy as np

def OptimalControlSolution_stage1(pos_a, v_a, pos_t, v_t, radius_t, p_GCAA, Agents, tf_t, tloiter_t, time_step, n_rounds, na, kdrag):
    pos_t_curr = []
    pos_a_curr = []
    X = np.zeros((4, na, n_rounds + 1))
    completed_tasks = []

    A = np.block([[np.zeros((2, 2)), np.eye(2, 2)], [np.zeros((2, 2)), np.zeros((2, 2))]])
    B = np.block([[np.zeros((2, 2))], [np.eye(2, 2)]])

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

            R = radius_t[ind_task]
            norm_vt = 2 * np.pi * R / tloiter_t[ind_task]
            norm_a = norm_vt ** 2 / R

            t = 0

            while np.linalg.norm(pos_t_curr - X[0:2, i, k + 1]) > 40:
                u = np.zeros((2, 1))
                angle = pos_t_curr - X[0:2, i, k + 1]

                r_target_circle = pos_t_curr - X[0:2, i, k + 1]
                d = np.linalg.norm(r_target_circle)
                curr_velocity = np.zeros((4, 1))
                curr_velocity[0:2, 0] = r_target_circle / d
                X[:, i, k + 2] = X[:, i, k + 1] + time_step * curr_velocity * 750

                t = t + time_step
                k = k + 1

            if np.linalg.norm(pos_t_curr - pos_a_curr) <= 40:
                completed_tasks.append(p_GCAA[i])

            for k2 in range(k + 2, n_rounds + 1):
                X[:, i, k2] = X[:, i, k + 1]

    return X, completed_tasks

