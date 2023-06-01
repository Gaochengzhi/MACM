import numpy as np

def OptimalControlSolution_stage1(pos_a, v_a, pos_t, v_t, radius_t, p_GCAA, Agents, tf_t, tloiter_t, time_step, n_rounds, na, kdrag):
    X = np.zeros((4, na, n_rounds+1))
    A = np.zeros((4,4))
    A[0:2, 2:4] = np.eye(2)
    B = np.zeros((4,2))
    B[2:4, 0:2] = np.eye(2)
    completed_tasks = []
    pos_a_curr = None
    pos_t_curr = None

    for i in range(na): #逐个计算每个Agent的状态量
        v_a[i,:] = np.zeros(v_a[i,:].shape)
        X[:, i, 0] = np.concatenate([pos_a[i,:], v_a[i,:]])

        if not p_GCAA[i] or p_GCAA[i] == 0:
            p_GCAA[i] = []
            for k in range(n_rounds):
                if k == 0:
                    u = - kdrag * X[2:4, i, k]
                    X[:, i, k+1] = X[:, i, k] + time_step * (A @ X[:, i, k] + B @ u)
                else:
                    X[:, i, k+1] = X[:, i, k]

        pGCAA_len = len(p_GCAA[i])
        for j in range(pGCAA_len):
            k = 0
            ind_task = p_GCAA[i][j]
            tf = tf_t[ind_task]

            if j > 0:
                tf -= tf_t[p_GCAA[i][j-1]]

            pos_t_curr = pos_t[ind_task,:]
            v_t_curr = v_t[ind_task,:]
            pos_a_curr = X[0:2, i, k]
            v_a_curr = X[2:4, i, k]

            R = radius_t[ind_task]
            # norm_vt = 2*np.pi * R / tloiter_t[ind_task]
            # norm_a = norm_vt**2 / R

            t = 0

            pos_t_curr_app = pos_t_curr + R * np.array([np.cos(np.radians(15)), np.sin(np.radians(15))])

            while np.linalg.norm(pos_t_curr - X[0:2, i, k]) > 40: # 设置终点触发条件
                u = 0
                angle = pos_t_curr - X[0:2, i, k]

                r_target_circle = pos_t_curr - X[0:2, i, k]
                d = np.linalg.norm(r_target_circle)
                curr_velocity = np.zeros(X[:, i, k].shape)
                curr_velocity[0:2] = r_target_circle / d
                X[:, i, k+1] = X[:, i, k] + time_step * curr_velocity * 750

                t += time_step
                k += 1

        if pos_t_curr is None or pos_a_curr is None:
            pos_t_curr = np.array([0])
            pos_a_curr = np.array([0])

        if np.linalg.norm(pos_t_curr - pos_a_curr) <= 40:
            completed_tasks.append(p_GCAA[i])

        for k2 in range(k+1, n_rounds+1):
            X[:,i,k2] = X[:,i,k]

    return X, completed_tasks

