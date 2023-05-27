def gcaa_bundle_add(gcaa_params, gcaa_data, agent, tasks, agent_idx):
    if gcaa_data['fixedAgents'][agent_idx - 1] == 1:
        return gcaa_data, agent

    M = len(tasks)
    task_pos = np.zeros((M, 2))
    task_v = np.zeros((M, 2))
    task_tf = np.zeros(M)
    task_tloiter = np.zeros(M)
    task_radius = np.zeros(M)
    task_type = np.zeros(M)
    task_value = np.zeros(M)

    for j, task in enumerate(tasks):
        task_pos[j] = [task.x, task.y]
        task_v[j] = task.Speed
        task_tf[j] = task.tf
        task_tloiter[j] = task.tloiter
        task_radius[j] = task.radius
        task_type[j] = task.type
        task_value[j] = task.value

    U = -1e14
    b = []

    winners_matrix = np.zeros((gcaa_params['N'], gcaa_params['M']))
    for i, winner in enumerate(gcaa_data['winners']):
        if winner > 0:
            winners_matrix[i][winner - 1] = 1

    availTasks = []
    for j in range(1, gcaa_params['M'] + 1):
        if j not in gcaa_data['winners']:
            availTasks.append(j)

    if not availTasks:
        availTasks = list(range(1, M + 1))
        allTasksAssigned = True
        U = 0

    newRin = False
    for j in availTasks:
        if task_tf[j - 1] > task_tloiter[j - 1]:
            b_new = j

            winners_matrix[agent_idx - 1] = np.zeros(gcaa_params['M'])
            winners_matrix[agent_idx - 1][j - 1] = 1
            rin_t_new, vin_t_new, U_new = calc_utility(agent.pos, agent.v_a, task_pos, task_v, task_type, task_radius, task_tloiter, task_tf, task_value, b_new, agent_idx, gcaa_params['prob_a_t'], gcaa_params['N'], winners_matrix, gcaa_params['lambda'], agent.kdrag)

            if U_new > U:
                U = U_new
                b = b_new
                rin_t = rin_t_new
                vin_t = vin_t_new
                newRin = True

    gcaa_data['path'].append(b)
    gcaa_data['winnerBids'][agent_idx - 1] = U
    gcaa_data['winners'][agent_idx - 1] = b

    if not newRin:
        return gcaa_data, agent

    agent.rin_task = rin_t
    agent.vin_task = vin_t

    return gcaa_data, agent

