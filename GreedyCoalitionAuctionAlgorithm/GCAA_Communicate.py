def gcaa_communicate(gcaa_params, gcaa_data, graph, old_t, T, agent_idx):
    old_z = []
    old_y = []
    for n in range(gcaa_params["N"]):
        old_z.append(gcaa_data[n]["winners"])
        old_y.append(gcaa_data[n]["winnerBids"])

    z = old_z.copy()
    y = old_y.copy()
    t = old_t.copy()

    epsilon = 10e-6

    i = agent_idx
    for k in range(gcaa_params["N"]):
        if graph[k][i] == True:
            for j in range(gcaa_params["M"]):
                z[k][i] = old_z[k][k]

            for n in range(gcaa_params["N"]):
                if n != i and t[i][n] < old_t[k][n]:
                    t[i][n] = old_t[k][n]
            t[i][k] = T

    for n in range(gcaa_params["N"]):
        gcaa_data[n]["winners"] = z[n]
        t[n][n] = T

    return gcaa_data, t
