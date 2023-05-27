def gcaa_communicate_single_assignment(gcaa_params, gcaa_data, graph, old_t, T):
    old_z = []
    old_y = []
    old_f = []
    for n in range(gcaa_params['N']):
        old_z.append(gcaa_data[n]['winners'])
        old_y.append(gcaa_data[n]['winnerBids'])
        old_f.append(gcaa_data[n]['fixedAgents'])
    
    z = old_z.copy()
    y = old_y.copy()
    f = old_f.copy()
    t = old_t.copy()

    epsilon = 10e-6

    for i in range(gcaa_params['N']):
        for k in range(gcaa_params['N']):
            if graph[k][i] == 1:
                z[i][k] = old_z[k][k]
                y[i][k] = old_y[k][k]
                f[i][k] = old_f[k][k]

                for n in range(gcaa_params['N']):
                    if n != i and t[i][n] < old_t[k][n]:
                        t[i][n] = old_t[k][n]
                t[i][k] = T

    for n in range(gcaa_params['N']):
        gcaa_data[n]['winners'] = z[n]
        gcaa_data[n]['winnerBids'] = y[n]
        gcaa_data[n]['fixedAgents'] = f[n]
        t[n][n] = T

    return gcaa_data, t

