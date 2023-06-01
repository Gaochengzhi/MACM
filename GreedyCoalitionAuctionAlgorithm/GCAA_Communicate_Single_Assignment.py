def gcaa_communicate_single_assignment(GCAA_Params, GCAA_Data, Graph, old_t, T):
    N = GCAA_Params['N']

    # Initialize and Copy data
    old_z = [[GCAA_Data[n]['winners'] if k == n else 0 for k in range(N)] for n in range(N)]
    old_y = [[GCAA_Data[n]['winnerBids'] if k == n else 0 for k in range(N)] for n in range(N)]
    old_f = [[GCAA_Data[n]['fixedAgents'] if k == n else 0 for k in range(N)] for n in range(N)]

    z = old_z.copy()
    y = old_y.copy()
    f = old_f.copy()
    t = old_t.copy()

    epsilon = 10e-6

    # Start communication between agents
    for i in range(N):
        for k in range(N):
            if Graph[k][i] == 1:
                z[i][k] = old_z[k][k]
                y[i][k] = old_y[k][k]
                f[i][k] = old_f[k][k]
                # Update timestamps for all agents based on latest comm
                for n in range(N):
                    if n != i and t[i][n] < old_t[k][n]:
                        t[i][n] = old_t[k][n]
                t[i][k] = T

    # Copy data
    for n in range(N):
        GCAA_Data[n]['winners'] = z[n]
        GCAA_Data[n]['winnerBids'] = y[n]
        GCAA_Data[n]['fixedAgents'] = f[n]
        t[n][n] = T

    return GCAA_Data, t

