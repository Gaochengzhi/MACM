import numpy as np


def gcaa_communicate_single_assignment(GCAA_Params, GCAA_Data, Graph, old_t, T):
    # Copy data
    old_z = [d["winners"] for d in GCAA_Data]
    old_y = [d["winnerBids"] for d in GCAA_Data]
    old_f = [d["fixedAgents"] for d in GCAA_Data]

    z = old_z.copy()
    y = old_y.copy()
    f = old_f.copy()
    t = old_t.copy()

    epsilon = 10e-6

    # Start communication between agents
    for i in range(GCAA_Params["N"]):
        for k in range(GCAA_Params["N"]):
            if Graph[k][i] == 1:
                z[i][k] = old_z[k][k]
                y[i][k] = old_y[k][k]
                f[i][k] = old_f[k][k]

                # Update timestamps for all agents based on latest comm
                for n in range(GCAA_Params["N"]):
                    if n != i and t[i][n] < old_t[k][n]:
                        t[i][n] = old_t[k][n]

                t[i][k] = T

    # Copy data
    for n in range(GCAA_Params["N"]):
        GCAA_Data[n]["winners"] = z[n]
        GCAA_Data[n]["winnerBids"] = y[n]
        GCAA_Data[n]["fixedAgents"] = f[n]
        t[n][n] = T

    return GCAA_Data, t
