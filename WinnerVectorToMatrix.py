import numpy as np


def WinnerVectorToMatrix(N, M, winners):
    winners_matrix = np.zeros((N, M))
    for i in range(N):
        if winners[i] > 0:
            winners_matrix[i, int(winners[i])] = 1
    return winners_matrix
