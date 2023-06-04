import numpy as np


def gcaa_bundle_remove_single_assignment(GCAA_Params, GCAA_Data, agent_idx):
    if np.sum(GCAA_Data["winnerBids"]) == 0:
        return GCAA_Data

    if GCAA_Data["winners"][agent_idx] > 0:
        All_winners = (GCAA_Data["winners"] == GCAA_Data["winners"][agent_idx]) & (
            GCAA_Data["fixedAgents"] == 0
        )

        if np.any(All_winners):
            All_winnerBids = GCAA_Data["winnerBids"].copy()
            All_winnerBids[~All_winners] = -1e16
            idxMaxBid = np.argmax(All_winnerBids)
            GCAA_Data["winners"][All_winners] = 0
            GCAA_Data["winnerBids"][All_winners] = 0
            GCAA_Data["winners"][idxMaxBid] = GCAA_Data["winners"][agent_idx]
            GCAA_Data["winnerBids"][idxMaxBid] = GCAA_Data["winnerBids"][agent_idx]
            GCAA_Data["fixedAgents"][idxMaxBid] = 1

    return GCAA_Data
