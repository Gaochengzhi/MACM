import numpy as np


def gcaa_bundle_remove_single_assignment(GCAA_Params, GCAA_Data, agent_idx):
    if sum(GCAA_Data["winnerBids"]) == 0:
        return GCAA_Data

    if GCAA_Data["winners"][agent_idx] > 0:
        All_winners = [
            i == GCAA_Data["winners"][agent_idx] and j == 0
            for i, j in zip(GCAA_Data["winners"], GCAA_Data["fixedAgents"])
        ]

        if sum(All_winners) > 0:
            All_winnerBids = [
                i if cond else -1e16
                for i, cond in zip(GCAA_Data["winnerBids"], All_winners)
            ]
            maxBid, idxMaxBid = max(All_winnerBids), All_winnerBids.index(
                max(All_winnerBids)
            )

            All_losers = All_winners.copy()
            All_losers[idxMaxBid] = 0

            GCAA_Data["winners"] = [
                i if not cond else 0
                for i, cond in zip(GCAA_Data["winners"], All_losers)
            ]
            GCAA_Data["winnerBids"] = [
                i if not cond else 0
                for i, cond in zip(GCAA_Data["winnerBids"], All_losers)
            ]
            GCAA_Data["fixedAgents"][idxMaxBid] = 1

    return GCAA_Data
