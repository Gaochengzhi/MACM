def gcaa_bundle_remove_single_assignment(gcaa_params, gcaa_data, agent_idx):
    if sum(gcaa_data['winnerBids']) == 0:
        return gcaa_data

    if gcaa_data['winners'][agent_idx - 1] > 0:
        all_winners = (gcaa_data['winners'] == gcaa_data['winners'][agent_idx - 1]) * (gcaa_data['fixedAgents'] == 0)
        if sum(all_winners) > 0:
            all_winnerBids = gcaa_data['winnerBids'] * all_winners
            all_winnerBids[all_winnerBids == 0] = -1e16
            maxBid = max(all_winnerBids)
            idxMaxBid = all_winnerBids.index(maxBid)
            all_losers = all_winners
            all_losers[idxMaxBid] = 0
            gcaa_data['winners'] = (~all_losers) * gcaa_data['winners']
            gcaa_data['winnerBids'] = (~all_losers) * gcaa_data['winnerBids']
            gcaa_data['fixedAgents'][idxMaxBid] = 1

    return gcaa_data

