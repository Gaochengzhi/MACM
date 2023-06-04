from GreedyCoalitionAuctionAlgorithm.GCAA_BundleRemoveSingleAssignment import (
    gcaa_bundle_remove_single_assignment,
)
from GreedyCoalitionAuctionAlgorithm.GCAA_BundleAdd import gcaa_bundle_add


def gcaa_bundle(gcaa_params, gcaa_data, agent, tasks, agent_idx):
    # Update bundles after messaging to drop tasks that are outbid

    gcaa_data = gcaa_bundle_remove_single_assignment(gcaa_params, gcaa_data, agent_idx)
    
    # Bid on new tasks and add them to the bundle
    gcaa_data, agent = gcaa_bundle_add(gcaa_params, gcaa_data, agent, tasks, agent_idx)

    newBid = 0
    return gcaa_data, newBid, agent
