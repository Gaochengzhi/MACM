from GreedyCoalitionAuctionAlgorithm.GCAA_Init import GCAA_Init
from GreedyCoalitionAuctionAlgorithm.GCAA_Bundle import gcaa_bundle
from GreedyCoalitionAuctionAlgorithm.GCAA_Communicate_Single_Assignment import (
    gcaa_communicate_single_assignment,
)


def GCAA_Main(agents, tasks, graph, prob_a_t, lambd):
    # Initialize GCAA parameters
    gcaa_params = GCAA_Init(len(agents), len(tasks), prob_a_t, lambd)
    # already

    gcaa_data = []
    winners = [0] * gcaa_params["N"]
    winnerBids = [0] * gcaa_params["N"]
    fixedAgents = [0] * gcaa_params["N"]
    for i, agent in enumerate(agents):
        gcaa_data.append(
            {
                "agentID": agent["id"],
                "agentIndex": i,
                "path": -1 * agent["Lt"],
                "times": -1 * agent["Lt"],
                "winners": winners,
                "winnerBids": winnerBids,
                "fixedAgents": fixedAgents,
                "Lt": agent["Lt"],
            }
        )

    # Fix the tasks if the completion is close
    for i, agent in enumerate(agents):
        task_idx = agent["previous_task"]
        if (
            task_idx != 0
            and (tasks[task_idx]["tf"] - tasks[task_idx]["tloiter"])
            / tasks[task_idx]["tloiter"]
            < 1
        ):
            gcaa_data[i]["fixedAgents"][i] = 1
            gcaa_data[i]["path"] = agent["previous_task"]
            gcaa_data[i]["winners"][i] = task_idx
            gcaa_data[i]["winnerBids"][i] = agent["previous_winnerBids"]

    T = 0
    t = [[0] * gcaa_params["N"] for _ in range(gcaa_params["N"])]
    lastTime = T - 1
    doneFlag = 0

    # Main GCAA loop (runs until convergence)

    while doneFlag == 0:
        # Communicate
        gcaa_data, t = gcaa_communicate_single_assignment(
            gcaa_params, gcaa_data, graph, t, T
        )

        # Run GCAA bundle building/updating
        for n in range(gcaa_params["N"]):
            if gcaa_data[n]["fixedAgents"][n] == 0:
                gcaa_data[n], newBid, agents[n] = gcaa_bundle(
                    gcaa_params, gcaa_data[n], agents[n], tasks, n
                )

        doneFlag = 1
        for n in range(gcaa_params["N"]):
            if gcaa_data[n]["fixedAgents"][n] == 0:
                doneFlag = 0
                break

        if T - lastTime > gcaa_params["N"]:
            doneFlag = 1
        elif T - lastTime > 2 * gcaa_params["N"]:
            print("Algorithm did not converge due to communication trouble")
            doneFlag = 1
        else:
            T += 1

    # Compute the total score of the GCAA assignment

    total_score = 0
    all_scores = []
    for n in range(gcaa_params["N"]):
        all_scores.append(gcaa_data[n]["winnerBids"][n])
        total_score += all_scores[n]

    return gcaa_data, total_score, all_scores, agents
