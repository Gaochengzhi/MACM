from GreedyCoalitionAuctionAlgorithm.GCAA_Init import GCAA_Init
from GreedyCoalitionAuctionAlgorithm.GCAA_Bundle import gcaa_bundle
from GreedyCoalitionAuctionAlgorithm.GCAA_Communicate_Single_Assignment import (
    gcaa_communicate_single_assignment,
)
import numpy as np


def GCAA_Main(agents, tasks, graph, prob_a_t, lambd):
    # Initialize GCAA parameters
    GCAA_Params = GCAA_Init(len(agents), len(tasks), prob_a_t, lambd)
    GCAA_Data = []
    for n in range(GCAA_Params["N"]):
        agent_data = {}
        agent_data["agentID"] = agents[n]["id"]
        agent_data["agentIndex"] = n
        agent_data["path"] = -1 * int(agents[n]["Lt"])
        agent_data["times"] = -1 * int(agents[n]["Lt"])
        agent_data["winners"] = [0] * GCAA_Params["N"]
        agent_data["winnerBids"] = [0] * GCAA_Params["N"]
        agent_data["fixedAgents"] = [0] * GCAA_Params["N"]
        agent_data["Lt"] = agents[n]["Lt"]
        GCAA_Data.append(agent_data)

    for i in range(GCAA_Params["N"]):
        task_idx = agents[i]["previous_task"]
        if (
            task_idx != 0
            and (tasks[task_idx]["tf"] - tasks[task_idx]["tloiter"])
            / tasks[task_idx]["tloiter"]
            < 1
        ):
            GCAA_Data[i]["fixedAgents"][i] = 1
            GCAA_Data[i]["path"] = [agents[i]["previous_task"]]
            GCAA_Data[i]["winners"][i] = task_idx
            GCAA_Data[i]["winnerBids"][i] = agents[i]["previous_winnerBids"]

    T = 0
    t = np.zeros((GCAA_Params["N"], GCAA_Params["N"]))
    lastTime = T - 1
    doneFlag = 0

    while doneFlag == 0:
        GCAA_Data, t = gcaa_communicate_single_assignment(
            GCAA_Params, GCAA_Data, graph, t, T
        )
        for n in range(GCAA_Params["N"]):
            if GCAA_Data[n]["fixedAgents"][n] == 0:
                GCAA_Data[n], newBid, agents[n] = gcaa_bundle(
                    GCAA_Params, GCAA_Data[n], agents[n], tasks, n
                )

        doneFlag = 1
        for n in range(GCAA_Params["N"]):
            if GCAA_Data[n]["fixedAgents"][n] == 0:
                doneFlag = 0
                break

        if T - lastTime > GCAA_Params["N"]:
            doneFlag = 1
        elif T - lastTime > 2 * GCAA_Params["N"]:
            print("Algorithm did not converge due to communication trouble")
            doneFlag = 1
        else:
            T += 1

    Total_Score = 0
    All_scores = [0] * GCAA_Params["N"]
    for n in range(GCAA_Params["N"]):
        All_scores[n] = GCAA_Data[n]["winnerBids"][n]
        Total_Score += All_scores[n]

    return GCAA_Data, Total_Score, All_scores, agents
