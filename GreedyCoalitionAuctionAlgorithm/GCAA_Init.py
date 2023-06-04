def GCAA_Init(N, M, prob_a_t, lambda_value):
    GCAA_Params = {}

    # Define GCAA Constants
    GCAA_Params["N"] = N  # number of agents
    GCAA_Params["M"] = M  # number of tasks
    GCAA_Params["prob_a_t"] = prob_a_t
    GCAA_Params["lambda"] = lambda_value
    GCAA_Params["MAX_STEPS"] = 10000000

    # Define agent types
    GCAA_Params["AGENT_TYPES"] = {"QUAD": 1, "CAR": 2}

    # Define task types
    GCAA_Params["TASK_TYPES"] = {"TRACK": 1, "RESCUE": 2}

    # Initialize Compatibility Matrix
    GCAA_Params["CM"] = [
        [0] * len(GCAA_Params["TASK_TYPES"])
        for _ in range(len(GCAA_Params["AGENT_TYPES"]))
    ]
    # Set agent-task pairs (which types of agents can do which types of tasks)
    GCAA_Params["CM"][GCAA_Params["AGENT_TYPES"]["QUAD"] - 1][
        GCAA_Params["TASK_TYPES"]["TRACK"] - 1
    ] = 1
    GCAA_Params["CM"][GCAA_Params["AGENT_TYPES"]["CAR"] - 1][
        GCAA_Params["TASK_TYPES"]["RESCUE"] - 1
    ] = 1

    return GCAA_Params
