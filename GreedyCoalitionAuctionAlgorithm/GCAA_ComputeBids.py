def gcaa_compute_bids(gcaa_params, gcaa_data, agent, tasks, feasibility):
    L = [idx for idx, val in enumerate(gcaa_data['path']) if val == -1]
    if len(L) == 0:
        return gcaa_data, [], [], feasibility
    
    gcaa_data['bids'] = [0] * gcaa_params['M']
    best_idxs = [0] * gcaa_params['M']
    task_times = [0] * gcaa_params['M']

    for m in range(1, gcaa_params['M'] + 1):
        if gcaa_params['CM'][agent['type']][tasks[m]['type']] > 0:
            if m not in gcaa_data['path'][0:L[0]]:
                best_bid = 0
                best_index = 0
                best_time = -1

                for j in range(1, L[0] + 1):
                    if feasibility[m][j] == 1:
                        skip = 0

                        if j == 1:
                            task_prev = None
                            time_prev = None
                        else:
                            task_prev = tasks[gcaa_data['path'][j-1]]
                            time_prev = gcaa_data['times'][j-1]

                        if j == L[0]:
                            task_next = None
                            time_next = None
                        else:
                            task_next = tasks[gcaa_data['path'][j]]
                            time_next = gcaa_data['times'][j]

                        score, min_start, max_start = scoring_calc_score(gcaa_params, agent, tasks[m], task_prev, time_prev, task_next, time_next)

                        if min_start > max_start:
                            skip = 1
                            feasibility[m][j] = 0

                        if not skip:
                            if score > best_bid:
                                best_bid = score
                                best_index = j
                                best_time = min_start

                if best_bid > 0:
                    gcaa_data['bids'][m-1] = best_bid
                    best_idxs[m-1] = best_index
                    task_times[m-1] = best_time

    return gcaa_data, best_idxs, task_times, feasibility

