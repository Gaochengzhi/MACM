def gcaa_bundle_remove(gcaa_params, gcaa_data):
    outbidForTask = False

    for j in range(gcaa_data['Lt']):
        if gcaa_data['bundle'][j] < 0:
            break
        else:
            if gcaa_data['winners'][gcaa_data['bundle'][j] - 1] != gcaa_data['agentIndex']:
                outbidForTask = True

            if outbidForTask:
                if gcaa_data['winners'][gcaa_data['bundle'][j] - 1] == gcaa_data['agentIndex']:
                    gcaa_data['winners'][gcaa_data['bundle'][j] - 1] = 0
                    gcaa_data['winnerBids'][gcaa_data['bundle'][j] - 1] = 0

                idx = gcaa_data['path'].index(gcaa_data['bundle'][j])

                gcaa_data['path'].pop(idx)
                gcaa_data['times'].pop(idx)
                gcaa_data['scores'].pop(idx)

                gcaa_data['bundle'][j] = -1

    return gcaa_data

