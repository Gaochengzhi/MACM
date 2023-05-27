import math

def Scoring_CalcScore(GCAA_Params, agent, taskCurr, taskPrev, timePrev, taskNext, timeNext):
    if agent['type'] in [GCAA_Params['AGENT_TYPES']['QUAD'], GCAA_Params['AGENT_TYPES']['CAR']]:
        if taskPrev is None:  # First task in path
            # Compute start time of task
            dt = math.sqrt((agent['x'] - taskCurr['x'])**2 + (agent['y'] - taskCurr['y'])**2 + (agent['z'] - taskCurr['z'])**2) / agent['nom_vel']
            minStart = max(taskCurr['start'], agent['avail'] + dt)
        else:  # Not first task in path
            dt = math.sqrt((taskPrev['x'] - taskCurr['x'])**2 + (taskPrev['y'] - taskCurr['y'])**2 + (taskPrev['z'] - taskCurr['z'])**2) / agent['nom_vel']
            minStart = max(taskCurr['start'], timePrev + taskPrev['duration'] + dt)

        if taskNext is None:  # Last task in path
            maxStart = taskCurr['end']
        else:  # Not last task, check if we can still make promised task
            dt = math.sqrt((taskNext['x'] - taskCurr['x'])**2 + (taskNext['y'] - taskCurr['y'])**2 + (taskNext['z'] - taskCurr['z'])**2) / agent['nom_vel']
            maxStart = min(taskCurr['end'], timeNext - taskCurr['duration'] - dt)

        # Compute score
        reward = taskCurr['value'] * taskCurr['lambda']**(minStart - taskCurr['start'])

        # Subtract fuel cost. Implement constant fuel to ensure
        # that resulting scoring scheme satisfies a property called
        # diminishing marginal gain (DMG).
        # NOTE: This is a fake score since it double counts fuel. Should
        # not be used when comparing to optimal score. Need to compute
        # real score of GCAA paths once GCAA algorithm has finished running.
        # penalty = agent['fuel'] * math.sqrt((agent['x'] - taskCurr['x'])**2 + (agent['y'] - taskCurr['y'])**2 + (agent['z'] - taskCurr['z'])**2)
        penalty = 0

        score = reward - penalty

        # FOR USER TO DO: Define score function for specialized agents, for example:
        # elif agent['type'] == GCAA_Params['AGENT_TYPES']['NEW_AGENT'], ...

        # Need to define score, minStart, and maxStart
    else:
        print('Unknown agent type')

    return score, minStart, maxStart

