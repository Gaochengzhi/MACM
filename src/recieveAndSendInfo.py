from inSensorRange import inSensorRange


def getNewTarget(agent_i, targetSearched, fakeTargets):
    """
    only return the target that newly added not not finished, check previews added target to prevent overadd
    """
    targetFoundByAgent_i = []
    for fake_target in fakeTargets:
        if inSensorRange(agent_i, fake_target["position"]) and fake_target[
            "id"
        ] not in [target["id"] for target in targetSearched]:
            targetFoundByAgent_i.append(fake_target)
    return targetFoundByAgent_i
