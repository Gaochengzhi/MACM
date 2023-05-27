def legendUnq(h=None, sortType=None):
    import matlab
    from matlab import matlabroot
    import numpy as np
    
    def getLegendChildren(x):
        if useOldMethod:
            return matlab.graph2dhelper('get_legendable_children', x)
        else:
            return matlab.graphics.illustration.internal.getLegendableChildren(x)
    
    if h is None:
        h = matlab.gca()
    
    if isinstance(h, matlab.figure.Figure):
        h = np.flipud(h.findall('type', 'Axes'))
    
    useOldMethod = False if matlab.verLessThan('matlab', '9.5.0') else True
    
    legChildren = matlab.graphics.chart.primitive.Line()
    for i in range(len(h)):
        temp = getLegendChildren(h[i])
        if not temp.isempty():
            legChildren = matlab.concat([legChildren, temp], axis=0)
    legChildren.pop(0)
    
    dispNames = legChildren.get('DisplayName')
    if dispNames.isempty():
        dispNames = matlab.cell(0, 1)
    if not isinstance(dispNames, matlab.cell):
        dispNames = matlab.cell.from_array(dispNames)
    
    _, firstIdx = matlab.unique(dispNames, 'first')
    legRmIdx = matlab.ones(legChildren.size(), dtype=matlab.logical)
    legRmIdx[firstIdx] = matlab.logical(0)
    legRmIdx = legRmIdx.logical_or(matlab.cellfun(@isempty, dispNames))
    
    annot = legChildren.get('Annotation')
    for i in range(len(annot)):
        if legRmIdx[i]:
            legendInformation = annot[i].get('LegendInformation')
            legendInformation.set('IconDisplayStyle', 'off')
    
    unqLegHands = legChildren[~legRmIdx]
    
    if sortType is not None and len(unqLegHands) > 1:
        _, sortIdx = unqLegHands.get('DisplayName').sort()
        unqLegHands = unqLegHands[sortIdx]
    
    return unqLegHands

