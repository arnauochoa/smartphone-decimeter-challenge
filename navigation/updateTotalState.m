function xTotal = updateTotalState(x, statPos)
idxStatePos = PVTUtils.getStateIndex(PVTUtils.ID_POS);
xTotal = x;
xTotal(idxStatePos) = xTotal(idxStatePos) + statPos;
end %end of function updateTotalState
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%