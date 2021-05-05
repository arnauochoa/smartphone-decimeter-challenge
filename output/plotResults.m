function plotResults(xEst, ref)
%PLOTRESULTS Summary of this function goes here
%   Detailed explanation goes here


% Initializations
idxStatePos = PVTUtils.getStateIndex(PVTUtils.ID_POS);
idxStateVel = PVTUtils.getStateIndex(PVTUtils.ID_VEL);

[posLat, posLon, posAlt] = ecef2geodetic(wgs84Ellipsoid, ...
    xEst(idxStatePos(1), :)', ...
    xEst(idxStatePos(2), :)', ...
    xEst(idxStatePos(3), :)');

figure;
geoplot(ref.posLla(:, 1), ref.posLla(:, 2), '.-', posLat, posLon, '.-');
legend('Groundtruth', 'Computed');

end

