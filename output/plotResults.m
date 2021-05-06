function plotResults(ref, xEst, prInnovations, prInnovationCovariances, ...
    dopInnovations, dopInnovationCovariances, utcSecondsHist)
%PLOTRESULTS Summary of this function goes here
%   Detailed explanation goes here


% Initializations
idxStatePos = PVTUtils.getStateIndex(PVTUtils.ID_POS);
idxStateVel = PVTUtils.getStateIndex(PVTUtils.ID_VEL);
timelineSec = (utcSecondsHist - utcSecondsHist(1));

[posLat, posLon, posAlt] = ecef2geodetic(wgs84Ellipsoid, ...
    xEst(idxStatePos(1), :)', ...
    xEst(idxStatePos(2), :)', ...
    xEst(idxStatePos(3), :)');

figure;
geoplot(ref.posLla(:, 1), ref.posLla(:, 2), '.-', posLat, posLon, '.-');
geobasemap none
legend('Groundtruth', 'Computed');

figure; plot(timelineSec, prInnovations')
xlabel('Time since start (s)'); ylabel('Pseudorange innovations (m)');

figure; plot(timelineSec, prInnovationCovariances')
xlabel('Time since start (s)'); ylabel('Pseudorange innovation covariances (m²)');

figure; plot(timelineSec, dopInnovations')
xlabel('Time since start (s)'); ylabel('Doppler innovations (m/s)');

figure; plot(timelineSec, dopInnovationCovariances')
xlabel('Time since start (s)'); ylabel('Doppler innovation covariances (m²/s²)');

end

