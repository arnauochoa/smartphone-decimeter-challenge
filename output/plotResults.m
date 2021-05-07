function plotResults(ref, xEst, prInnovations, prInnovationCovariances, ...
    dopInnovations, dopInnovationCovariances, refInnovations, refInnovationCovariances, utcSecondsHist)
%PLOTRESULTS Summary of this function goes here
%   Detailed explanation goes here


% Initializations
idxStatePos = PVTUtils.getStateIndex(PVTUtils.ID_POS);
idxStateClkBias = PVTUtils.getStateIndex(PVTUtils.ID_CLK_BIAS);
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

figure; plot(timelineSec(1:end-1), xEst(idxStateClkBias, 1:end-1))
xlabel('Time since start (s)'); ylabel('Receiver clock bias (m)');

figure; plot(timelineSec(1:end-1), xEst(idxStateVel, 1:end-1))
xlabel('Time since start (s)'); ylabel('Velocity (m/s)');
legend('X', 'Y', 'Z');

figure; plot(prInnovations', '.')
xlabel('Time since start (s)'); ylabel('Pseudorange innovations (m)');

figure; plot(prInnovationCovariances', '.')
xlabel('Time since start (s)'); ylabel('Pseudorange innovation covariances (m²)');

figure; plot(timelineSec, dopInnovations')
xlabel('Time since start (s)'); ylabel('Doppler innovations (m/s)');

figure; plot(timelineSec, dopInnovationCovariances')
xlabel('Time since start (s)'); ylabel('Doppler innovation covariances (m²/s²)');

% figure; plot(timelineSec(1:end-1), refInnovations')
% xlabel('Time since start (s)'); ylabel('Reference pos. innovations (m)');
% 
% 
% figure; plot(timelineSec(1:end-1), refInnovationCovariances')
% xlabel('Time since start (s)'); ylabel('Reference pos. innovation covariances (m²)');

end

