function plotResults(ref, xEst, prInnovations, prInnovationCovariances, ...
    dopInnovations, dopInnovationCovariances, utcSecondsHist)
%PLOTRESULTS Summary of this function goes here
%   Detailed explanation goes here


% Initializations
idxStatePos = PVTUtils.getStateIndex(PVTUtils.ID_POS);
idxStateClkBias = PVTUtils.getStateIndex(PVTUtils.ID_CLK_BIAS);
idxStateVel = PVTUtils.getStateIndex(PVTUtils.ID_VEL);
timelineSec = (utcSecondsHist - utcSecondsHist(1));

% Estimated position in geodetic
[posLat, posLon, posAlt] = ecef2geodetic(wgs84Ellipsoid, ...
    xEst(idxStatePos(1), :)', ...
    xEst(idxStatePos(2), :)', ...
    xEst(idxStatePos(3), :)');

% Interpolate groundtruth at the computed position's time
refInterpLla(:, 1) = interp1(ref.utcSeconds, ref.posLla(:, 1), utcSecondsHist);
refInterpLla(:, 2) = interp1(ref.utcSeconds, ref.posLla(:, 2), utcSecondsHist);
refInterpLla(:, 3) = interp1(ref.utcSeconds, ref.posLla(:, 3), utcSecondsHist);
assert(size(refInterpLla, 1) == size(xEst, 2), 'Reference and computed position vectors are not the same size');

hError = Lla2Hd(refInterpLla, [posLat, posLon, posAlt]);
vError = refInterpLla(:, 3) - posAlt;

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

%% CDFs
pctl = 95;
% Horizontal
hErrPctl = prctile(abs(hError),pctl);
[hErrF,hEerrX] = ecdf(abs(hError));

figure; hold on;
plot(hEerrX,hErrF,'LineWidth',2)
plot([1;1]*hErrPctl, [0;1]*pctl/100, '--k')
legend('CDF',sprintf('%d%% bound = %.2f', pctl, hErrPctl));
xlabel('Horizontal error (m)'); ylabel('Frequency')
title([Config.CAMPAIGN_NAME ' - ' Config.PHONE_NAME], 'Interpreter', 'none');
 
% Vertical
vErrPctl = prctile(abs(vError),pctl);
[vErrF,vErrX] = ecdf(abs(vError));

figure; hold on;
plot(vErrX,vErrF,'LineWidth',2)
plot([1;1]*vErrPctl, [0;1]*pctl/100, '--k')
legend('CDF',sprintf('%d%% bound = %.2f', pctl, vErrPctl));
xlabel('Vertical error error (m)'); ylabel('Frequency')
title([Config.CAMPAIGN_NAME ' - ' Config.PHONE_NAME], 'Interpreter', 'none');
end

