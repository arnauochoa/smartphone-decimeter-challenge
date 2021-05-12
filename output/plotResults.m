function plotResults(ref, xEst, prInnovations, prInnovationCovariances, ...
    dopInnovations, dopInnovationCovariances, utcSecondsHist, sigmaHist, prRejectedHist, dopRejectedHist)
%PLOTRESULTS Summary of this function goes here
%   Detailed explanation goes here
close all;

% Initializations
idxStatePos = PVTUtils.getStateIndex(PVTUtils.ID_POS);
idxStateVel = PVTUtils.getStateIndex(PVTUtils.ID_VEL);
idxStateClkBias = PVTUtils.getStateIndex(PVTUtils.ID_CLK_BIAS);
idxStateClkDrift = PVTUtils.getStateIndex(PVTUtils.ID_CLK_DRIFT);
idxStateIFBias = PVTUtils.getStateIndex(PVTUtils.ID_INTER_FREQ_BIAS);
idxStateISBias = PVTUtils.getStateIndex(PVTUtils.ID_INTER_SYS_BIAS);
timelineSec = (utcSecondsHist - utcSecondsHist(1));

% Estimated position in geodetic
[posLat, posLon, posAlt] = ecef2geodetic(wgs84Ellipsoid, ...
    xEst(idxStatePos(1), :)', ...
    xEst(idxStatePos(2), :)', ...
    xEst(idxStatePos(3), :)');

% Interpolate groundtruth at the computed position's time
refInterpLla = interp1(ref.utcSeconds, ref.posLla, utcSecondsHist);
assert(size(refInterpLla, 1) == size(xEst, 2), 'Reference and computed position vectors are not the same size');

nedError = Lla2Ned(refInterpLla, [posLat, posLon, posAlt]);
hError = Lla2Hd(refInterpLla, [posLat, posLon, posAlt]);

% Groundtruth velocity
dtRef = diff(ref.tow);
[xRef, yRef, zRef] = geodetic2ecef(wgs84Ellipsoid, ref.posLla(:, 1), ref.posLla(:, 2), ref.posLla(:, 3));
refEcef = [xRef, yRef, zRef];
refVelEcef = diff(refEcef) ./ dtRef;
refVelTime = ref.utcSeconds(1:end-1) + dtRef/2;
refVelEcefInterp = interp1(refVelTime, refVelEcef, utcSecondsHist);

velErr = refVelEcefInterp - xEst(idxStateVel, :)';

%% State plots
figs = [];
figs = [figs figure];
if isprop(Config, 'OBS_RINEX_REF_XYZ') % Use observations from rinex
    geoplot(ref.posLla(1, 1), ref.posLla(1, 2), 'x', posLat, posLon, '.', 'LineWidth', 1);
else
    geoplot(ref.posLla(:, 1), ref.posLla(:, 2), '.-', posLat, posLon, '.-');
end
geobasemap none
legend('Groundtruth', 'Computed');
figureWindowTitle(figs(end), 'Map');

figs = [figs figure];
plot(timelineSec, nedError)
xlabel('Time since start (s)'); ylabel('Position error (m)');
legend('N', 'E', 'D')
title('Groundtruth - Estimation')
grid on
figureWindowTitle(figs(end), 'Position error');

figs = [figs figure];
plot(timelineSec(1:end-1), xEst(idxStateVel, 1:end-1))
xlabel('Time since start (s)'); ylabel('Velocity (m/s)');
legend('X', 'Y', 'Z');
figureWindowTitle(figs(end), 'Velocity');

figs = [figs figure];
plot(timelineSec, velErr)
xlabel('Time since start (s)'); ylabel('Velocity error (m)');
legend('X', 'Y', 'Z');
title('Groundtruth - Estimation')
grid on
figureWindowTitle(figs(end), 'Velocity error');

figs = [figs figure];
subplot(2,1,1)
plot(timelineSec, xEst(idxStateClkBias, :))
xlabel('Time since start (s)'); ylabel('RX clock bias (m)');
subplot(2,1,2)
plot(timelineSec, sigmaHist(idxStateClkBias, :))
xlabel('Time since start (s)'); ylabel('STD of RX clock bias (m)');
figureWindowTitle(figs(end), 'RX clock bias');

figs = [figs figure];
subplot(2,1,1);
plot(timelineSec, xEst(idxStateClkDrift, :))
xlabel('Time since start (s)'); ylabel('RX clock drift (m/s)');
subplot(2,1,2)
plot(timelineSec, sigmaHist(idxStateClkDrift, :))
xlabel('Time since start (s)'); ylabel('STD of RX clock drift (m/s)');
figureWindowTitle(figs(end), 'RX clock drift');

if ~isempty(idxStateIFBias)
    figs = [figs figure];
    subplot(2,1,1);
    plot(timelineSec, xEst(idxStateIFBias, :))
    xlabel('Time since start (s)'); ylabel('Inter-frequency bias (m)');
    subplot(2,1,2)
    plot(timelineSec, sigmaHist(idxStateIFBias, :))
    xlabel('Time since start (s)'); ylabel('STD of I-F bias (m)');
    figureWindowTitle(figs(end), 'I-F bias');
end

if ~isempty(idxStateISBias)
    figs = [figs figure];
    subplot(2,1,1);
    plot(timelineSec, xEst(idxStateISBias, :))
    xlabel('Time since start (s)'); ylabel('Inter-system bias (m)');
    subplot(2,1,2)
    plot(timelineSec, sigmaHist(idxStateISBias, :))
    xlabel('Time since start (s)'); ylabel('STD of I-S bias (m)');
    figureWindowTitle(figs(end), 'I-S bias');
end


%% Innovations
figs = [figs figure];
subplot(2,1,1)
plot(prInnovations', '.')
xlabel('Time since start (s)'); ylabel('Pseudorange innovations (m)');
subplot(2,1,2)
plot(prInnovationCovariances', '.')
xlabel('Time since start (s)'); ylabel('Pseudorange innovation covariances (m²)');
figureWindowTitle(figs(end), 'Code innovations');


figs = [figs figure];
subplot(2,1,1)
plot(timelineSec, dopInnovations', '.')
xlabel('Time since start (s)'); ylabel('Doppler innovations (m/s)');
subplot(2,1,2)
plot(timelineSec, dopInnovationCovariances', '.')
xlabel('Time since start (s)'); ylabel('Doppler innovation covariances (m²/s²)');
figureWindowTitle(figs(end), 'Doppler innovations');

%% # of rejected
figs = [figs figure]; subplot(2,1,1);
plot(timelineSec, prRejectedHist)
xlabel('Time since start (s)'); ylabel('# rejected Code obs');
subplot(2,1,2);
plot(timelineSec, dopRejectedHist)
xlabel('Time since start (s)'); ylabel('# rejected Doppler obs');
figureWindowTitle(figs(end), 'Outlier rejections');

%% CDFs
pctl = 95;
% Horizontal
hErrPctl = prctile(abs(hError),pctl);
[hErrF,hEerrX] = ecdf(abs(hError));

figs = [figs figure]; hold on;
plot(hEerrX,hErrF,'LineWidth',2)
plot([1;1]*hErrPctl, [0;1]*pctl/100, '--k')
legend('CDF',sprintf('%d%% bound = %.2f', pctl, hErrPctl));
xlabel('Horizontal error (m)'); ylabel('Frequency')
title([Config.CAMPAIGN_NAME ' - ' Config.PHONE_NAME], 'Interpreter', 'none');
figureWindowTitle(figs(end), 'Hor. pos. CDF');
 
% Vertical
vErrPctl = prctile(abs(nedError(:,3)),pctl);
[vErrF,vErrX] = ecdf(abs(nedError(:,3)));

figs = [figs figure]; hold on;
plot(vErrX,vErrF,'LineWidth',2)
plot([1;1]*vErrPctl, [0;1]*pctl/100, '--k')
legend('CDF',sprintf('%d%% bound = %.2f', pctl, vErrPctl));
xlabel('Vertical error error (m)'); ylabel('Frequency')
title([Config.CAMPAIGN_NAME ' - ' Config.PHONE_NAME], 'Interpreter', 'none');
figureWindowTitle(figs(end), 'Ver. pos. CDF');

% Velocity
velErrPctl = prctile(abs(velErr),pctl);
for iDim = 1:3
    [velErrF(:, iDim), velErrX(:, iDim)] = ecdf(abs(velErr(:, iDim)));
end

figs = [figs figure]; hold on;
colors = [0 0 1; 0 1 0; 1 0 0];
for iDim = 1:3
    plot(velErrX(:, iDim),velErrF(:, iDim),'LineWidth',2, 'Color', colors(iDim, :))
    plot([1;1]*velErrPctl(iDim), [0;1]*pctl/100, '--', 'Color', colors(iDim, :))
end
legend({'X',sprintf('%d%% bound = %.2f', pctl, velErrPctl(1)), ...
        'Y',sprintf('%d%% bound = %.2f', pctl, velErrPctl(2)), ...
        'Z',sprintf('%d%% bound = %.2f', pctl, velErrPctl(3))}, ...
        'Location','northeastoutside');
xlabel('Velocity error error (m/s)'); ylabel('Frequency')
title([Config.CAMPAIGN_NAME ' - ' Config.PHONE_NAME], 'Interpreter', 'none');
figureWindowTitle(figs(end), 'Velocity CDF');
end

