function plotResults(ref, result)
%PLOTRESULTS Summary of this function goes here
%   Detailed explanation goes here
close all;

%% Initializations
config = Config.getInstance;
idxStatePos = PVTUtils.getStateIndex(PVTUtils.ID_POS);
idxStateVel = PVTUtils.getStateIndex(PVTUtils.ID_VEL);
idxStateClkDrift = PVTUtils.getStateIndex(PVTUtils.ID_CLK_DRIFT);
idxStateAllSdAmb = PVTUtils.getStateIndex(PVTUtils.ID_SD_AMBIGUITY);
timelineSec = (result.utcSeconds - result.utcSeconds(1));
nEpochs = size(result.xRTK, 2);
figures = [];
basemap = 'none';

%% RTK estimation
estPosXyz = result.xRTK(idxStatePos, :)';
estPosLla = ecef2geodeticVector(estPosXyz);
% Horizontal sigma as norm of North and East
stdHor = vecnorm(result.posStdNed(:, 1:2), 2, 2);

%% WLS estimation
estPosWLSLla = ecef2geodeticVector(result.xWLS(1:3, :)');

%% State plots
% Position
figures = [figures figure];
if strcmp(config.DATASET_TYPE, 'test')
    geoplot(estPosWLSLla(:, 1), estPosWLSLla(:, 2), '.b');
else
    if isprop(config, 'OBS_RINEX_REF_XYZ') % Use observations from rinex
        geoplot(ref.posLla(1, 1), ref.posLla(1, 2), 'xk', ...
            estPosWLSLla(:, 1), estPosWLSLla(:, 2), '.b', ...
            'LineWidth', 1);
    else
        geoplot(ref.posLla(:, 1), ref.posLla(:, 2), '.-k', ...
            estPosWLSLla(:, 1), estPosWLSLla(:, 2), '.b');
    end
end
geobasemap(basemap);
% Plot estimation with color depending on covariance
hold on; colormap summer;
geoscatter(estPosLla(:, 1), estPosLla(:, 2), 6, stdHor, 'filled');
c = colorbar; 
c.Label.String = 'Horizontal position STD (m)';
if strcmp(config.DATASET_TYPE, 'test'), legend('WLS', 'RTK');
else, legend('Groundtruth', 'WLS', 'RTK'); end
figureWindowTitle(figures(end), 'Map');

% Velocity
figures = [figures figure];
plot(timelineSec, result.velNed);
xlabel('Time since start (s)'); ylabel('Velocity (m/s)');
legend('North', 'East', 'Down');
figureWindowTitle(figures(end), 'Velocity');

if ~isempty(idxStateClkDrift)
    % Rx clock bias
    figures = [figures figure]; hold on;
    plot(timelineSec, result.xWLS(4, :))
    plot(timelineSec, cumsum(result.xRTK(idxStateClkDrift, :)))
    legend('WLS', 'RTK (clock drift)')
    xlabel('Time since start (s)'); ylabel('Clock bias (m)');
    figureWindowTitle(figures(end), 'Rx clock bias');
    
    % Rx clock drift
    figures = [figures figure]; hold on;
    plot(timelineSec(1:end-1), diff(result.xWLS(4, :)))
    plot(timelineSec, result.xRTK(idxStateClkDrift, :))
    legend('WLS (clock bias)', 'RTK')
    xlabel('Time since start (s)'); ylabel('Clock drift (m/s)');
    figureWindowTitle(figures(end), 'Rx clock drift');
end

% Ambiguities
if ~isempty(idxStateAllSdAmb)
    figures = [figures figure];
    plot(timelineSec, result.xRTK(idxStateAllSdAmb, :), '.')
    xlabel('Time since start (s)'); ylabel('Ambiguities (cyc)');
    % legend('X', 'Y', 'Z');
    figureWindowTitle(figures(end), 'Ambiguities');
end

%% Innovations
figures = [figures figure];
subplot(2,1,1)
plot(result.prInnovations', '.')
xlabel('Time since start (s)'); ylabel('Code DD innovations (m)');
subplot(2,1,2)
plot(result.prInnovationCovariances', '.')
xlabel('Time since start (s)'); ylabel('Code DD innovation covariances (m²)');
figureWindowTitle(figures(end), 'Code DD innovations');

figures = [figures figure];
subplot(2,1,1)
plot(result.phsInnovations', '.')
xlabel('Time since start (s)'); ylabel('Phase DD innovations (m)');
subplot(2,1,2)
plot(result.phsInnovationCovariances', '.')
xlabel('Time since start (s)'); ylabel('Phase DD innovation covariances (m²)');
figureWindowTitle(figures(end), 'Phase DD innovations');

figures = [figures figure];
subplot(2,1,1)
plot(result.dopInnovations', '.')
xlabel('Time since start (s)'); ylabel('Doppler innovations (m/s)');
subplot(2,1,2)
plot(result.dopInnovationCovariances', '.')
xlabel('Time since start (s)'); ylabel('Doppler innovation covariances (m²/s²)');
figureWindowTitle(figures(end), 'Doppler innovations');

%% Rejected
figures = [figures figure]; 
subplot(3,1,1); 
area(timelineSec, [result.prRejectedHist', result.prInvalidHist', result.prNumDD' - result.prRejectedHist' - result.prInvalidHist']);
legend({'Rejected', 'Invalid', 'Used'});
ylabel('# Code DDs');
xlabel('Time since start (s)');
grid on
%
subplot(3,1,2);
area(timelineSec, [result.phsRejectedHist', result.phsInvalidHist', result.phsNumDD' - result.phsRejectedHist' - result.phsInvalidHist']);
legend({'Rejected', 'Invalid', 'Used'});
ylabel('# Phase DDs');
xlabel('Time since start (s)');
grid on
%
subplot(3,1,3);
area(timelineSec, [result.dopRejectedHist', result.dopInvalidHist', result.dopNumDD' - result.dopRejectedHist' - result.dopInvalidHist']);
legend({'Rejected', 'Invalid', 'Used'});
ylabel('# Doppler obs');
xlabel('Time since start (s)');
figureWindowTitle(figures(end), 'Outlier rejections');
grid on

%% Training plots
if contains(config.DATASET_TYPE, 'train')
    % Interpolate groundtruth at the computed position's time
    refInterpLla = interp1(ref.utcSeconds, ref.posLla, result.utcSeconds);
    assert(size(refInterpLla, 1) == size(result.xRTK, 2), 'Reference and computed position vectors are not the same size');
    % Position error
    posErrNed = Lla2Ned(refInterpLla, estPosLla);
    posErrHor = Lla2Hd(refInterpLla, estPosLla);
    
    % Compute score
    hErr95 = prctile(abs(posErrHor),95);
    hErr50 = prctile(abs(posErrHor),50);
    fprintf('\n ==== Score: %.4f ====\n', mean([hErr95, hErr50]));
    
    % Groundtruth velocity
    dtRef = diff(ref.tow);
    [xRef, yRef, zRef] = geodetic2ecef(wgs84Ellipsoid, ref.posLla(:, 1), ref.posLla(:, 2), ref.posLla(:, 3));
    refEcef = [xRef, yRef, zRef];
    refVelEcef = diff(refEcef) ./ dtRef;
    refVelTime = ref.utcSeconds(1:end-1) + dtRef/2;
    refVelEcefInterp = interp1(refVelTime, refVelEcef, result.utcSeconds);
    % Velocity error
    velErrEcef = refVelEcefInterp - result.xRTK(idxStateVel, :)';
    velErrNed = nan(nEpochs, 3);
    for iEpoch = 1:nEpochs
        Rn2e = compute_Rn2e(estPosXyz(iEpoch, 1), estPosXyz(iEpoch, 2), estPosXyz(iEpoch, 3));
        % Velocity to NED
        velErrNed(iEpoch, :) = Rn2e' * velErrEcef(iEpoch, :)';
    end

    % Position error
    figures = [figures figure];
    coord = {'North', 'East', 'Down'};
    title('Groundtruth - Estimation')
    for i = 1:3
        subplot(3,1,i); hold on;
        p1 = plot(timelineSec, posErrNed(:, i));
        p2 = plot(timelineSec, 3*result.posStdNed(:, i), 'r');
        plot(timelineSec, -3*result.posStdNed(:, i), 'r');
        h = [p1 p2];
        legend(h, {'Error', '±3\sigma'})
        xlabel('Time since start (s)'); ylabel([coord{i} ' error (m)']);
        grid on
        hold off
    end
    figureWindowTitle(figures(end), 'Position error');
    
    % Velocity error
    figures = [figures figure];
    title('Groundtruth - Estimation')
    for i = 1:3
        subplot(3,1,i); hold on;
        p1 = plot(timelineSec, velErrNed(:, i));
        p2 = plot(timelineSec, 3*result.velStdNed(:, i), 'r');
        plot(timelineSec, -3*result.velStdNed(:, i), 'r');
        h = [p1 p2];
        legend(h, {'Error', '±3\sigma'})
        xlabel('Time since start (s)'); ylabel([coord{i} ' error (m/s)']);
        grid on
        hold off
    end
    figureWindowTitle(figures(end), 'Velocity error');
    
    %% CDFs
    pctl = 95;
    % Horizontal
    hErrPctl = prctile(abs(posErrHor),pctl);
    [hErrF,hEerrX] = ecdf(abs(posErrHor));
    
    figures = [figures figure]; hold on;
    plot(hEerrX,hErrF,'LineWidth',2)
    plot([1;1]*hErrPctl, [0;1]*pctl/100, '--k')
    legend('CDF',sprintf('%d%% bound = %.2f', pctl, hErrPctl));
    xlabel('Horizontal error (m)'); ylabel('Frequency')
    title([config.campaignName ' - ' config.phoneName], 'Interpreter', 'none');
    figureWindowTitle(figures(end), 'Hor. pos. CDF');
    
    % Vertical
    vErrPctl = prctile(abs(posErrNed(:,3)),pctl);
    [vErrF,vErrX] = ecdf(abs(posErrNed(:,3)));
    
    figures = [figures figure]; hold on;
    plot(vErrX,vErrF,'LineWidth',2)
    plot([1;1]*vErrPctl, [0;1]*pctl/100, '--k')
    legend('CDF',sprintf('%d%% bound = %.2f', pctl, vErrPctl));
    xlabel('Vertical error error (m)'); ylabel('Frequency')
    title([config.campaignName ' - ' config.phoneName], 'Interpreter', 'none');
    figureWindowTitle(figures(end), 'Ver. pos. CDF');
    
    % Velocity
    velErrPctl = prctile(abs(velErrNed),pctl);
    for iDim = 1:3
        [velErrF{iDim}, velErrX{iDim}] = ecdf(abs(velErrNed(:, iDim)));
    end
    
    figures = [figures figure]; hold on;
    colors = [0 0 1; 0 1 0; 1 0 0];
    for iDim = 1:3
        plot(velErrX{iDim},velErrF{iDim},'LineWidth',2, 'Color', colors(iDim, :))
        plot([1;1]*velErrPctl(iDim), [0;1]*pctl/100, '--', 'Color', colors(iDim, :))
    end
    legend({'N',sprintf('%d%% bound = %.2f', pctl, velErrPctl(1)), ...
            'E',sprintf('%d%% bound = %.2f', pctl, velErrPctl(2)), ...
            'D',sprintf('%d%% bound = %.2f', pctl, velErrPctl(3))}, ...
            'Location','northeastoutside');
    xlabel('Velocity error error (m/s)'); ylabel('Frequency')
    title([config.campaignName ' - ' config.phoneName], 'Interpreter', 'none');
    figureWindowTitle(figures(end), 'Velocity CDF');
end

%% Group plots
navi = [];
if ~isfield(navi, 'nav_report_group')
    try
        navi = groupPlots(figures, navi);
    catch e
        warning(['Exception while grouping plots: ' e.message]);
    end
end
end

