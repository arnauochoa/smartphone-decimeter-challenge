function plotResults(ref, estPosLla, result)
%PLOTRESULTS Summary of this function goes here
%   Detailed explanation goes here
close all;

% Initializations
config = Config.getInstance;
idxStateVel = PVTUtils.getStateIndex(PVTUtils.ID_VEL);
idxStateClkDrift = PVTUtils.getStateIndex(PVTUtils.ID_CLK_DRIFT);
idxStateAllSdAmb = PVTUtils.getStateIndex(PVTUtils.ID_SD_AMBIGUITY);
timelineSec = (result.utcSeconds - result.utcSeconds(1));
% nEpochs = size(result.xEst, 2);
figures = [];
basemap = 'none';

%% State plots
% Position
figures = [figures figure];
if strcmp(config.DATASET_TYPE, 'test')
    geoplot(estPosLla(:, 1), estPosLla(:, 2), '.-');
else
    if isprop(config, 'OBS_RINEX_REF_XYZ') % Use observations from rinex
        geoplot(ref.posLla(1, 1), ref.posLla(1, 2), 'x', estPosLla(:, 1), estPosLla(:, 2), '.', 'LineWidth', 1);
    else
        geoplot(ref.posLla(:, 1), ref.posLla(:, 2), '.-', estPosLla(:, 1), estPosLla(:, 2), '.-');
    end
    legend('Groundtruth', 'Computed');
end
geobasemap(basemap);
figureWindowTitle(figures(end), 'Map');

% Velocity
figures = [figures figure];
plot(timelineSec, result.xEst(idxStateVel, :))
xlabel('Time since start (s)'); ylabel('Velocity (m/s)');
legend('X', 'Y', 'Z');
figureWindowTitle(figures(end), 'Velocity');

% Rx clock drift
figures = [figures figure];
plot(timelineSec, result.xEst(idxStateClkDrift, :))
xlabel('Time since start (s)'); ylabel('Clock drift (m/s)');
figureWindowTitle(figures(end), 'Rx clock drift');

% Ambiguities
figures = [figures figure];
plot(timelineSec, result.xEst(idxStateAllSdAmb, :), '.')
xlabel('Time since start (s)'); ylabel('Ambiguities (cyc)');
% legend('X', 'Y', 'Z');
figureWindowTitle(figures(end), 'Ambiguities');

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
subplot(3,1,1); plot(timelineSec, result.prRejectedHist)
xlabel('Time since start (s)'); ylabel('% rejected Code obs');
subplot(3,1,2); plot(timelineSec, result.phsRejectedHist)
xlabel('Time since start (s)'); ylabel('% rejected Phase obs');
subplot(3,1,3); plot(timelineSec, result.dopRejectedHist)
xlabel('Time since start (s)'); ylabel('% rejected Doppler obs');
figureWindowTitle(figures(end), 'Outlier rejections');

%% Training plots
if contains(config.DATASET_TYPE, 'train')
    % Interpolate groundtruth at the computed position's time
    refInterpLla = interp1(ref.utcSeconds, ref.posLla, result.utcSeconds);
    assert(size(refInterpLla, 1) == size(result.xEst, 2), 'Reference and computed position vectors are not the same size');
    % Position error
    nedError = Lla2Ned(refInterpLla, estPosLla);
    hError = Lla2Hd(refInterpLla, estPosLla);
    
    % Groundtruth velocity
    dtRef = diff(ref.tow);
    [xRef, yRef, zRef] = geodetic2ecef(wgs84Ellipsoid, ref.posLla(:, 1), ref.posLla(:, 2), ref.posLla(:, 3));
    refEcef = [xRef, yRef, zRef];
    refVelEcef = diff(refEcef) ./ dtRef;
    refVelTime = ref.utcSeconds(1:end-1) + dtRef/2;
    refVelEcefInterp = interp1(refVelTime, refVelEcef, result.utcSeconds);
    % Velocity error
    velErr = refVelEcefInterp - result.xEst(idxStateVel, :)';

    % Position error
    figures = [figures figure];
    plot(timelineSec, nedError)
    xlabel('Time since start (s)'); ylabel('Position error (m)');
    legend('N', 'E', 'D')
    title('Groundtruth - Estimation')
    grid on
    figureWindowTitle(figures(end), 'Position error');
    
    % Velocity error
    figures = [figures figure];
    plot(timelineSec, velErr)
    xlabel('Time since start (s)'); ylabel('Velocity error (m)');
    legend('X', 'Y', 'Z');
    title('Groundtruth - Estimation')
    grid on
    figureWindowTitle(figures(end), 'Velocity error');
    
    %% CDFs
    pctl = 95;
    % Horizontal
    hErrPctl = prctile(abs(hError),pctl);
    [hErrF,hEerrX] = ecdf(abs(hError));
    
    figures = [figures figure]; hold on;
    plot(hEerrX,hErrF,'LineWidth',2)
    plot([1;1]*hErrPctl, [0;1]*pctl/100, '--k')
    legend('CDF',sprintf('%d%% bound = %.2f', pctl, hErrPctl));
    xlabel('Horizontal error (m)'); ylabel('Frequency')
    title([config.campaignName ' - ' config.phoneName], 'Interpreter', 'none');
    figureWindowTitle(figures(end), 'Hor. pos. CDF');
    
    % Vertical
    vErrPctl = prctile(abs(nedError(:,3)),pctl);
    [vErrF,vErrX] = ecdf(abs(nedError(:,3)));
    
    figures = [figures figure]; hold on;
    plot(vErrX,vErrF,'LineWidth',2)
    plot([1;1]*vErrPctl, [0;1]*pctl/100, '--k')
    legend('CDF',sprintf('%d%% bound = %.2f', pctl, vErrPctl));
    xlabel('Vertical error error (m)'); ylabel('Frequency')
    title([config.campaignName ' - ' config.phoneName], 'Interpreter', 'none');
    figureWindowTitle(figures(end), 'Ver. pos. CDF');
    
    % Velocity
    velErrPctl = prctile(abs(velErr),pctl);
    for iDim = 1:3
        [velErrF{iDim}, velErrX{iDim}] = ecdf(abs(velErr(:, iDim)));
    end
    
    figures = [figures figure]; hold on;
    colors = [0 0 1; 0 1 0; 1 0 0];
    for iDim = 1:3
        plot(velErrX{iDim},velErrF{iDim},'LineWidth',2, 'Color', colors(iDim, :))
        plot([1;1]*velErrPctl(iDim), [0;1]*pctl/100, '--', 'Color', colors(iDim, :))
    end
    legend({'X',sprintf('%d%% bound = %.2f', pctl, velErrPctl(1)), ...
            'Y',sprintf('%d%% bound = %.2f', pctl, velErrPctl(2)), ...
            'Z',sprintf('%d%% bound = %.2f', pctl, velErrPctl(3))}, ...
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

function navi = groupPlots(figures, navi)
desktop = com.mathworks.mde.desk.MLDesktop.getInstance;
navi.nav_report_group = desktop.addGroup('Navigation report');
desktop.setGroupDocked('Navigation report', 0);
myDim   = java.awt.Dimension(length(figures), 1);   % columns, rows
desktop.setDocumentArrangement('Navigation report', 1, myDim)
bakWarn = warning('off','MATLAB:HandleGraphics:ObsoletedProperty:JavaFrame');
warning('off')
for k=1:length(figures)
    figures(k).WindowStyle = 'docked';
    drawnow;
    pause(0.02);  % Magic, reduces rendering errors
    set(get(handle(figures(k)), 'javaframe'), 'GroupName', 'Navigation report');
end
warning('on')
warning(bakWarn);
end

