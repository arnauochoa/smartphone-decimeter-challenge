function [xEst] = navigate(gnssRnx, imuRaw, nav, iono)
%NAVIGATE Summary of this function goes here
%   Detailed explanation goes here

%% Initializations
nStates = PVTUtils.getNStates();
% EKF:
esekf = EKF.build(uint16(nStates));
% Disable outlier rejection:
esekf.probabilityOfFalseOutlierRejection = 0;


%% Obtain first position
[esekf.x, esekf.P, esekf.tx] = getFirstPosition(gnssRnx, nav);

% Loop variables
thisUtcMillis = esekf.tx; % First time is from the first GNSS estimation
idxEst = 1;
xEst = esekf.x; % TODO change to error state

% First gnss observations -> same as for the first approx pos
gnss = getNextGnss(thisUtcMillis, gnssRnx, 'this');
hasEnded = isempty(gnss); % TODO check imu
while ~hasEnded % while there are more observations/measurements
    thisUtcMillis = gnss.utcMillis; % TODO check imu time
    
    % Get states of satellites selected in Config
    [satPos, satClkBias, satClkDrift, satVel] = ...
        compute_satellite_state_all(gnss.tow, gnss.obs, nav, Config.CONSTELLATIONS);
    
    % Remove invalid observations (no ephem, elevation mask, large dopplers)
    [gnss.obs, satPos, satClkBias, satClkDrift, satVel] = ...
        filterObs(gnss.obs, satPos, satClkBias, satClkDrift, satVel, xEst(1:3, idxEst));
    
    % Compute elevation and azimuth of satellites
    [satAzDeg, satElDeg, rxLLH] = getSatAzEl(satPos, xEst(1:3, idxEst));
    
    % Apply iono and tropo corrections
    switch Config.IONO_CORRECTION
        case 'Klobuchar'
            ionoCorr = compute_klobuchar_iono_correction(...
                iono.alpha,              ...
                iono.beta,               ...
                deg2rad(satElDeg),  ...
                deg2rad(satAzDeg),  ...
                deg2rad(rxLLH(1)),  ...
                deg2rad(rxLLH(2)),  ...
                gnss.tow);
            ionoCorr = ionoCorr .* (Constants.GPS_L1_HZ./[gnss.obs(:).D_fcarrier_Hz]).^2;
        otherwise
            ionoCorr = zeros(1, length(gnss.obs));
    end
    tropo = compute_saastamoinen_tropo_correction(rxLLH(3), deg2rad(satElDeg), deg2rad(rxLLH(1)));
    % Apply pr correction and convert doppler (Hz) to pr rate (m/s)
    prCorr = [gnss.obs(:).C]' - ionoCorr' - tropo';
    prRate = -[gnss.obs(:).D_Hz]' .* Constants.CELERITY ./ [gnss.obs(:).D_fcarrier_Hz]';
    
    % Transition model arguments
    fArgs.xEst = xEst(:, idxEst);
    % Measurement model arguments
    hArgs.xEst = xEst(:, idxEst);
    
    for iObs = 1:length(prCorr)
        hArgs.obsFreq = gnss.obs(iObs).D_fcarrier_Hz;
        hArgs.obsConst = gnss.obs(iObs).constellation;
        hArgs.satPos = satPos(:, iObs);
        hArgs.satVel = satVel(:, iObs);
        hArgs.satClkBias = satClkBias(iObs);
        hArgs.satClkDrift = satClkDrift(iObs);
        hArgs.satElev = satElDeg(iObs);
        
        % Process code observation
        hArgs.obs = prCorr(iObs);
        hArgs.sigmaObs = gnss.obs(iObs).C_sigma;
        esekf = EKF.processObservation(esekf, gnss.utcMillis, ...
            @fTransition, fArgs, ...
            @hCodeObs, hArgs, ...
            'gnssObs');
        
        % Process Doppler observation
        hArgs.obs = prRate(iObs);
        hArgs.sigmaObs = gnss.obs(iObs).D_sigma .* ...
            Constants.CELERITY ./ hArgs.obsFreq;
        esekf = EKF.processObservation(esekf, gnss.utcMillis, ...
            @fTransition, fArgs, ...
            @hDopplerObs, hArgs, ...
            'gnssObs');
    end
    
    
    % loop over measurements and call processObs for each
    
    
    %%%%%%%%%%%%%%%%%% DEBUG %%%%%%%%%%%%%%%%
    esekf.tx
    Xyz2Lla(esekf.x(1:3)')
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    idxEst = idxEst + 1;
    xEst(:, idxEst) = esekf.x; % TODO perform correction when using error-state
    
    % Check if there are more measurements/observations
    gnss = getNextGnss(thisUtcMillis, gnssRnx);
    hasEnded = isempty(gnss); % TODO check imu
end

end

% State transition function
function [x2, F, Q] = fTransition(x1, t1, t2, fArgs)
dt = t2 - t1;
F = eye(10);
% dp/dv
F(1,4) = dt; F(2,5) = dt; F(3,6) = dt;
% d clkbias/d clkdrift
F(7,8) = dt;
% State prediction
x2 = F * x1;
% Process noise covariance matrix
rotN2E = compute_Rn2e(fArgs.xEst(1), fArgs.xEst(2), fArgs.xEst(3));
Qvel = dt * rotN2E * diag(Config.SIGMA_VEL_NED.^2) * rotN2E';
Qclk(1, 1) = dt * Config.SIGMA_CLK_BIAS^2 + 1/3 * dt^3 * Config.SIGMA_CLK_DRIFT^2;
Qclk(1, 2) = 1/2 * dt^2 * Config.SIGMA_CLK_DRIFT^2;
Qclk(2, 1) = Qclk(1, 2);
Qclk(2, 2) =  dt * Config.SIGMA_CLK_DRIFT^2;
Qif = dt * Config.SIGMA_CLK_INTERFREQ^2 * eye(Config.getNumFreq - 1);
Qis = dt * Config.SIGMA_CLK_INTERSYS^2 * eye(Config.getNumConst - 1);
Q = blkdiag(zeros(3), Qvel, Qclk, Qif, Qis);
end


% Measurement function
function [z, y, H, R] = hMeasurement(~, hArgs)
nObs = length(hArgs.prCorr);
freqs = unique(hArgs.obsFreq);
nFreqs = length(freqs);

indGoodC = find(~isnan(hArgs.prCorr));
nObsC = length(indGoodC);
indGoodD = find(~isnan(hArgs.prRate));
nObsD = length(indGoodD);

z = [hArgs.prCorr(indGoodC); hArgs.prRate(indGoodD)];

H = zeros(nObsC + nObsD, 8  + nFreqs-1 + Config.getNumConst()-1);
y = zeros(size(z));
iRowC = 1; iRowD = 1;
for iObs = 1:nObs
    % check frequency and constellation of the observable
    idxFreq = find(freqs == hArgs.obsFreq(iObs));
    idxConst = strfind(Config.CONSTELLATIONS, hArgs.obsConst(iObs));
    % measurement prediction at x0 (excluding tropo and iono delays which are already corrected on the measurements)
    dist = norm(hArgs.xEst(1:3) - hArgs.satPos(:, iObs)) + ...
        Constants.OMEGA_E/Constants.CELERITY * ...
        (hArgs.satPos(1, iObs)*hArgs.xEst(2) - hArgs.satPos(2, iObs)*hArgs.xEst(1));
    if ~isnan(hArgs.prCorr(iObs))
        % Fill y and H for Code measurements
        y(iRowC) = dist + (hArgs.xEst(7) - ...
            Constants.CELERITY * hArgs.satClkBias(iObs)) + ...
            hArgs.xEst(8+idxFreq-1)*(idxFreq>1) + ...
            hArgs.xEst(8+nFreqs-1+idxConst-1)*(idxConst>1);
        % Jacobian matrix: C rows
        H(iRowC, 1:3) = (hArgs.xEst(1:3) - hArgs.satPos(:, iObs))/dist;
        H(iRowC, 7) = 1;
        H(iRowC, 8+idxFreq-1) = idxFreq > 1; % 1 if not 1st freq
        H(iRowC, 8+nFreqs-1+idxFreq-1) = -(idxConst > 1); % -1 if not 1st const
        iRowC = iRowC + 1;
    end
    
    if ~isnan(hArgs.prRate(iObs))
        % Unit vector from user to satellite
        vUS = unitVector(hArgs.satPos(:,iObs) - hArgs.xEst(1:3));
        % Fill y and H for Doppler measurements
        y(nObsC+iRowD) = dot(hArgs.satVel(:, iObs) - hArgs.xEst(4:6), vUS) ...
            + hArgs.xEst(8) - MagnitudeConstants.celerity*hArgs.satClkDrift(iObs);
        % Jacobian matrix: D rows
        H(nObsC+iRowD, 4:6) = (hArgs.xEst(1:3) - hArgs.satPos(:, iObs))/dist;
        H(nObsC+iRowD, 8) = 1;
        iRowD = iRowD + 1;
    end
end
assert(isequal(size(y), size(z)), 'Vectors y and z should be the same size');

Rpr = computeMeasCovariance(hArgs.satElev(indGoodC), hArgs.sigmaPr(indGoodC), ...
    Config.SIGMA_PR_M, hArgs.obsConst(indGoodC));
Rdop = computeMeasCovariance(hArgs.satElev(indGoodD), hArgs.sigmaPrRate(indGoodD), ...
    Config.SIGMA_DOP_MPS, hArgs.obsConst(indGoodD));
R = blkdiag(Rpr, Rdop);
end