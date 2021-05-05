function [xEst] = navigate(gnssRnx, imuRaw, nav, iono)
%NAVIGATE Summary of this function goes here
%   Detailed explanation goes here

%% Initializations
nStates = PVTUtils.getNumStates();
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
    
    % Sequentally update with all observations
    for iObs = 1:length(prCorr)
        % Pack arguments that are common for Code and Doppler observations
        hArgs.obsFreqHz = gnss.obs(iObs).D_fcarrier_Hz;
        hArgs.obsConst = gnss.obs(iObs).constellation;
        hArgs.satPos = satPos(:, iObs);
        hArgs.satVel = satVel(:, iObs);
        hArgs.satClkBias = satClkBias(iObs);
        hArgs.satClkDrift = satClkDrift(iObs);
        hArgs.satElev = satElDeg(iObs);
        
        % Pack code observation
        hArgs.obs = prCorr(iObs);
        hArgs.sigmaObs = gnss.obs(iObs).C_sigma;
        % Process code observation
        esekf = EKF.processObservation(esekf, gnss.utcMillis, ...
            @fTransition, fArgs, ...
            @hCodeObs, hArgs, ...
            'Code');
        
        % Pack Doppler observation
        hArgs.obs = prRate(iObs);
        hArgs.sigmaObs = gnss.obs(iObs).D_sigma .* ...
            Constants.CELERITY ./ hArgs.obsFreqHz; % Doppler sigma in mps
        % Process Doppler observation
        esekf = EKF.processObservation(esekf, gnss.utcMillis, ...
            @fTransition, fArgs, ...
            @hDopplerObs, hArgs, ...
            'Doppler');
    end
    
    %%%%%%%%%%%%%%%%%% DEBUG %%%%%%%%%%%%%%%%
%     esekf.tx
%     Xyz2Lla(esekf.x(1:3)')
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
F = eye(PVTUtils.getNumStates);
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
Qif = dt * Config.SIGMA_CLK_INTERFREQ^2 * eye(PVTUtils.getNumFrequencies - 1);
Qis = dt * Config.SIGMA_CLK_INTERSYS^2 * eye(PVTUtils.getNumConstellations - 1);
Q = blkdiag(zeros(3), Qvel, Qclk, Qif, Qis);
end %end of function fTransition
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [z, y, H, R] = hCodeObs(~, hArgs)
% HCODEOBS provides the measurement model for the sequential code
% observations

% Initializations
idxStatePos = PVTUtils.getStateIndex(PVTUtils.ID_POS);
idxStateClkBias = PVTUtils.getStateIndex(PVTUtils.ID_CLK_BIAS);
idxStateFreq = PVTUtils.getStateIndex(PVTUtils.ID_INTER_FREQ_BIAS);
idxStateConstel = PVTUtils.getStateIndex(PVTUtils.ID_INTER_SYS_BIAS);
idxFreq = PVTUtils.getFreqIdx(hArgs.obsFreqHz);
idxConstel = PVTUtils.getConstelIdx(hArgs.obsConst);

% Observation
z = hArgs.obs;

% Distance between receiver and satellite. Second term is Sagnac effect.
dist = norm(hArgs.xEst(idxStatePos) - hArgs.satPos) + ...
    Constants.OMEGA_E/Constants.CELERITY * ...
    (hArgs.satPos(1)*hArgs.xEst(idxStatePos(2)) - ...
    hArgs.satPos(2)*hArgs.xEst(idxStatePos(1)));

% Inter-frequency bias if it's not ref freq
if isempty(idxStateFreq) || idxFreq == 1, interFreqBias = 0; % If only 1 freq or this is ref freq
else, interFreqBias = hArgs.xEst(idxStateFreq(idxFreq-1)); end
% Inter-system bias if it's not ref system
if isempty(idxStateConstel) || idxConstel == 1, interSysBias = 0; % If only 1 const or this is ref const
else, interSysBias = hArgs.xEst(idxStateConstel(idxConstel-1)); end
% Observation estimation
y = dist + ...                                  % Distance between receiver and satellite.
    hArgs.xEst(idxStateClkBias) - ...           % Receiver clock bias
    Constants.CELERITY * hArgs.satClkBias + ... % Satellite clock bias
    interFreqBias + ...                         % Inter-frequency bias
    interSysBias;                               % Inter-system bias 

% Jacobian matrix
H = zeros(1, PVTUtils.getNumStates);
H(idxStatePos) = (hArgs.xEst(idxStatePos) - hArgs.satPos)/dist;
H(idxStateClkBias) = 1;
H(idxStateFreq) = idxFreq > 1; % 1 if not ref freq
H(idxStateConstel) = -(idxConstel > 1); % -1 if not ref const

% Measurement covariance matrix
R = computeMeasCovariance(hArgs.satElev, hArgs.sigmaObs, ...
    Config.SIGMA_PR_M, hArgs.obsConst);
end %end of function hCodeObs
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [z, y, H, R] = hDopplerObs(~, hArgs)
% HDOPPLEROBS provides the measurement model for the sequential Doppler
% observations

% Initializations
idxStatePos = PVTUtils.getStateIndex(PVTUtils.ID_POS);
idxStateVel = PVTUtils.getStateIndex(PVTUtils.ID_VEL);
idxStateClkDrift = PVTUtils.getStateIndex(PVTUtils.ID_CLK_DRIFT);

% Observation
z = hArgs.obs;

% Distance between receiver and satellite. Second term is Sagnac effect.
dist = norm(hArgs.xEst(idxStatePos) - hArgs.satPos) + ...
    Constants.OMEGA_E/Constants.CELERITY * ...
    (hArgs.satPos(1)*hArgs.xEst(idxStatePos(2)) - ...
    hArgs.satPos(2)*hArgs.xEst(idxStatePos(1)));

% Unit vector from user to satellite
vUS = unitVector(hArgs.satPos - hArgs.xEst(idxStatePos));

% Observation estimation
y = dot(hArgs.satVel - hArgs.xEst(idxStateVel), vUS) + ...  % Radial velocity from user to sat
    hArgs.xEst(idxStateClkDrift) - ...                      % Receiver clock drift
    Constants.CELERITY*hArgs.satClkDrift;                   % Satellite clock drift

% Jacobian matrix
H = zeros(1, PVTUtils.getNumStates);
H(idxStatePos) = (hArgs.xEst(idxStatePos) - hArgs.satPos)/dist;
H(idxStateClkDrift) = 1;

% Measurement covariance matrix
R = computeMeasCovariance(hArgs.satElev, hArgs.sigmaObs, ...
    Config.SIGMA_DOP_MPS, hArgs.obsConst);
end %end of function hDopplerObs
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%