function [result] = navigate(phoneRnx, imuMeas, nav, osrRnx, ref)
%NAVIGATE Summary of this function goes here
%   Detailed explanation goes here

%% Initializations
config = Config.getInstance;
idxStatePos = PVTUtils.getStateIndex(PVTUtils.ID_POS);
% idxStateAllSdAmb = PVTUtils.getStateIndex(PVTUtils.ID_SD_AMBIGUITY);
% idxStateVel = PVTUtils.getStateIndex(PVTUtils.ID_VEL);
nStates = PVTUtils.getNumStates();
nSatellites = PVTUtils.getNumSatelliteIndices();
gnssEpochs = unique(phoneRnx.utcSeconds);
nGnssEpochs = length(gnssEpochs);
% EKF:
ekf = EKF.build(uint16(nStates), config.P_FALSE_OUTLIER_REJECT);

% TODO: debugging variables, check which need to be kept
result.prInnovations = nan(nSatellites, nGnssEpochs);
result.prInnovationCovariances = nan(nSatellites, nGnssEpochs);
result.prRejectedHist = zeros(1, nGnssEpochs);
result.phsInnovations = nan(nSatellites, nGnssEpochs);
result.phsInnovationCovariances = nan(nSatellites, nGnssEpochs);
result.phsRejectedHist = zeros(1, nGnssEpochs);
result.dopInnovations = nan(nSatellites, nGnssEpochs);
result.dopInnovationCovariances = nan(nSatellites, nGnssEpochs);
result.dopRejectedHist = zeros(1, nGnssEpochs);

%% Obtain first position
[x0, ekf.P, ekf.tx] = getFirstPosition(phoneRnx, nav);
 % TODO remove >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
% idxRef = find(ref.utcSeconds > ekf.tx, 1, 'first');
% [x, y, z] = geodetic2ecef(wgs84Ellipsoid, ref.posLla(idxRef, 1), ref.posLla(idxRef, 2), ref.posLla(idxRef, 3));
% x0(idxStatePos) = [x y z]';
% <<<<<<<<<<<<<<<<<<<<<<<<<<<<<
ekf.x = zeros(PVTUtils.getNumStates, 1);
ekf.x(idxStatePos) = x0(idxStatePos) - osrRnx.statPos;

% Loop variables
% thisUtcSeconds -> time ref for both IMU and GNSS.
thisUtcSeconds = ekf.tx; % First time is from the first GNSS estimation
idxEst = 1;
result.xEst(:, 1) = x0; % TODO change to error state, prealocate if num pos is known (= groundtruth?)
result.sigmaHist = zeros(nStates, 1);

%% Evaluate all trajectory
% First gnss observations is the same as for the first approx position
[phoneGnss, osrGnss] = getNextGnss(thisUtcSeconds, phoneRnx, osrRnx, 'this');
hasEnded = isempty(phoneGnss); % TODO check imu

while ~hasEnded % while there are more observations/measurements
    idxRef = find(ref.utcSeconds > thisUtcSeconds, 1, 'first'); % TODO remove
    % First iteration: x0 is result from LS
    if idxEst == 1, x0 = result.xEst(:, 1);
    else,           x0 = result.xEst(:, idxEst-1); end
    
    thisUtcSeconds = phoneGnss.utcSeconds; % TODO check imu time
    
    % Get states of satellites selected in Config
    [satPos, satClkBias, satClkDrift, satVel] = ...
        compute_satellite_state_all(phoneGnss.tow, phoneGnss.obs, nav, config.CONSTELLATIONS);
    
    % Remove invalid observations (no ephem, elevation mask)
    [phoneGnss.obs, sat] = ...
        filterObs(phoneGnss.obs, satPos, satClkBias, satClkDrift, satVel, x0(idxStatePos));
    
    % Obtain satellite elevations
    [sat.azDeg, sat.elDeg, ~] = getSatAzEl(sat.pos, x0(idxStatePos));
    
    %% Ref observation
%     fArgs.x0 = x0;
%     hArgs.x0 = x0;
%     hArgs.statPos = osrRnx.statPos;
%     hArgs.obs = Lla2Xyz(ref.posLla(idxRef, :))' - hArgs.statPos;
%     hArgs.sigmaObs = 1;
%     label = 'ref';
%     [ekf, innovation, innovationCovariance, rejected, ~, ~] = ...
%         EKF.processObservation(ekf, thisUtcSeconds,           ...
%         @fTransition, fArgs,                                    ...
%         @hRefObs, hArgs,                                        ...
%         label);
%     x0 = updateTotalState(ekf.x, osrRnx.statPos);
    
    result.utcSeconds(idxEst) = thisUtcSeconds;
    result.gpsWeekN(idxEst) = phoneGnss.weekN;
    result.gpsTow(idxEst) = phoneGnss.tow;
    if isempty(phoneGnss.obs)
        fprintf(2, 'TOW = %d - Not enough observations to estimate a potition. Propagating state.\n', phoneGnss.tow);
        % Initial estimate for the transition model
        fArgs.x0 = x0;
        esekf = EKF.propagateState(esekf, thisUtcSeconds, @fTransition, fArgs);
%         fprintf(2, 'TOW = %d - Not enough observations to estimate a potition. Skipping epoch.\n', phoneGnss.tow);
%         [phoneGnss, osrGnss] = getNextGnss(thisUtcSeconds, phoneRnx, osrRnx);
%         hasEnded = isempty(phoneGnss); % TODO check imu
%         continue
    else
        doubleDifferences = computeDoubleDifferences(osrGnss, phoneGnss, sat.pos, sat.elDeg);
        [x0, ekf, result] = updateWithDD(x0, ekf, thisUtcSeconds, idxEst, osrRnx, doubleDifferences, result);
        
        [x0, ekf, result] = updateWithDoppler(x0, ekf, thisUtcSeconds, idxEst, osrRnx, phoneGnss, sat, result);
    end
    
    result.xEst(:, idxEst) = x0;
    result.sigmaHist(:, idxEst) = sqrt(diag(ekf.P));
    idxEst = idxEst + 1;
    
    % Check if there are more measurements/observations
    [phoneGnss, osrGnss] = getNextGnss(thisUtcSeconds, phoneRnx, osrRnx);
    hasEnded = isempty(phoneGnss); % TODO check imu
end

end

function [x0, ekf, result] = updateWithDD(x0, ekf, thisUtcSeconds, idxEst, osrRnx, doubleDifferences, result)
% UPDATEWITHDD Performs the KF update with the double differenced
% observations

% Initializations

% Sequentally update with all DDs
for iObs = 1:length(doubleDifferences)
    idxSat = PVTUtils.getSatelliteIndex(doubleDifferences(iObs).varSatPrn, ...
        doubleDifferences(iObs).constel);
    
    %% Pack arguments that are common for all DDs
    % Transition model arguments
    fArgs.x0 = x0;
    fArgs.obsConst = doubleDifferences(iObs).constel;
    fArgs.pivSatPrn = doubleDifferences(iObs).pivSatPrn;
    fArgs.varSatPrn = doubleDifferences(iObs).varSatPrn;
%     fArgs.statPos = osrRnx.statPos;
    % Measurement model arguments
    hArgs.x0 = x0;
    hArgs.statPos = osrRnx.statPos;
    hArgs.obsConst = doubleDifferences(iObs).constel;
    hArgs.freqHz = doubleDifferences(iObs).freqHz;
    hArgs.pivSatPrn = doubleDifferences(iObs).pivSatPrn;
    hArgs.varSatPrn = doubleDifferences(iObs).varSatPrn;
    hArgs.pivSatPos = doubleDifferences(iObs).pivSatPos;
    hArgs.varSatPos = doubleDifferences(iObs).varSatPos;
    hArgs.satElDeg = [doubleDifferences(iObs).pivSatElDeg
        doubleDifferences(iObs).varSatElDeg];
    hArgs.pivSatCmcSd = doubleDifferences(iObs).pivSatCmcSd;
    hArgs.varSatCmcSd = doubleDifferences(iObs).varSatCmcSd;
    
    %% Code DD observation
    hArgs.obs = doubleDifferences(iObs).C;
    hArgs.sigmaObs = [doubleDifferences(iObs).pivSatSigmaC
        doubleDifferences(iObs).varSatSigmaC];
    
    % Label to show on console when outliers are detected
    label = sprintf('Code DD (%c%d-%c%d, f = %g)', ...
        doubleDifferences(iObs).constel,        ...
        doubleDifferences(iObs).pivSatPrn,      ...
        doubleDifferences(iObs).constel,        ...
        doubleDifferences(iObs).varSatPrn,      ...
        doubleDifferences(iObs).freqHz);
    % Process code observation
    [ekf, innovation, innovationCovariance, rejected, ~, ~] = ...
        EKF.processObservation(ekf, thisUtcSeconds,           ...
        @fTransition, fArgs,                                    ...
        @hCodeDD, hArgs,                                        ...
        label);
    
    result.prInnovations(idxSat, idxEst) = innovation;
    result.prInnovationCovariances(idxSat, idxEst) = innovationCovariance;
    result.prRejectedHist(idxEst) = result.prRejectedHist(idxEst) + rejected;
    
    % Update total-state with absolute position
    x0 = updateTotalState(ekf.x, osrRnx.statPos);
    
    %% Phase DD observation
    if ~isnan(doubleDifferences(iObs).L)
    %     idxStatePivSat = PVTUtils.getStateIndex(PVTUtils.ID_SD_AMBIGUITY, hArgs.pivSatPrn, hArgs.obsConst);
    %     idxStateVarSat = PVTUtils.getStateIndex(PVTUtils.ID_SD_AMBIGUITY, hArgs.varSatPrn, hArgs.obsConst);
        % Ambiguities: set to CMC if it's 0 (not estimated yet for this sat) 
        % TODO: what if N is estimated as 0
    %     if esekf.x(idxStatePivSat) == 0, esekf.x(idxStatePivSat) = hArgs.pivSatCmcSd; end
    %     if esekf.x(idxStateVarSat) == 0, esekf.x(idxStateVarSat) = hArgs.varSatCmcSd; end

        hArgs.obs = doubleDifferences(iObs).L;
        hArgs.sigmaObs = [doubleDifferences(iObs).pivSatSigmaL
            doubleDifferences(iObs).varSatSigmaL];

        % Label to show on console when outliers are detected
        label = sprintf('Phase DD (%c%d-%c%d, f = %g)', ...
            doubleDifferences(iObs).constel,        ...
            doubleDifferences(iObs).pivSatPrn,      ...
            doubleDifferences(iObs).constel,        ...
            doubleDifferences(iObs).varSatPrn,      ...
            doubleDifferences(iObs).freqHz);
        % Process code observation
        [ekf, innovation, innovationCovariance, rejected, ~, ~] = ...
            EKF.processObservation(ekf, thisUtcSeconds,           ...
            @fTransition, fArgs,                                    ...
            @hPhaseDD, hArgs,                                        ...
            label);

        result.phsInnovations(idxSat, idxEst) = innovation;
        result.phsInnovationCovariances(idxSat, idxEst) = innovationCovariance;
        result.phsRejectedHist(idxEst) = result.phsRejectedHist(idxEst) + rejected;

        % Update total-state with absolute position
        x0 = updateTotalState(ekf.x, osrRnx.statPos);
    else
        pause;
    end
end
% Percentage of rejected code observations
result.prRejectedHist(idxEst) = 100*result.prRejectedHist(idxEst) / length(doubleDifferences);
result.phsRejectedHist(idxEst) = 100*result.phsRejectedHist(idxEst) / length(doubleDifferences);
end %end of function updateWithDD
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [x0, ekf, result] = updateWithDoppler(x0, ekf, thisUtcSeconds, idxEst, osrRnx, phoneGnss, sat, result)
% UPDATEWITHDD Performs the KF update with the double differenced
% observations

% Initializations

% Sequentally update with all Doppler observations
for iObs = 1:length(phoneGnss.obs)
    thisObs = phoneGnss.obs(iObs);
    idxSat = PVTUtils.getSatelliteIndex(thisObs.prn, thisObs.constellation);
    if ~isempty(thisObs.D_Hz) && ~isnan(thisObs.D_Hz)
        % Pack arguments that are common for all observations
        fArgs.x0 = x0;
        fArgs.statPos = osrRnx.statPos;
        hArgs.x0 = x0;
        hArgs.satPos = sat.pos(:, iObs);
        hArgs.satVel = sat.vel(:, iObs);
        hArgs.satClkDrift = sat.clkDrift(iObs);
        hArgs.satElDeg = sat.elDeg(iObs);
        hArgs.obsConst = thisObs.constellation;

        % Convert Doppler from Hz to mps (pseudorange-rate)
        hArgs.obs = -hz2mps(thisObs.D_Hz, thisObs.D_fcarrier_Hz);
        hArgs.sigmaObs = hz2mps(thisObs.D_sigma, thisObs.D_fcarrier_Hz);
        
        % Label to show on console when outliers are detected
        label = sprintf('Doppler (%c%d, f = %g)',   ...
            thisObs.constellation,                  ...
            thisObs.prn,                      ...
            thisObs.D_fcarrier_Hz);
        
        % Process code observation
        [ekf, innovation, innovationCovariance, rejected, ~, ~] = ...
            EKF.processObservation(ekf, thisUtcSeconds,           ...
            @fTransition, fArgs,                                    ...
            @hDoppler, hArgs,                                        ...
            label);
        
        result.dopInnovations(idxSat, idxEst) = innovation;
        result.dopInnovationCovariances(idxSat, idxEst) = innovationCovariance;
        result.dopRejectedHist(idxEst) = result.dopRejectedHist(idxEst) + rejected;
        
        % Update total-state with absolute position
        x0 = updateTotalState(ekf.x, osrRnx.statPos);
    end
end
result.dopRejectedHist(idxEst) = 100*result.dopRejectedHist(idxEst) / length(phoneGnss.obs);
end %end of function updateWithDoppler
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% State transition function
function [x2, F, Q] = fTransition(x1, t1, t2, fArgs)
% Initializations
config = Config.getInstance;
idxStatePos = PVTUtils.getStateIndex(PVTUtils.ID_POS);
idxStateVel = PVTUtils.getStateIndex(PVTUtils.ID_VEL);
idxStateClkDrift = PVTUtils.getStateIndex(PVTUtils.ID_CLK_DRIFT);
idxStateAllSdAmb = PVTUtils.getStateIndex(PVTUtils.ID_SD_AMBIGUITY);

% Transition time step
dt = t2 - t1;

% State transition matrix
F = eye(PVTUtils.getNumStates);
% dp/dv
F(idxStatePos, idxStateVel) = dt * eye(3);

% State prediction
x2 = F * x1;

% Process noise covariance matrix
Q = zeros(PVTUtils.getNumStates);
Q(idxStateVel, idxStateVel) = diag(config.SIGMA_Q_VEL_XYZ.^2);
Q(idxStateClkDrift, idxStateClkDrift) = config.SIGMA_Q_CLK_DRIFT.^2;
Q(idxStateAllSdAmb, idxStateAllSdAmb) = (config.SIGMA_Q_SD_AMBIG.^2) * eye(PVTUtils.getNumSatelliteIndices);
end %end of function fTransition
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [z, y, H, R] = hRefObs(~, hArgs)
idxStatePos = PVTUtils.getStateIndex(PVTUtils.ID_POS);

z = hArgs.obs;
y = hArgs.x0(idxStatePos) - hArgs.statPos;
% Jacobian matrix
H = zeros(3, PVTUtils.getNumStates);
H(idxStatePos,idxStatePos) = eye(3);
R = diag(hArgs.sigmaObs);
end

function [z, y, H, R] = hCodeDD(~, hArgs)
% HCODEDD provides the measurement model for the sequential code
% double-differenced observations

% Initializations
config = Config.getInstance;
idxStatePos = PVTUtils.getStateIndex(PVTUtils.ID_POS);
rxPos = hArgs.x0(idxStatePos);

% Observation
z = hArgs.obs;

% Observation estimation
y = norm(hArgs.statPos - hArgs.pivSatPos) - ... % |stat - sat1|
    norm(rxPos - hArgs.pivSatPos) -         ... % |user - sat1|
    norm(hArgs.statPos - hArgs.varSatPos) + ... % |stat - sat2|
    norm(rxPos - hArgs.varSatPos);              % |user - sat2|

% Difference between LOS vectors of satellites towards receiver
pivSatLosVec = unitVector(hArgs.statPos - hArgs.pivSatPos);
varSatLosVec = unitVector(hArgs.statPos - hArgs.varSatPos);
ddLosVec = varSatLosVec - pivSatLosVec;

% Jacobian matrix
H = zeros(1, PVTUtils.getNumStates);
H(idxStatePos) = ddLosVec;

% Measurement covariance matrix, consider DD sigmas
R = config.COV_FACTOR_C * computeRtkMeasCovariance(hArgs.satElDeg, ...
    hArgs.sigmaObs, config.SIGMA_C_M, hArgs.obsConst);
end %end of function hCodeDD
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [z, y, H, R] = hPhaseDD(x, hArgs)
% HCODEDD provides the measurement model for the sequential code
% double-differenced observations

% Initializations
config = Config.getInstance;
idxStatePos = PVTUtils.getStateIndex(PVTUtils.ID_POS);
idxStatePivSat = PVTUtils.getStateIndex(PVTUtils.ID_SD_AMBIGUITY, hArgs.pivSatPrn, hArgs.obsConst);
idxStateVarSat = PVTUtils.getStateIndex(PVTUtils.ID_SD_AMBIGUITY, hArgs.varSatPrn, hArgs.obsConst);
rxPos = hArgs.x0(idxStatePos);
lambda = Constants.CELERITY / hArgs.freqHz;

% Observation
z = hArgs.obs;

% Observation estimation
y = norm(hArgs.statPos - hArgs.pivSatPos)   - ...   % |stat - sat1|
    norm(rxPos - hArgs.pivSatPos)           - ...   % |user - sat1|
    norm(hArgs.statPos - hArgs.varSatPos)   + ...   % |stat - sat2|
    norm(rxPos - hArgs.varSatPos)           + ...   % |user - sat2|
    lambda * x(idxStatePivSat)              - ...   % lambda * N_piv
    lambda * x(idxStateVarSat);                     % lambda * N_var

% Difference between LOS vectors of satellites towards receiver
pivSatLosVec = unitVector(hArgs.statPos - hArgs.pivSatPos);
varSatLosVec = unitVector(hArgs.statPos - hArgs.varSatPos);
ddLosVec = varSatLosVec - pivSatLosVec;

% Jacobian matrix
H = zeros(1, PVTUtils.getNumStates);
H(idxStatePos) = ddLosVec;
H(idxStatePivSat) = lambda;
H(idxStateVarSat) = -lambda;

% Measurement covariance matrix, consider DD sigmas
R = config.COV_FACTOR_L * computeRtkMeasCovariance(hArgs.satElDeg, ...
    hArgs.sigmaObs, config.SIGMA_L_M, hArgs.obsConst);
% z-y
end %end of function hPhaseDD
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [z, y, H, R] = hDoppler(~, hArgs)
% HDOPPLER provides the measurement model for the sequential Doppler
% observations

% Initializations
idxStatePos = PVTUtils.getStateIndex(PVTUtils.ID_POS);
idxStateVel = PVTUtils.getStateIndex(PVTUtils.ID_VEL);
idxStateClkDrift = PVTUtils.getStateIndex(PVTUtils.ID_CLK_DRIFT);

% Observation
z = hArgs.obs;

% Distance between receiver and satellite. Second term is Sagnac effect.
dist = norm(hArgs.x0(idxStatePos) - hArgs.satPos) + ...
    Constants.OMEGA_E/Constants.CELERITY *          ...
    ( hArgs.satPos(1) * hArgs.x0(idxStatePos(2)) -  ...
    hArgs.satPos(2) * hArgs.x0(idxStatePos(1)) );

% Unit vector from user to satellite
vUS = unitVector(hArgs.satPos - hArgs.x0(idxStatePos));

% Observation estimation
y = dot(hArgs.satVel - hArgs.x0(idxStateVel), vUS) + ...    % Radial velocity from user to sat
    hArgs.x0(idxStateClkDrift) -                     ...    % Receiver clock drift
    Constants.CELERITY * hArgs.satClkDrift;                 % Satellite clock drift

% Jacobian matrix
H = zeros(1, PVTUtils.getNumStates);
H(idxStateVel) = (hArgs.x0(idxStatePos) - hArgs.satPos)/dist;
H(idxStateClkDrift) = 1;

% Measurement covariance matrix
R = Config.COV_FACTOR_D * computeMeasCovariance(hArgs.satElDeg, ...
    hArgs.sigmaObs, Config.SIGMA_D_MPS, hArgs.obsConst);
end %end of function hDopplerObs
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function xTotal = updateTotalState(x, statPos)
idxStatePos = PVTUtils.getStateIndex(PVTUtils.ID_POS);
xTotal = x;
xTotal(idxStatePos) = xTotal(idxStatePos) + statPos;
end %end of function updateTotalState
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
