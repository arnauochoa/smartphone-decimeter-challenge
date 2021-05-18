function [xEstHist, prInnovations, prInnovationCovariances, dopInnovations, ...
    dopInnovationCovariances, utcSecondsHist, sigmaHist, prRejectedHist, dopRejectedHist] = ...
    navigate(rxGnssRnx, imuMeas, nav, osrGnssRnx, ref)
%NAVIGATE Summary of this function goes here
%   Detailed explanation goes here

%% Initializations
idxStatePos = PVTUtils.getStateIndex(PVTUtils.ID_POS);
idxStateVel = PVTUtils.getStateIndex(PVTUtils.ID_VEL);
nStates = PVTUtils.getNumStates();
nSatellites = PVTUtils.getNumSatelliteIndices();
gnssEpochs = unique(rxGnssRnx.utcSeconds);
nGnssEpochs = length(gnssEpochs);
% EKF:
esekf = EKF.build(uint16(nStates));
if ~Config.OUTLIER_REJECTION, esekf.probabilityOfFalseOutlierRejection = 0; end

%% Obtain first position
[esekf.x, esekf.P, esekf.tx] = getFirstPosition(rxGnssRnx, nav);

% Loop variables
% thisUtcSeconds -> time ref for both IMU and GNSS.
thisUtcSeconds = esekf.tx; % First time is from the first GNSS estimation
idxEst = 1;
xEstHist = esekf.x; % TODO change to error state, prealocate if num pos is known (= groundtruth?)
sigmaHist = zeros(nStates, 1);

% First gnss observations is the same as for the first approx position
[gnssRx, gnssOsr] = getNextGnss(thisUtcSeconds, rxGnssRnx, osrGnssRnx, 'this');
hasEnded = isempty(gnssRx); % TODO check imu

% TODO: debugging variables, check which need to be kept
prInnovations = nan(nSatellites, nGnssEpochs);
prInnovationCovariances = nan(nSatellites, nGnssEpochs);
dopInnovations = nan(nSatellites, nGnssEpochs);
dopInnovationCovariances = nan(nSatellites, nGnssEpochs);
prRejectedHist = zeros(1, nGnssEpochs);
dopRejectedHist = zeros(1, nGnssEpochs);

while ~hasEnded % while there are more observations/measurements
    
    % First iteration: x0 is result from LS
    if idxEst == 1,     x0 = xEstHist(:, 1);
    else,               x0 = xEstHist(:, idxEst-1); end
    % Initial estimate for transition and measurement models
    fArgs.x0 = x0;
    hArgs.x0 = x0;
    
    thisUtcSeconds = gnssRx.utcSeconds; % TODO check imu time
    
    % Get states of satellites selected in Config
    [satPos, satClkBias, satClkDrift, satVel] = ...
        compute_satellite_state_all(gnssRx.tow, gnssRx.obs, nav, Config.CONSTELLATIONS);
    
    % Remove invalid observations (no ephem, elevation mask)
    [gnssRx.obs, satPos, ~, ~, ~] = ...
        filterObs(gnssRx.obs, satPos, satClkBias, satClkDrift, satVel, x0(1:3));
    
    if length([gnssRx.obs(:).C]) < 4 + PVTUtils.getNumFrequencies + PVTUtils.getNumConstellations
        warning('TOW = %d - Not enough observations to estimate a potition. Propagating state.', gnssRx.tow);
        esekf = EKF.propagateState(esekf, thisUtcSeconds, @fTransition, fArgs);
    else
        % TODO: compute double differences
        doubleDifferences = computeDoubleDifferences(gnssRx, gnssOsr);
        
        
        % Sequentally update with all observations
        for iObs = 1:length(doubleDifferences)
            idxSat = PVTUtils.getSatelliteIndex(doubleDifferences(iObs).prns(2), doubleDifferences(iObs).constel);
            % Pack arguments that are common for all observations
            hArgs.obsConst = doubleDifferences(iObs).constel;
            hArgs.satPos = getSatPositions(gnssRx.obs, satPos, ...
                hArgs.obsConst, doubleDifferences(iObs).prns);
            [~, hArgs.satElev(1), ~] = getSatAzEl(hArgs.satPos(:, 1), hArgs.x0(idxStatePos));
            [~, hArgs.satElev(2), ~] = getSatAzEl(hArgs.satPos(:, 2), hArgs.x0(idxStatePos));
            
            % Pack code DD observation
            hArgs.obs = doubleDifferences(iObs).C;
            hArgs.sigmaObs = doubleDifferences(iObs).sigmaC;
            
            % Label to show on console when outliers are detected
            label = sprintf('Code (%c%d-%c%d, f = %g)', ...
                doubleDifferences(iObs).constel, ...
                doubleDifferences(iObs).prns(1), ...
                doubleDifferences(iObs).constel, ...
                doubleDifferences(iObs).prns(2), ...
                doubleDifferences(iObs).freqHz);
            % Process code observation
            [esekf, innovation, innovationCovariance, rejected, ~, ~] = ...
                EKF.processObservation(esekf, thisUtcSeconds, ...
                @fTransition, fArgs, ...
                @hCodeObs, hArgs, ...
                label);
            
            prInnovations(idxSat, idxEst) = innovation;
            prInnovationCovariances(idxSat, idxEst) = innovationCovariance;
            prRejectedHist(idxEst) = prRejectedHist(idxEst) + rejected;
            
            fArgs.x0(idxStatePos) = Config.STATION_POS_XYZ' - esekf.x(idxStatePos);
            fArgs.x0(idxStateVel) = esekf.x(idxStateVel);
            fArgs.x0 = fArgs.x0;
            hArgs.x0 = fArgs.x0;
        end
    end
    
    utcSecondsHist(idxEst) = thisUtcSeconds;
    
    xEstHist(:, idxEst) = esekf.x; % TODO perform correction when using error-state
    sigmaHist(:, idxEst) = sqrt(diag(esekf.P));
    idxEst = idxEst + 1;
    
    % Check if there are more measurements/observations
    gnssRx = getNextGnss(thisUtcSeconds, rxGnssRnx);
    hasEnded = isempty(gnssRx); % TODO check imu
end

end

% State transition function
function [x2, F, Q] = fTransition(x1, t1, t2, fArgs)
% Initializations
idxStatePos = PVTUtils.getStateIndex(PVTUtils.ID_POS);
idxStateVel = PVTUtils.getStateIndex(PVTUtils.ID_VEL);

dt = t2 - t1;
F = eye(PVTUtils.getNumStates);
% dp/dv
F(idxStatePos, idxStateVel) = dt * eye(3);
% State prediction
x2 = F * x1;
% Process noise covariance matrix - Velocity
rotN2E = compute_Rn2e(fArgs.x0(1), fArgs.x0(2), fArgs.x0(3));
Qvel = dt * rotN2E * diag(Config.SIGMA_VEL_NED.^2) * rotN2E';
% Process noise covariance matrix
Q = zeros(PVTUtils.getNumStates);
Q(idxStateVel, idxStateVel) = Qvel;
end %end of function fTransition
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [z, y, H, R] = hCodeObs(~, hArgs)
% HCODEOBS provides the measurement model for the sequential code
% observations

% Initializations
idxStatePos = PVTUtils.getStateIndex(PVTUtils.ID_POS);
rxPos = hArgs.x0(idxStatePos);

% Observation
z = hArgs.obs;

% Observation estimation: Distance between receiver and OSR station
y = norm(rxPos - Config.STATION_POS_XYZ');

% Difference between LOS vectors of satellites towards
refSatLosVec = rxPos - hArgs.satPos(:, 1);
satLosVec = rxPos - hArgs.satPos(:, 2);
ddLosVec = unitVector(refSatLosVec - satLosVec);

% Jacobian matrix
H = zeros(1, PVTUtils.getNumStates);
H(idxStatePos) = ddLosVec;

% Measurement covariance matrix, consider DD sigmas
R = Config.COV_FACTOR_C * computeRtkMeasCovariance(hArgs.satElev, ...
    hArgs.sigmaObs, Config.SIGMA_PR_M, hArgs.obsConst);
end %end of function hCodeObs
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
