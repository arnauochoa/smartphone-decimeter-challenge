function [result] = navigate(phoneRnx, imuMeas, nav, osrRnx, ref)
%NAVIGATE Summary of this function goes here
%   Detailed explanation goes here

%% Initializations
config = Config.getInstance;
idxStatePos = PVTUtils.getStateIndex(PVTUtils.ID_POS);
% idxStateVel = PVTUtils.getStateIndex(PVTUtils.ID_VEL);
nStates = PVTUtils.getNumStates();
nSatellites = PVTUtils.getNumSatelliteIndices();
gnssEpochs = unique(phoneRnx.utcSeconds);
nGnssEpochs = length(gnssEpochs);
% EKF:
esekf = EKF.build(uint16(nStates));
if ~config.OUTLIER_REJECTION, esekf.probabilityOfFalseOutlierRejection = 0; end

%% Obtain first position
[x0, esekf.P, esekf.tx] = getFirstPosition(phoneRnx, nav);
esekf.x = zeros(PVTUtils.getNumStates, 1);
esekf.x(idxStatePos) = x0(idxStatePos) - osrRnx.statPos;
% Loop variables
% thisUtcSeconds -> time ref for both IMU and GNSS.
thisUtcSeconds = esekf.tx; % First time is from the first GNSS estimation
idxEst = 1;
result.xEst(:, 1) = x0; % TODO change to error state, prealocate if num pos is known (= groundtruth?)
result.sigmaHist = zeros(nStates, 1);

% First gnss observations is the same as for the first approx position
[phoneGnss, osrGnss] = getNextGnss(thisUtcSeconds, phoneRnx, osrRnx, 'this');
hasEnded = isempty(phoneGnss); % TODO check imu

% TODO: debugging variables, check which need to be kept
result.prInnovations = nan(nSatellites, nGnssEpochs);
result.prInnovationCovariances = nan(nSatellites, nGnssEpochs);
result.prRejectedHist = zeros(1, nGnssEpochs);

while ~hasEnded % while there are more observations/measurements
%     idxRef = find(ref.utcSeconds > thisUtcSeconds, 1, 'first'); % TODO remove
    % First iteration: x0 is result from LS
    if idxEst == 1,     x0 = result.xEst(:, 1);
    else,               x0 = result.xEst(:, idxEst-1); end
    
    thisUtcSeconds = phoneGnss.utcSeconds; % TODO check imu time

    % Get states of satellites selected in Config
    [satPos, satClkBias, satClkDrift, satVel] = ...
        compute_satellite_state_all(phoneGnss.tow, phoneGnss.obs, nav, config.CONSTELLATIONS);

    % Remove invalid observations (no ephem, elevation mask)
    [phoneGnss.obs, satPos, ~, ~, ~] = ...
        filterObs(phoneGnss.obs, satPos, satClkBias, satClkDrift, satVel, x0(idxStatePos));

    % Obtain satellite elevations
    [~, satElDeg, ~] = getSatAzEl(satPos, x0(idxStatePos));

    if isempty(phoneGnss.obs)
        warning('TOW = %d - Not enough observations to estimate a potition. Propagating state.', phoneGnss.tow);
        % Initial estimate for the transition model
        fArgs.x0 = x0;
        esekf = EKF.propagateState(esekf, thisUtcSeconds, @fTransition, fArgs);
    else
%         refPos = Lla2Xyz(ref.posLla(idxRef, :))'; % TODO: remove
        doubleDifferences = computeDoubleDifferences(osrGnss, phoneGnss, satPos, satElDeg);

        % Sequentally update with all observations
        for iObs = 1:length(doubleDifferences)
            idxSat = PVTUtils.getSatelliteIndex(doubleDifferences(iObs).varSatPrn, ...
                doubleDifferences(iObs).constel);

            % Pack arguments that are common for all observations
            fArgs.x0 = x0;
            hArgs.x0 = x0;
            hArgs.statPos = osrRnx.statPos;
            hArgs.obsConst = doubleDifferences(iObs).constel;
            hArgs.pivSatPos = doubleDifferences(iObs).pivSatPos;
            hArgs.varSatPos = doubleDifferences(iObs).varSatPos;
            hArgs.satElDeg = [doubleDifferences(iObs).pivSatElDeg
                doubleDifferences(iObs).varSatElDeg];

            % Pack code DD observation
            hArgs.obs = doubleDifferences(iObs).C;
            hArgs.sigmaObs = [doubleDifferences(iObs).pivSatSigmaC
                doubleDifferences(iObs).varSatSigmaC];

            % Label to show on console when outliers are detected
            label = sprintf('Code (%c%d-%c%d, f = %g)', ...
                doubleDifferences(iObs).constel,        ...
                doubleDifferences(iObs).pivSatPrn,      ...
                doubleDifferences(iObs).constel,        ...
                doubleDifferences(iObs).varSatPrn,      ...
                doubleDifferences(iObs).freqHz);
            % Process code observation
            [esekf, innovation, innovationCovariance, rejected, ~, ~] = ...
                EKF.processObservation(esekf, thisUtcSeconds, ...
                @fTransition, fArgs, ...
                @hCodeObs, hArgs, ...
                label);

            result.prInnovations(idxSat, idxEst) = innovation;
            result.prInnovationCovariances(idxSat, idxEst) = innovationCovariance;
            result.prRejectedHist(idxEst) = result.prRejectedHist(idxEst) + rejected;

            % TODO perform correction when using error-state
            x0(idxStatePos) = osrRnx.statPos + esekf.x(idxStatePos);
%                 x0(idxStateVel) = esekf.x(idxStateVel);
        end
            % Percentage of rejected code observations
            result.prRejectedHist(idxEst) = 100*result.prRejectedHist(idxEst) / length(doubleDifferences);
    end
    
    result.utcSeconds(idxEst) = thisUtcSeconds;
    result.gpsWeekN(idxEst) = phoneGnss.weekN;
    result.gpsTow(idxEst) = phoneGnss.tow;
    
    result.xEst(:, idxEst) = x0;
    result.sigmaHist(:, idxEst) = sqrt(diag(esekf.P));
    idxEst = idxEst + 1;
    
    % Check if there are more measurements/observations
    [phoneGnss, osrGnss] = getNextGnss(thisUtcSeconds, phoneRnx, osrRnx);
    hasEnded = isempty(phoneGnss); % TODO check imu
end

end

% State transition function
function [x2, F, Q] = fTransition(x1, t1, t2, fArgs)
% Initializations
config = Config.getInstance;
idxStatePos = PVTUtils.getStateIndex(PVTUtils.ID_POS);

F = eye(PVTUtils.getNumStates);

% State prediction
x2 = F * x1;
% Process noise covariance matrix
Q = zeros(PVTUtils.getNumStates);
Q(idxStatePos, idxStatePos) = config.SIGMA_Q_POS*eye(3);
end %end of function fTransition
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% function [z, y, H, R] = hRefObs(~, hArgs)
% 
% idxStatePos = PVTUtils.getStateIndex(PVTUtils.ID_POS);
% 
% z = hArgs.obs;
% 
% y = hArgs.x0(idxStatePos) - osrRnx.statPos;
% 
% % Jacobian matrix
% H = zeros(3, PVTUtils.getNumStates);
% H(idxStatePos,idxStatePos) = eye(3);
% 
% R = diag(hArgs.sigmaObs);
% end


function [z, y, H, R] = hCodeObs(~, hArgs)
% HCODEOBS provides the measurement model for the sequential code
% observations

% Initializations
config = Config.getInstance;
idxStatePos = PVTUtils.getStateIndex(PVTUtils.ID_POS);
rxPos = hArgs.x0(idxStatePos);

% Observation
z = hArgs.obs;

% Observation estimation
y = norm(hArgs.statPos - hArgs.pivSatPos) - ... % |stat - sat1|
    norm(rxPos - hArgs.pivSatPos) -                  ... % |user - sat1|
    norm(hArgs.statPos - hArgs.varSatPos) + ... % |stat - sat2|
    norm(rxPos - hArgs.varSatPos);                       % |user - sat2|

% Difference between LOS vectors of satellites towards receiver
pivSatLosVec = unitVector(hArgs.statPos - hArgs.pivSatPos);
varSatLosVec = unitVector(hArgs.statPos - hArgs.varSatPos);
ddLosVec = varSatLosVec - pivSatLosVec;

% figure(1); clf; hold on
% plot3(osrRnx.statPos(1), osrRnx.statPos(2), osrRnx.statPos(3), '^')
% plot3(rxPos(1), rxPos(2), rxPos(3), 'o')
% plot3(hArgs.pivSatPos(1), hArgs.pivSatPos(2), hArgs.pivSatPos(3), 'v')
% plot3(hArgs.varSatPos(1), hArgs.varSatPos(2), hArgs.varSatPos(3), 's')
% plot3([hArgs.pivSatPos(1), hArgs.pivSatPos(1) + 1e6*pivSatLosVec(1)], [hArgs.pivSatPos(2), hArgs.pivSatPos(2) + 1e6*pivSatLosVec(2)], [hArgs.pivSatPos(3), hArgs.pivSatPos(3) + 1e6*pivSatLosVec(3)], 'LineWidth',1)
% plot3([hArgs.varSatPos(1), hArgs.varSatPos(1) + 1e6*varSatLosVec(1)], [hArgs.varSatPos(2), hArgs.varSatPos(2) + 1e6*varSatLosVec(2)], [hArgs.varSatPos(3), hArgs.varSatPos(3) + 1e6*varSatLosVec(3)], 'LineWidth',1)
% plot3([hArgs.varSatPos(1), hArgs.varSatPos(1) + 1e6*ddLosVec(1)], [hArgs.varSatPos(2), hArgs.varSatPos(2) + 1e6*ddLosVec(2)], [hArgs.varSatPos(3), hArgs.varSatPos(3) + 1e6*ddLosVec(3)], 'LineWidth',1)

% Jacobian matrix
H = zeros(1, PVTUtils.getNumStates);
H(idxStatePos) = ddLosVec;

% Measurement covariance matrix, consider DD sigmas
R = config.COV_FACTOR_C * computeRtkMeasCovariance(hArgs.satElDeg, ...
    hArgs.sigmaObs, config.SIGMA_PR_M, hArgs.obsConst);
end %end of function hCodeObs
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
