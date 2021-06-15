function [result] = navigate(phoneRnx, imuMeas, nav, osrRnx, ref)
%NAVIGATE Summary of this function goes here
%   Detailed explanation goes here

%% Initializations
config = Config.getInstance;
idxStatePos = PVTUtils.getStateIndex(PVTUtils.ID_POS);
idxStateAllSdAmb = PVTUtils.getStateIndex(PVTUtils.ID_SD_AMBIGUITY);
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
[x0, ekf.P, ekf.tx, x0WLS] = getFirstPosition(phoneRnx, nav);
% (TODO remove) >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
%  Use ref position as first position
% idxRef = find(ref.utcSeconds > ekf.tx, 1, 'first');
% [x, y, z] = geodetic2ecef(wgs84Ellipsoid, ref.posLla(idxRef, 1), ref.posLla(idxRef, 2), ref.posLla(idxRef, 3));
% x0(idxStatePos) = [x y z]';
% <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
ekf.x = zeros(PVTUtils.getNumStates, 1);
ekf.x(idxStatePos) = x0(idxStatePos) - osrRnx.statPos;

% Loop variables
% thisUtcSeconds -> time ref for both IMU and GNSS.
thisUtcSeconds = ekf.tx; % First time is from the first GNSS estimation
idxEst = 1;
result.xRTK(:, 1) = x0; % TODO change to error state, prealocate if num pos is known (= groundtruth?)
result.PRTK(:, :, 1) = zeros(nStates, nStates, 1);

% WLS solution
nStatesWLS = 4 + PVTUtils.getNumConstellations - 1;
result.xWLS(:, 1) = x0WLS; % TODO prealocate if num pos is known (= groundtruth?)
result.PWLS(:, :, 1) = zeros(nStatesWLS, nStatesWLS, 1);

%% Evaluate all trajectory
% First gnss observations is the same as for the first approx position
[phoneEpoch, osrEpoch] = getNextGnss(thisUtcSeconds, phoneRnx, osrRnx, 'this');
hasEnded = isempty(phoneEpoch); % TODO check imu

while ~hasEnded % while there are more observations/measurements
    % First iteration: x0 from getFirstPosition (WLS estimation)
    % Following iterations: x0 from previous iteration
    if idxEst > 1
        x0 = result.xRTK(:, idxEst-1); 
        x0WLS = result.xWLS(:, idxEst-1); 
    end
    
    thisUtcSeconds = phoneEpoch.utcSeconds; % TODO check imu time
    
    % Get states of satellites selected in Config
    [satPos, satClkBias, satClkDrift, satVel] = ...
        compute_satellite_state_all(phoneEpoch.tow, phoneEpoch.obs, nav, config.CONSTELLATIONS);
    
    % Remove invalid observations (no ephem, elevation mask)
    [phoneEpoch.obs, sat] = ...
        filterObs(phoneEpoch.obs, satPos, satClkBias, satClkDrift, satVel, x0(idxStatePos));
    clear satPos satClkBias satClkDrift satVel
    
    % Obtain satellite elevations
    [sat.azDeg, sat.elDeg, ~] = getSatAzEl(sat.pos, x0(idxStatePos));
    
    %% Ref observation
% (TODO remove) >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
%     idxRef = find(ref.utcSeconds > thisUtcSeconds, 1, 'first');
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
% <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    
    result.utcSeconds(idxEst) = thisUtcSeconds;
    result.gpsWeekN(idxEst) = phoneEpoch.weekN;
    result.gpsTow(idxEst) = phoneEpoch.tow;
    if isempty(phoneEpoch.obs)
        fprintf(2, 'TOW = %d - Not enough observations to estimate a position. Propagating state.\n', phoneEpoch.tow);
        % Initial estimate for the transition model
        fArgs.x0 = x0;
        ekf = EKF.propagateState(ekf, thisUtcSeconds, @fTransition, fArgs);
        result.xWLS(:, idxEst) = result.xWLS(:, idxEst-1);
        result.PWLS(:, :, idxEst) = result.PWLS(:, :, idxEst-1);
        % (TODO remove) >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
        % Skip epochs with missing obs
%         fprintf(2, 'TOW = %d - Not enough observations to estimate a potition. Skipping epoch.\n', phoneGnss.tow);
%         [phoneGnss, osrGnss] = getNextGnss(thisUtcSeconds, phoneRnx, osrRnx);
%         hasEnded = isempty(phoneGnss); % TODO check imu
%         continue
        % <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    else
        if length([phoneEpoch.obs(:).C]) >= nStatesWLS
            %% WLS estimation
            [obsConstel, idxObsConst] = intersect(Config.CONSTELLATIONS, unique([phoneEpoch.obs.constellation]), 'stable');
            idxComputedStates = [1:4, 4+idxObsConst(2:end)'-1]; % pos, clock, inter-const bias
            R = computeMeasCovariance(sat.elDeg, [phoneEpoch.obs(:).C_sigma], ...
                Config.SIGMA_D_MPS, [phoneEpoch.obs(:).constellation]);
            [xWLS, ~, PWLS, ~, ~] = compute_spp_wls([phoneEpoch.obs(:).C]', ...
                [phoneEpoch.obs(:).constellation], sat.pos, sat.clkBias, x0WLS(idxComputedStates), R, obsConstel);
            % Save all states including missing constellations
            result.xWLS(:, idxEst) = zeros(nStatesWLS, 1);
            result.PWLS(:, :, idxEst) = zeros(nStatesWLS, nStatesWLS, 1);
            result.xWLS(idxComputedStates, idxEst) = xWLS;
            result.PWLS(idxComputedStates, idxComputedStates, idxEst) = PWLS;
            clear xWLS PWLS idxObsConst
        else
            result.xWLS(:, idxEst) = result.xWLS(:, idxEst-1);
            result.PWLS(:, :, idxEst) = result.PWLS(:, :, idxEst-1);
        end
        
        %% RTK estimation
        doubleDifferences = computeDoubleDifferences(osrEpoch, phoneEpoch, sat.pos, sat.elDeg);
        [x0, ekf, result] = updateWithDD(x0, ekf, thisUtcSeconds, idxEst, osrRnx.statPos, doubleDifferences, result);
        
        [x0, ekf, result] = updateWithDoppler(x0, ekf, thisUtcSeconds, idxEst, osrRnx.statPos, phoneEpoch, sat, result);
    end
    
    result.xRTK(:, idxEst) = x0;
    result.PRTK(:, :, idxEst) = ekf.P;
    idxEst = idxEst + 1;
    
    % Check if there are more measurements/observations
    [phoneEpoch, osrEpoch] = getNextGnss(thisUtcSeconds, phoneRnx, osrRnx);
    hasEnded = isempty(phoneEpoch); % TODO check imu
end

end

function [z, y, H, R] = hRefObs(~, hArgs)
idxStatePos = PVTUtils.getStateIndex(PVTUtils.ID_POS);

z = hArgs.obs;
y = hArgs.x0(idxStatePos) - hArgs.statPos;
% Jacobian matrix
H = zeros(3, PVTUtils.getNumStates);
H(idxStatePos,idxStatePos) = eye(3);
R = diag(hArgs.sigmaObs);
end