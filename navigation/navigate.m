function [result] = navigate(phoneRnx, imuMeas, nav, osrRnx, ref)
%NAVIGATE Summary of this function goes here
%   Detailed explanation goes here

%% Initializations
config = Config.getInstance;
idxStatePos = PVTUtils.getStateIndex(PVTUtils.ID_POS);
idxStateVel = PVTUtils.getStateIndex(PVTUtils.ID_VEL);
idxStateAllSdAmb = PVTUtils.getStateIndex(PVTUtils.ID_SD_AMBIGUITY);
% idxStateVel = PVTUtils.getStateIndex(PVTUtils.ID_VEL);
nStates = PVTUtils.getNumStates();
nSatFreqs = PVTUtils.getNumSatFreqIndices();
phoneEpochs = unique(phoneRnx.utcSeconds);
nGnssEpochs = length(phoneEpochs);
% EKF:
ekf = EKF.build(uint16(nStates), config.P_FALSE_OUTLIER_REJECT);

nStatesWLS = 4 + PVTUtils.getNumConstellations - 1;
% TODO: debugging variables, check which need to be kept
result.xWLS = nan(nStatesWLS, nGnssEpochs);
result.xRTK = nan(nStates, nGnssEpochs);
result.utcSeconds = nan(1, nGnssEpochs);
result.prInnovations = nan(nSatFreqs, nGnssEpochs);
result.prInnovationCovariances = nan(nSatFreqs, nGnssEpochs);
result.prRejectedHist = zeros(1, nGnssEpochs);
result.prNumDD = nan(1, nGnssEpochs);
result.phsInnovations = nan(nSatFreqs, nGnssEpochs);
result.phsInnovationCovariances = nan(nSatFreqs, nGnssEpochs);
result.phsRejectedHist = zeros(1, nGnssEpochs);
result.phsNumDD = nan(1, nGnssEpochs);
result.dopInnovations = nan(nSatFreqs, nGnssEpochs);
result.dopInnovationCovariances = nan(nSatFreqs, nGnssEpochs);
result.dopRejectedHist = zeros(1, nGnssEpochs);
result.dopNumDD = nan(1, nGnssEpochs);

%% Obtain first position
[x0, ekf.P, ekf.tx, x0WLS] = getFirstPosition(phoneRnx, nav);
% (TODO remove) >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
% % Use ref position as first position
% idxRef = find(ref.utcSeconds > ekf.tx, 1, 'first');
% x0(idxStatePos) = geodetic2ecefVector(ref.posLla(idxRef, :))';
% <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
ekf.x = zeros(PVTUtils.getNumStates, 1);
ekf.x(idxStatePos) = x0(idxStatePos) - osrRnx.statPos;

% Loop variables
% thisUtcSeconds -> time ref for both IMU and GNSS.
thisUtcSeconds = ekf.tx; % First time is from the first GNSS estimation
idxEst = find(phoneEpochs == ekf.tx);
result.xRTK(:, idxEst) = x0; % TODO change to error state, prealocate if num pos is known (= groundtruth?)
PRTK(:, :, idxEst) = zeros(nStates, nStates, 1);

% WLS solution
result.xWLS(:, idxEst) = x0WLS; % TODO prealocate if num pos is known (= groundtruth?)
PWLSHist(:, :, idxEst) = zeros(nStatesWLS, nStatesWLS, 1);

% If some of first epochs have been skipped, set their result as the first
% valid result
if idxEst > 1
    nSkipped = idxEst-1;
    result.xRTK(:, 1:nSkipped) = repmat(x0, 1, nSkipped);
    result.xWLS(:, 1:nSkipped) = repmat(x0WLS, 1, nSkipped);
    result.utcSeconds(1:nSkipped) = phoneEpochs(1:nSkipped);
end

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
    if config.USE_REF_POS
        assert(strcmp(config.DATASET_TYPE, 'train'), 'Reference can only be used in train datasets');
        [x0, ekf, result, thisUtcSeconds] = updateWithRefPos(x0, ekf, thisUtcSeconds, ref, osrRnx, result);
    end
    % <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    
    result.utcSeconds(idxEst) = thisUtcSeconds;
    result.gpsWeekN(idxEst) = phoneEpoch.weekN;
    result.gpsTow(idxEst) = phoneEpoch.tow;
    if isempty(phoneEpoch.obs)
        fprintf(2, 'TOW = %d - Not enough observations to estimate a position. Propagating state.\n', phoneEpoch.tow);
        % Initial estimate for the transition model
        fArgs.x0 = x0;
        ekf = EKF.propagateState(ekf, thisUtcSeconds, @fTransition, fArgs);
        % Update total-state with absolute position
        x0 = updateTotalState(ekf.x, osrRnx.statPos);

        result.xWLS(:, idxEst) = x0WLS;
        PWLSHist(:, :, idxEst) = PWLSHist(:, :, idxEst-1);

%         if config.USE_REF_POS
%             [x0, ekf, result, thisUtcSeconds] = updateWithRefPos(x0, ekf, thisUtcSeconds, ref, osrRnx, result);
%         end

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
            PWLSHist(:, :, idxEst) = zeros(nStatesWLS, nStatesWLS, 1);
            result.xWLS(idxComputedStates, idxEst) = xWLS;
            PWLSHist(idxComputedStates, idxComputedStates, idxEst) = PWLS;
            clear xWLS PWLS idxObsConst
        else
            result.xWLS(:, idxEst) = result.xWLS(:, idxEst-1);
            PWLSHist(:, :, idxEst) = PWLSHist(:, :, idxEst-1);
        end
        
        %% RTK estimation
        doubleDifferences = computeDoubleDifferences(osrEpoch, phoneEpoch, sat.pos, sat.elDeg);
        if ~isempty(doubleDifferences)
            [x0, ekf, result] = updateWithDD(x0, ekf, thisUtcSeconds, idxEst, osrRnx.statPos, doubleDifferences, result);
        end
        if config.USE_DOPPLER
            [x0, ekf, result] = updateWithDoppler(x0, ekf, thisUtcSeconds, idxEst, osrRnx.statPos, phoneEpoch, sat, result);
        end
    end
    
    result.xRTK(:, idxEst) = x0;
    PRTK(:, :, idxEst) = ekf.P;
    idxEst = idxEst + 1;
    
    % Check if there are more measurements/observations
    [phoneEpoch, osrEpoch] = getNextGnss(thisUtcSeconds, phoneRnx, osrRnx);
    hasEnded = isempty(phoneEpoch); % TODO check imu
end

nEpochs = size(result.xRTK, 2);
estPosXyz = result.xRTK(idxStatePos, :)';
result.posStdNed = nan(nEpochs, 3);
result.velNed = nan(nEpochs, 3);
result.velStdNed = nan(nEpochs, 3);
for iEpoch = 1:nEpochs
    Rn2e = compute_Rn2e(estPosXyz(iEpoch, 1), estPosXyz(iEpoch, 2), estPosXyz(iEpoch, 3));
    % Position STD in NED
    Ppos = PRTK(idxStatePos, idxStatePos, iEpoch);
    result.posStdNed(iEpoch, :) = sqrt(diag(Rn2e' * Ppos * Rn2e)');
    % Velocity to NED
    result.velNed(iEpoch, :) = Rn2e' * result.xRTK(idxStateVel, iEpoch);
    % Velocity STD in NED
    Pvel = PRTK(idxStateVel, idxStateVel, iEpoch);
    result.velStdNed(iEpoch, :) = sqrt(diag(Rn2e' * Pvel * Rn2e)');
end

end