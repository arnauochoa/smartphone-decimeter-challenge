function [result] = navigate(phones, osr, nav)
%NAVIGATE Runs the KF over all the trace
%   Runs the KF over all the trace. Returns a structure containing the
%   estimated positions and other information (e.g. covariances)
%
%       [result] = NAVIGATE(phones, osr, nav)
%
% Input:    phones  =   Array of structures. Contains the measurements (INS
%                       and GNSS), groundtruth (if any), and information of
%                       each phone
%           osr     =   Structure containing the position of the reference
%                       station and the observations.
%           nav     =   Structure containing the navigation data.
%
% Output:   result  =   Structure containing timeline, estimated state,
%                       covariances, rejections, and other information 
%
%% Initializations
config = Config.getInstance;
pb = CmdLineProgressBar(sprintf('Evaluating %s %s... ', config.campaignName, strjoin(config.phoneNames, '+')));
nPhones = length(config.phoneNames);
idxStatePos = PVTUtils.getStateIndex(PVTUtils.ID_POS);
idxStateAtt = PVTUtils.getStateIndex(PVTUtils.ID_ATT_XYZ);
idxStateVel = PVTUtils.getStateIndex(PVTUtils.ID_VEL);
% idxStateAllSdAmb = PVTUtils.getStateIndex(PVTUtils.ID_SD_AMBIGUITY, 1:nPhones);     % TODO: change phones index to iPhone
% idxStateVel = PVTUtils.getStateIndex(PVTUtils.ID_VEL);
nStates = PVTUtils.getNumStates();
nSatFreqs = PVTUtils.getNumSatFreqIndices();
epochTimesUtc = getAllPhonesEpochs(phones);
if isinf(config.EPOCHS_TO_RUN), nGnssEpochs = length(epochTimesUtc);
else,                           nGnssEpochs = config.EPOCHS_TO_RUN;     end

fid = fopen('log.txt', 'a+');
fprintf(fid, '\n >>>>>>>>>>>>>>>>>>>>>> %s :\n', config.resFileTimestamp);

% EKF:
ekf = EKF.build(uint16(nStates), config.P_FALSE_OUTLIER_REJECT);
ekf.debugDisplay = config.SHOW_DEBUG_MESSAGES;

nStatesWLS = 4 + PVTUtils.getNumConstellations - 1;
% TODO: debugging variables, check which need to be kept
result.xWLS                     = nan(nStatesWLS, nGnssEpochs);
result.xRTK                     = nan(nStates, nGnssEpochs);
result.utcSeconds               = nan(1, nGnssEpochs);
result.phoneUsed                = nan(1, nGnssEpochs);
result.prInnovations            = nan(nSatFreqs, nGnssEpochs, nPhones);
result.prInnovationCovariances  = nan(nSatFreqs, nGnssEpochs, nPhones);
result.prRejectedHist           = zeros(1, nGnssEpochs);
result.prInvalidHist            = zeros(1, nGnssEpochs);
result.prNumDD                  = zeros(1, nGnssEpochs);
result.phsInnovations           = nan(nSatFreqs, nGnssEpochs, nPhones);
result.phsInnovationCovariances = nan(nSatFreqs, nGnssEpochs, nPhones);
result.phsRejectedHist          = zeros(1, nGnssEpochs);
result.phsInvalidHist           = zeros(1, nGnssEpochs);
result.phsNumDD                 = zeros(1, nGnssEpochs);
result.dopInnovations           = nan(nSatFreqs, nGnssEpochs, nPhones);
result.dopInnovationCovariances = nan(nSatFreqs, nGnssEpochs, nPhones);
result.dopRejectedHist          = zeros(1, nGnssEpochs);
result.dopInvalidHist           = zeros(1, nGnssEpochs);
result.dopNumDD                 = zeros(1, nGnssEpochs);
% Reference position observations
if config.USE_REF_POS
    result.refRejectedHist      = zeros(1, nGnssEpochs);
end
result.posStdNed                = nan(nGnssEpochs, 3);
result.velNed                   = nan(nGnssEpochs, 3);
result.velStdNed                = nan(nGnssEpochs, 3);

%% Obtain first position
[x0, P, ekf.tx, x0WLS, ~]   = getFirstPosition(phones, nav);
ekf.P = P;
% (TODO remove) >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
% % Use ref position as first position
% idxRef = find(phones(phoneInfo.idx).ref.utcSeconds > ekf.tx, 1, 'first');
% x0(idxStatePos) = geodetic2ecefVector(phones(phoneInfo.idx).ref.posLla(idxRef, :))';
% <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
ekf.x = zeros(PVTUtils.getNumStates, 1);
ekf.x(idxStatePos) = x0(idxStatePos) - osr.statPos;

% Loop variables
% thisUtcSeconds -> time ref for both IMU and GNSS.
thisUtcSeconds = ekf.tx; % First time is from the first GNSS estimation
idxEst = find(epochTimesUtc == ekf.tx);
result.xRTK(:, idxEst) = x0; % TODO change to error state, prealocate if num pos is known (= groundtruth?)
PRTK(:, :, idxEst) = zeros(nStates, nStates, 1);

% Progress bar
pb.print(idxEst, nGnssEpochs); % 

% WLS solution
result.xWLS(:, idxEst) = x0WLS; % TODO prealocate if num pos is known (= groundtruth?)
PWLSHist(:, :, idxEst) = zeros(nStatesWLS, nStatesWLS, 1);

% If some of first epochs have been skipped, set their result as the first
% valid result
if idxEst > 1
    nSkipped = idxEst-1;
    result.xRTK(:, 1:nSkipped) = repmat(x0, 1, nSkipped);
    result.xWLS(:, 1:nSkipped) = repmat(x0WLS, 1, nSkipped);
    result.utcSeconds(1:nSkipped) = epochTimesUtc(1:nSkipped);
end

%% Evaluate all trajectory
% First gnss observations is the same as for the first approx position
[gnssEpoch, osrEpoch, phoneInfo] = getNextGnss(thisUtcSeconds, phones, osr, 'this');
hasEnded = isempty(gnssEpoch); % TODO check imu

while ~hasEnded % while there are more observations/measurements
    % First iteration: x0 from getFirstPosition (WLS estimation)
    % Following iterations: x0 from previous iteration
    if idxEst > 1
        x0 = result.xRTK(:, idxEst-1);
        x0WLS = result.xWLS(:, idxEst-1);
    end
    thisUtcSeconds = gnssEpoch.utcSeconds; % TODO check imu time
    
    % Get states of satellites selected in Config
    [satPos, satClkBias, satClkDrift, satVel] = ...
        compute_satellite_state_all(gnssEpoch.tow, gnssEpoch.obs, nav, config.CONSTELLATIONS);
    
    % Remove invalid observations (no ephem, elevation mask)
    [gnssEpoch.obs, sat] = ...
        filterObs(gnssEpoch.obs, satPos, satClkBias, satClkDrift, satVel, x0(idxStatePos));
    clear satPos satClkBias satClkDrift satVel
    
    % Obtain satellite elevations
    [sat.azDeg, sat.elDeg, ~] = getSatAzEl(sat.pos, x0(idxStatePos));
    
    %% Ref observation
    % (TODO remove) >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    if config.USE_REF_POS
        assert(strcmp(config.DATASET_TYPE, 'train'), 'Reference can only be used in train datasets');
        [x0, ekf, result, thisUtcSeconds] = ...
            updateWithRefPos(x0, ekf, thisUtcSeconds, idxEst, phones(phoneInfo.idx).ref, phones(phoneInfo.idx).osr, result);
    end
    % <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    
    result.utcSeconds(idxEst) = thisUtcSeconds;
    result.gpsWeekN(idxEst) = gnssEpoch.weekN;
    result.gpsTow(idxEst) = gnssEpoch.tow;
    if isempty(gnssEpoch.obs)
        fprintf(2, 'TOW = %d - Not enough observations to estimate a position. Propagating state.\n', gnssEpoch.tow);
        % Initial estimate for the transition model
        fArgs.x0 = x0;
        ekf = EKF.propagateState(ekf, thisUtcSeconds, @fTransition, fArgs);
        % Update total-state with absolute position
        x0 = updateTotalState(ekf.x, osr.statPos);

        result.xWLS(:, idxEst) = x0WLS;
        PWLSHist(:, :, idxEst) = PWLSHist(:, :, idxEst-1);

%         if config.USE_REF_POS
%             [x0, ekf, result, thisUtcSeconds] = ...
%                 updateWithRefPos(x0, ekf, thisUtcSeconds, phones(phoneInfo.idx).ref, phones(phoneInfo.idx).osr, result);
%         end

        % (TODO remove) >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
        % Skip epochs with missing obs
        %         fprintf(2, 'TOW = %d - Not enough observations to estimate a potition. Skipping epoch.\n', phoneGnss.tow);
        %         [phoneGnss, osrGnss] = getNextGnss(thisUtcSeconds, phones(1).gnss, phones(1).osr);% TODO: change phones index to iPhone
        %         hasEnded = isempty(phoneGnss); % TODO check imu
        %         continue
        % <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    else
        if length([gnssEpoch.obs(:).C]) >= nStatesWLS && phoneInfo.idx == 1
            %% WLS estimation
            [obsConstel, idxObsConst] = intersect(Config.CONSTELLATIONS, unique([gnssEpoch.obs.constellation]), 'stable');
            idxComputedStates = [1:4, 4+idxObsConst(2:end)'-1]; % pos, clock, inter-const bias
            R = computeMeasCovariance(sat.elDeg, [gnssEpoch.obs(:).C_sigma], ...
                Config.SIGMA_D_MPS, [gnssEpoch.obs(:).constellation]);
            [xWLS, ~, PWLS, ~, ~] = compute_spp_wls([gnssEpoch.obs(:).C]', ...
                [gnssEpoch.obs(:).constellation], sat.pos, sat.clkBias, x0WLS(idxComputedStates), R, obsConstel);
            % Save all states including missing constellations
            result.xWLS(:, idxEst) = zeros(nStatesWLS, 1);
            PWLSHist(:, :, idxEst) = zeros(nStatesWLS, nStatesWLS, 1);
            result.xWLS(idxComputedStates, idxEst) = xWLS;
            PWLSHist(idxComputedStates, idxComputedStates, idxEst) = PWLS;
            clear xWLS PWLS idxObsConst
        else
            if idxEst > 1
                result.xWLS(:, idxEst) = result.xWLS(:, idxEst-1);
                PWLSHist(:, :, idxEst) = PWLSHist(:, :, idxEst-1);
            end
        end
        
        %% RTK estimation
        doubleDifferences = computeDoubleDifferences(osrEpoch, gnssEpoch, sat.pos, sat.elDeg);
        if ~isempty(doubleDifferences) %&& phoneInfo.idx == 1                % TODO consider all phones
            [x0, ekf, result] = updateWithDD(phoneInfo, x0, ekf, thisUtcSeconds, idxEst, osr.statPos, doubleDifferences, result);
        end
        if config.USE_DOPPLER
            [x0, ekf, result] = updateWithDoppler(phoneInfo, x0, ekf, thisUtcSeconds, idxEst, osr.statPos, gnssEpoch, sat, result);
        end
    end
    
    if any(diag(ekf.P) < 0)
        error('Covariance should not be negative');
    end
    
    %% Save output
    result.xRTK(:, idxEst) = x0;
    estPosEcef  = x0(idxStatePos);
    Rn2e = compute_Rn2e(estPosEcef(1), estPosEcef(2), estPosEcef(3));
    % Position STD in NED
    Ppos = ekf.P(idxStatePos, idxStatePos);
    result.posStdNed(idxEst, :) = sqrt(diag(Rn2e' * Ppos * Rn2e)');
    % Velocity to NED
    result.velNed(idxEst, :) = Rn2e' * result.xRTK(idxStateVel, idxEst);
    % Velocity STD in NED
    Pvel = ekf.P(idxStateVel, idxStateVel);
    result.velStdNed(idxEst, :) = sqrt(diag(Rn2e' * Pvel * Rn2e)');
    result.phoneUsed(idxEst) = phoneInfo.idx;
    
    % Progress indicator
    if mod(idxEst, Constants.PROGRESS_BAR_STEP) == 0
        pb.print(idxEst, nGnssEpochs);
        fprintf(fid, '%i / %i \n', idxEst, nGnssEpochs);
    end
    
    % TODO: Remove (lever arm test)
    result.sat(idxEst) = sat;
    
    %% Prepare next observation
    idxEst = idxEst + 1;
    % Check if there are more measurements/observations
    [gnssEpoch, osrEpoch, phoneInfo] = getNextGnss(thisUtcSeconds, phones, osr);
    hasEnded = isempty(gnssEpoch) || idxEst > nGnssEpochs; % TODO check imu
    
    
end
% Progress indicator
pb.print(idxEst-1, nGnssEpochs);

% nEpochs     = size(result.xRTK, 2);
% estPosEcef  = result.xRTK(idxStatePos, :)';
% % estAtt      = result.xRTK(idxStateAtt, :)';
% % result.phone(nPhones, 1).posEcef = nan(nEpochs, 3);
% 
% for iEpoch = 1:nEpochs
%     Rn2e = compute_Rn2e(estPosEcef(iEpoch, 1), estPosEcef(iEpoch, 2), estPosEcef(iEpoch, 3));
%     Re2n = Rn2e';
%     
%     % TODO: transform position from master Rx to all Rx's
%     
%     % Position STD in NED
%     Ppos = PRTK(idxStatePos, idxStatePos, iEpoch);
%     result.posStdNed(iEpoch, :) = sqrt(diag(Rn2e' * Ppos * Rn2e)');
%     % Velocity to NED
%     result.velNed(iEpoch, :) = Re2n * result.xRTK(idxStateVel, iEpoch);
%     % Velocity STD in NED
%     Pvel = PRTK(idxStateVel, idxStateVel, iEpoch);
%     result.velStdNed(iEpoch, :) = sqrt(diag(Rn2e' * Pvel * Rn2e)');
% end

end