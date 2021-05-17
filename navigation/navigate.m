function [xEstHist, prInnovations, prInnovationCovariances, dopInnovations, ...
    dopInnovationCovariances, utcSecondsHist, sigmaHist, prRejectedHist, dopRejectedHist] = ...
    navigate(gnssRnx, imuMeas, nav, osrRnx, ref)
%NAVIGATE Summary of this function goes here
%   Detailed explanation goes here

%% Initializations
nStates = PVTUtils.getNumStates();
nSatellites = PVTUtils.getNumSatelliteIndices();
gnssEpochs = unique(gnssRnx.utcSeconds);
nGnssEpochs = length(gnssEpochs);
% EKF:
esekf = EKF.build(uint16(nStates));
if ~Config.OUTLIER_REJECTION, esekf.probabilityOfFalseOutlierRejection = 0; end

%% Obtain first position
[esekf.x, esekf.P, esekf.tx] = getFirstPosition(gnssRnx, nav);

% Loop variables
% thisUtcSeconds -> time ref for both IMU and GNSS.
thisUtcSeconds = esekf.tx; % First time is from the first GNSS estimation
idxEst = 1;
xEstHist = esekf.x; % TODO change to error state, prealocate if num pos is known (= groundtruth?)
sigmaHist = zeros(nStates, 1);

% First gnss observations is the same as for the first approx position
[gnssRx, gnssOsr] = getNextGnss(thisUtcSeconds, gnssRnx, osrRnx, 'this');
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
    % Transition model arguments
    fArgs.x0 = x0;
    
    thisUtcSeconds = gnssRx.utcSeconds; % TODO check imu time
    
    % Get states of satellites selected in Config
    [satPos, satClkBias, satClkDrift, satVel] = ...
        compute_satellite_state_all(gnssRx.tow, gnssRx.obs, nav, Config.CONSTELLATIONS);
    
    % Remove invalid observations (no ephem, elevation mask, large dopplers)
    [gnssRx.obs, satPos, satClkBias, satClkDrift, satVel] = ...
        filterObs(gnssRx.obs, satPos, satClkBias, satClkDrift, satVel, x0(1:3));
    
    if length([gnssRx.obs(:).C]) < 4 + PVTUtils.getNumFrequencies + PVTUtils.getNumConstellations
        warning('TOW = %d - Not enough observations to estimate a potition. Propagating state.', gnssRx.tow);
        esekf = EKF.propagateState(esekf, thisUtcSeconds, @fTransition, fArgs);
    else
        % TODO: compute double differences
        doubleDifferences = computeDoubleDifferences(gnssRx, gnssOsr);

        % Measurement model arguments
        hArgs.x0 = x0;
        
        % Sequentally update with all observations
        for iObs = 1:length(doubleDifferences)
            
%             idxSat = PVTUtils.getSatelliteIndex(gnss.obs(iObs).prn, gnss.obs(iObs).constellation);

            % TODO: Pack arguments that are common for Code and Phase observations
%             hArgs.obsFreqHz = gnss.obs(iObs).D_fcarrier_Hz;
%             hArgs.obsConst = gnss.obs(iObs).constellation;
%             hArgs.satPos = satPos(:, iObs);
%             hArgs.satVel = satVel(:, iObs);
%             hArgs.satClkBias = satClkBias(iObs);
%             hArgs.satClkDrift = satClkDrift(iObs);
%             hArgs.satElev = satElDeg(iObs);
            
            % TODO: Pack code observation
%             hArgs.obs = prCorr(iObs);
%             hArgs.sigmaObs = gnss.obs(iObs).C_sigma;

            % TODO: Process code observation
%             [esekf, innovation, innovationCovariance, rejected, z, y] = ...
%                 EKF.processObservation(esekf, thisUtcSeconds, ...
%                 @fTransition, fArgs, ...
%                 @hCodeObs, hArgs, ...
%                 sprintf('Code (''%c%d'' f = %d=',hArgs.obsConst, gnss.obs(iObs).prn, hArgs.obsFreqHz));
%             prInnovations(idxSat, idxEst) = innovation;
%             prInnovationCovariances(idxSat, idxEst) = innovationCovariance;
%             prRejectedHist(idxEst) = prRejectedHist(idxEst) + rejected;
%             fArgs.x0 = esekf.x;
%             hArgs.x0 = esekf.x;
            
            % TODO: Process phase observation
        end
    end
    
    utcSecondsHist(idxEst) = thisUtcSeconds;
    
    xEstHist(:, idxEst) = esekf.x; % TODO perform correction when using error-state
    sigmaHist(:, idxEst) = sqrt(diag(esekf.P));
    idxEst = idxEst + 1;
    
    % Check if there are more measurements/observations
    gnssRx = getNextGnss(thisUtcSeconds, gnssRnx);
    hasEnded = isempty(gnssRx); % TODO check imu
end

end

% State transition function
function [x2, F, Q] = fTransition(x1, t1, t2, fArgs)
% dt = t2 - t1;
% F = eye(PVTUtils.getNumStates);
% % dp/dv
% F(1,4) = dt; F(2,5) = dt; F(3,6) = dt;
% % d clkbias/d clkdrift
% F(7,8) = dt;
% % State prediction
% x2 = F * x1;
% % Process noise covariance matrix - Velocity
% rotN2E = compute_Rn2e(fArgs.x0(1), fArgs.x0(2), fArgs.x0(3));
% Qvel = dt * rotN2E * diag(Config.SIGMA_VEL_NED.^2) * rotN2E';
% % Process noise covariance matrix - Clock
% Qclk(1, 1) = dt * Config.SIGMA_CLK_BIAS^2 + 1/3 * dt^3 * Config.SIGMA_CLK_DRIFT^2;
% Qclk(1, 2) = 1/2 * dt^2 * Config.SIGMA_CLK_DRIFT^2;
% Qclk(2, 1) = Qclk(1, 2);
% Qclk(2, 2) =  dt * Config.SIGMA_CLK_DRIFT^2;
% % Process noise covariance matrix - Inter-Frequency bias(es)
% Qif = dt * Config.SIGMA_CLK_INTERFREQ^2 * eye(PVTUtils.getNumFrequencies - 1);
% % Process noise covariance matrix - Inter-System bias(es)
% Qis = dt * Config.SIGMA_CLK_INTERSYS^2 * eye(PVTUtils.getNumConstellations - 1);
% % Process noise covariance matrix
% Q = blkdiag(zeros(3), Qvel, Qclk, Qif, Qis);
end %end of function fTransition
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [z, y, H, R] = hCodeObs(~, hArgs)
% % HCODEOBS provides the measurement model for the sequential code
% % observations
% 
% % Initializations
% idxStatePos = PVTUtils.getStateIndex(PVTUtils.ID_POS);
% idxStateClkBias = PVTUtils.getStateIndex(PVTUtils.ID_CLK_BIAS);
% idxStateAllFreqBias = PVTUtils.getStateIndex(PVTUtils.ID_INTER_FREQ_BIAS);
% idxStateAllSysBias = PVTUtils.getStateIndex(PVTUtils.ID_INTER_SYS_BIAS);
% idxFreq = PVTUtils.getFreqIdx(hArgs.obsFreqHz);
% idxConstel = PVTUtils.getConstelIdx(hArgs.obsConst);
% 
% 
% % Observation
% z = hArgs.obs;
% 
% % Distance between receiver and satellite. Second term is Sagnac effect.
% dist = norm(hArgs.x0(idxStatePos) - hArgs.satPos) + ...
%     Constants.OMEGA_E/Constants.CELERITY * ...
%     (hArgs.satPos(1)*hArgs.x0(idxStatePos(2)) - ...
%     hArgs.satPos(2)*hArgs.x0(idxStatePos(1)));
% 
% % Inter-frequency bias if it's not ref freq
% if isempty(idxStateAllFreqBias) || idxFreq == 1
%     idxThisFreqBias = [];
%     interFreqBias = 0; % If only 1 freq or this is ref freq
% else
%     idxThisFreqBias = idxStateAllFreqBias(idxFreq-1);
%     interFreqBias = hArgs.x0(idxThisFreqBias);
% end
% % Inter-system bias if it's not ref system
% if isempty(idxStateAllSysBias) || idxConstel == 1
%     idxThisSysBias = [];
%     interSysBias = 0; % If only 1 const or this is ref const
% else
%     idxThisSysBias = idxStateAllSysBias(idxConstel-1);
%     interSysBias = hArgs.x0(idxThisSysBias);
% end
% % Observation estimation
% y = dist + ...                                  % Distance between receiver and satellite.
%     hArgs.x0(idxStateClkBias) - ...             % Receiver clock bias
%     Constants.CELERITY * hArgs.satClkBias + ... % Satellite clock bias
%     interFreqBias - ...                         % Inter-frequency bias
%     interSysBias;                               % Inter-system bias
% 
% % Jacobian matrix
% H = zeros(1, PVTUtils.getNumStates);
% H(idxStatePos) = (hArgs.x0(idxStatePos) - hArgs.satPos)/dist;
% H(idxStateClkBias) = 1;
% H(idxThisFreqBias) = idxFreq > 1; % 1 if not ref freq
% H(idxThisSysBias) = -(idxConstel > 1); % -1 if not ref const
% 
% % Measurement covariance matrix
% R = Config.COV_FACTOR_C * computeMeasCovariance(hArgs.satElev, ...
%     hArgs.sigmaObs, Config.SIGMA_PR_M, hArgs.obsConst);
end %end of function hCodeObs
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
