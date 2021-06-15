function [x0, ekf, result] = updateWithDD(x0, ekf, thisUtcSeconds, idxEst, statPos, doubleDifferences, result)
% UPDATEWITHDD Performs the KF update with the double differenced
% observations

% Initializations
config = Config.getInstance;
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
    %     fArgs.statPos = statPos;
    % Measurement model arguments
    hArgs.x0 = x0;
    hArgs.statPos = statPos;
    hArgs.obsConst = doubleDifferences(iObs).constel;
    hArgs.freqHz = doubleDifferences(iObs).freqHz;
    hArgs.pivSatPrn = doubleDifferences(iObs).pivSatPrn;
    hArgs.varSatPrn = doubleDifferences(iObs).varSatPrn;
    hArgs.pivSatPos = doubleDifferences(iObs).pivSatPos;
    hArgs.varSatPos = doubleDifferences(iObs).varSatPos;
    hArgs.satElDeg = [doubleDifferences(iObs).pivSatElDeg
        doubleDifferences(iObs).varSatElDeg];
    
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
    x0 = updateTotalState(ekf.x, statPos);
    
    %% Phase DD observation
    idxStatePivSat = PVTUtils.getStateIndex(PVTUtils.ID_SD_AMBIGUITY, hArgs.pivSatPrn, hArgs.obsConst);
    idxStateVarSat = PVTUtils.getStateIndex(PVTUtils.ID_SD_AMBIGUITY, hArgs.varSatPrn, hArgs.obsConst);
    % Ambiguities: set to CMC if it's 0 (not estimated yet for this sat)
    % TODO: what if N is estimated as 0
    if ekf.x(idxStatePivSat) == 0, ekf.x(idxStatePivSat) = doubleDifferences(iObs).pivSatCmcSd; end
    if ekf.x(idxStateVarSat) == 0, ekf.x(idxStateVarSat) = doubleDifferences(iObs).varSatCmcSd; end
    % Reinitialize ambiguity if LLI is on (pivot sat has always LLI = 0)
    if doubleDifferences(iObs).varSatIsLLI
        ekf.x(idxStateVarSat) = doubleDifferences(iObs).varSatCmcSd;
        ekf.P(idxStateVarSat, idxStateVarSat) = config.SIGMA_P0_SD_AMBIG^2;
    end
    
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
    
    % If Phase DD is rejected, reinitialize ambiguity estimations and
    % covariances
    if rejected
        ekf.x(idxStatePivSat) = doubleDifferences(iObs).pivSatCmcSd;
        ekf.x(idxStateVarSat) = doubleDifferences(iObs).varSatCmcSd;
        ekf.P(idxStatePivSat, idxStatePivSat) = config.SIGMA_P0_SD_AMBIG^2;
        ekf.P(idxStateVarSat, idxStateVarSat) = config.SIGMA_P0_SD_AMBIG^2;
    end
    
    result.phsInnovations(idxSat, idxEst) = innovation;
    result.phsInnovationCovariances(idxSat, idxEst) = innovationCovariance;
    result.phsRejectedHist(idxEst) = result.phsRejectedHist(idxEst) + rejected;
    
    % Update total-state with absolute position
    x0 = updateTotalState(ekf.x, statPos);
end
% Percentage of rejected code observations
result.prRejectedHist(idxEst) = 100*result.prRejectedHist(idxEst) / length(doubleDifferences);
result.phsRejectedHist(idxEst) = 100*result.phsRejectedHist(idxEst) / length(doubleDifferences);
end %end of function updateWithDD
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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