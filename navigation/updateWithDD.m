function [x0, ekf, result] = updateWithDD(x0, ekf, thisUtcSeconds, idxEst, statPos, doubleDifferences, result)
% UPDATEWITHDD Performs the KF update with the double differenced
% observations

% Initializations
config = Config.getInstance;
% Sequentally update with all DDs

if config.USE_CODE_DD
    [x0, ekf, result] = updateWithCodeDD(x0, ekf, thisUtcSeconds, idxEst, statPos, doubleDifferences, result);
end

if config.USE_PHASE_DD
    [x0, ekf, result, isPhsRejections] = updateWithPhaseDD(x0, ekf, thisUtcSeconds, idxEst, statPos, doubleDifferences, result);
    % If all are rejected in one set of constellation+freq, reinitialize 
    % ambiguity for pivot satellite of that set
    ddConstNum = c2i([doubleDifferences(:).constel]');
    ddFreq = [doubleDifferences(:).freqHz]';
    constFreqs = unique([ddConstNum ddFreq], 'rows', 'stable');
    for i = 1:size(constFreqs, 1)
        % Find indices of i'th combination of const and freq
        idxConstFreq = find(ddConstNum == constFreqs(i, 1) & ddFreq == constFreqs(i, 2));
        if all(isPhsRejections(idxConstFreq))
            idxStatePivSat = PVTUtils.getStateIndex(...
                PVTUtils.ID_SD_AMBIGUITY, ...
                doubleDifferences(idxConstFreq(1)).pivSatPrn,   ... % PRN of piv sat for this const+freq
                doubleDifferences(idxConstFreq(1)).constel,     ... % Constellation
                constFreqs(i, 2));                                  % Freq of this set of DDs
            lambda = Constants.CELERITY / constFreqs(i, 2);
            ekf.x(idxStatePivSat) = - doubleDifferences(idxConstFreq(1)).pivSatCmcSd / lambda;
            ekf.P(idxStatePivSat, idxStatePivSat) = config.SIGMA_P0_SD_AMBIG^2;
            [x0, ekf, result] = updateWithPhaseDD(x0, ekf, thisUtcSeconds, idxEst, statPos, doubleDifferences, result);
        end
    end
end
% Number of available and rejected observations
result.prNumDD(idxEst) = length([doubleDifferences(:).C]);
result.phsNumDD(idxEst) = length([doubleDifferences(:).L]);
result.prRejectedHist(idxEst) = result.prRejectedHist(idxEst);
result.phsRejectedHist(idxEst) = result.phsRejectedHist(idxEst);
end %end of function updateWithDD
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [x0, ekf, result] = updateWithCodeDD(x0, ekf, thisUtcSeconds, idxEst, statPos, doubleDifferences, result)
% UPDATEWITHCODEDD performs the KF sequential update with all code DD's
isPrRejections = zeros(1, length(doubleDifferences));
for iObs = 1:length(doubleDifferences)
    idxSat = PVTUtils.getSatFreqIndex(      ...
        doubleDifferences(iObs).varSatPrn,  ...
        doubleDifferences(iObs).constel,    ...
        doubleDifferences(iObs).freqHz);
    
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
    if ~isnan(doubleDifferences(iObs).C) && ...
            (doubleDifferences(iObs).pivSatSigmaC + doubleDifferences(iObs).varSatSigmaC) < Constants.MAX_C_SIGMA
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
        [ekf, innovation, innovationCovariance, isPrRejections(iObs), ~, ~] = ...
            EKF.processObservation(ekf, thisUtcSeconds,           ...
            @fTransition, fArgs,                                    ...
            @hCodeDD, hArgs,                                        ...
            label);
        
        result.prInnovations(idxSat, idxEst) = innovation;
        result.prInnovationCovariances(idxSat, idxEst) = innovationCovariance;
        
        % Update total-state with absolute position
        x0 = updateTotalState(ekf.x, statPos);
    else
        isPrRejections(iObs) = 1;
    end
end
result.prRejectedHist(idxEst) = sum(isPrRejections);
end %end of function updateWithCodeDD
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [x0, ekf, result, isPhsRejections] = updateWithPhaseDD(x0, ekf, thisUtcSeconds, idxEst, statPos, doubleDifferences, result)
% UPDATEWITHPHASEDD performs the KF sequential update with all phase DD's
config = Config.getInstance;
isPhsRejections = zeros(1, length(doubleDifferences));
for iObs = 1:length(doubleDifferences)
    idxSat = PVTUtils.getSatFreqIndex(      ...
        doubleDifferences(iObs).varSatPrn,  ...
        doubleDifferences(iObs).constel,    ...
        doubleDifferences(iObs).freqHz);
    
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
    
    %% Phase DD observation
    if ~isnan(doubleDifferences(iObs).L) && ...
            (doubleDifferences(iObs).pivSatSigmaL + doubleDifferences(iObs).varSatSigmaL) < Constants.MAX_L_SIGMA
        idxStatePivSat = PVTUtils.getStateIndex(PVTUtils.ID_SD_AMBIGUITY, hArgs.pivSatPrn, hArgs.obsConst, hArgs.freqHz);
        idxStateVarSat = PVTUtils.getStateIndex(PVTUtils.ID_SD_AMBIGUITY, hArgs.varSatPrn, hArgs.obsConst, hArgs.freqHz);
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
        [ekf, innovation, innovationCovariance, isPhsRejections(iObs), ~, ~] = ...
            EKF.processObservation(ekf, thisUtcSeconds,           ...
            @fTransition, fArgs,                                    ...
            @hPhaseDD, hArgs,                                        ...
            label);
        
        % If Phase DD is rejected, reinitialize ambiguity estimations and
        % covariances
        if isPhsRejections(iObs)
            lambda = Constants.CELERITY / hArgs.freqHz;
            ekf.x(idxStateVarSat) = -doubleDifferences(iObs).varSatCmcSd / lambda;
            ekf.P(idxStateVarSat, idxStateVarSat) = config.SIGMA_P0_SD_AMBIG^2;
        end
        
        result.phsInnovations(idxSat, idxEst) = innovation;
        result.phsInnovationCovariances(idxSat, idxEst) = innovationCovariance;
        
        % Update total-state with absolute position
        x0 = updateTotalState(ekf.x, statPos);
    else
        isPhsRejections(iObs) = 1;
    end
end
result.phsRejectedHist(idxEst) = sum(isPhsRejections);
end %end of function updateWithPhaseDD
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
y = norm(hArgs.statPos - hArgs.pivSatPos)   - ...   % |stat - sat1|
    norm(rxPos - hArgs.pivSatPos)           - ...   % |user - sat1|
    norm(hArgs.statPos - hArgs.varSatPos)   + ...   % |stat - sat2|
    norm(rxPos - hArgs.varSatPos);                  % |user - sat2|

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
idxStatePivSat = PVTUtils.getStateIndex(PVTUtils.ID_SD_AMBIGUITY, hArgs.pivSatPrn, hArgs.obsConst, hArgs.freqHz);
idxStateVarSat = PVTUtils.getStateIndex(PVTUtils.ID_SD_AMBIGUITY, hArgs.varSatPrn, hArgs.obsConst, hArgs.freqHz);
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
end %end of function hPhaseDD
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%