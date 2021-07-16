function [x0, ekf, result] = updateWithDD(x0, ekf, thisUtcSeconds, idxEst, statPos, doubleDifferences, result)
% UPDATEWITHDD Performs the KF update with the double differenced
% observations

% Initializations
config = Config.getInstance;
% Sequentally update with all DDs

if config.USE_CODE_DD
    [x0, ekf, result] = updateWithCodeDD(x0, ekf, thisUtcSeconds, idxEst, statPos, doubleDifferences, result);
    result.prNumDD(idxEst) = length([doubleDifferences(:).C]);
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
    result.phsNumDD(idxEst) = length([doubleDifferences(:).L]);
end
end %end of function updateWithDD
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [x0, ekf, result] = updateWithCodeDD(x0, ekf, thisUtcSeconds, idxEst, statPos, doubleDifferences, result)
% UPDATEWITHCODEDD performs the KF sequential update with all code DD's
config = Config.getInstance;
% Label to show on console when outliers are detected
label = 'Code DD';
% Initializations
idxStatePos = PVTUtils.getStateIndex(PVTUtils.ID_POS);
isPrRejected = zeros(1, length(doubleDifferences));
isPrInvalid = zeros(1, length(doubleDifferences));
iValidObs = 0;
idxSat = [];
innovation = [];
innovationCovariance = [];

for iObs = 1:length(doubleDifferences)
    % Transition model arguments
    fArgs.x0 = x0;
    fArgs.obsConst = doubleDifferences(iObs).constel;
    fArgs.pivSatPrn = doubleDifferences(iObs).pivSatPrn;
    fArgs.varSatPrn = doubleDifferences(iObs).varSatPrn;
    
    %% Code DD observation
    sigmaDD = doubleDifferences(iObs).pivSatSigmaC + doubleDifferences(iObs).varSatSigmaC;
    if ~isnan(doubleDifferences(iObs).C)    && ...
            sigmaDD > Constants.MIN_C_SIGMA && ...
            sigmaDD < Constants.MAX_C_SIGMA
        iValidObs = iValidObs + 1;
        
        idxSat(iValidObs) = PVTUtils.getSatFreqIndex(      ...
            doubleDifferences(iObs).varSatPrn,  ...
            doubleDifferences(iObs).constel,    ...
            doubleDifferences(iObs).freqHz);
        
        % Observation
        hArgs.z(iValidObs, 1) = doubleDifferences(iObs).C;
        
        % Observation estimation
        hArgs.y(iValidObs, 1) =                                             ...
            norm(statPos - doubleDifferences(iObs).pivSatPos)           -   ... % |stat - sat1|
            norm(x0(idxStatePos) - doubleDifferences(iObs).pivSatPos)   -   ... % |user - sat1|
            norm(statPos - doubleDifferences(iObs).varSatPos)           +   ... % |stat - sat2|
            norm(x0(idxStatePos) - doubleDifferences(iObs).varSatPos);          % |user - sat2|
        
        % Difference between LOS vectors of satellites towards receiver
        pivSatLosVec = unitVector(statPos - doubleDifferences(iObs).pivSatPos);
        varSatLosVec = unitVector(statPos - doubleDifferences(iObs).varSatPos);
        ddLosVec = varSatLosVec - pivSatLosVec;
        
        % Jacobian matrix
        hArgs.H(iValidObs, :) = zeros(1, PVTUtils.getNumStates);
        hArgs.H(iValidObs, idxStatePos) = ddLosVec;
        
        sigmaObs = [doubleDifferences(iObs).pivSatSigmaC
            doubleDifferences(iObs).varSatSigmaC];
        satElDeg = [doubleDifferences(iObs).pivSatElDeg
            doubleDifferences(iObs).varSatElDeg];
        diagR(iValidObs, 1) = config.COV_FACTOR_C * computeRtkMeasCovariance(...
            satElDeg, sigmaObs, config.SIGMA_C_M, doubleDifferences(iObs).constel);
        
        if config.UPDATE_MODE == 1
            hArgs.z = hArgs.z(iValidObs, 1);
            hArgs.y = hArgs.y(iValidObs, 1);
            hArgs.H = hArgs.H(iValidObs, :);
            hArgs.R = diagR(iValidObs, 1);
            % Process code observation
            [ekf, innovation(iValidObs), innovationCovariance(iValidObs), isPrRejected(iObs), ~, ~] =   ...
                EKF.processObservation(ekf, thisUtcSeconds,                 ...
                @fTransition, fArgs,                                        ...
                @hCodeDD, hArgs,                                            ...
                label,                                                      ...
                true); % true for sequential update
            
            % Update total-state with absolute position
            x0 = updateTotalState(ekf.x, statPos);
        end
    else
        isPrInvalid(iObs) = 1;
    end
end

if ~all(isPrInvalid) && (config.UPDATE_MODE == 2 || config.UPDATE_MODE == 3)
    hArgs.R = diag(diagR);
    % Process code observation
    [ekf, innovation, innovationCovariance, isPrRejected(~isPrInvalid), ~, ~] =           ...
        EKF.processObservation(ekf, thisUtcSeconds,                         ...
        @fTransition, fArgs,                                                ...
        @hCodeDD, hArgs,                                                    ...
        label,                                                              ...
        config.UPDATE_MODE == 2);
    
    % Update total-state with absolute position
    x0 = updateTotalState(ekf.x, statPos);
end

result.prInnovations(idxSat, idxEst) = innovation;
result.prInnovationCovariances(idxSat, idxEst) = innovationCovariance;
result.prRejectedHist(idxEst) = sum(isPrRejected);
result.prInvalidHist(idxEst) = sum(isPrInvalid);
end %end of function updateWithCodeDD
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [z, y, H, R] = hCodeDD(~, hArgs)
% HCODEDD provides the measurement model for the sequential code
% double-differenced observations

% Observation
z = hArgs.z;
% Observation estimation
y = hArgs.y;
% Jacobian matrix
H = hArgs.H;
% Measurement covariance matrix, consider DD sigmas
R = hArgs.R;
end %end of function hCodeDD
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [x0, ekf, result, isPhsRejected] = updateWithPhaseDD(x0, ekf, thisUtcSeconds, idxEst, statPos, doubleDifferences, result)
% UPDATEWITHPHASEDD performs the KF sequential update with all phase DD's
config = Config.getInstance;
% Label to show on console when outliers are detected
label = 'Phase DD';
% Initializations
idxStatePos = PVTUtils.getStateIndex(PVTUtils.ID_POS);
isPhsRejected = zeros(1, length(doubleDifferences));
isPhsInvalid = zeros(1, length(doubleDifferences));
iValidObs = 0;
idxSat = [];
innovation = [];
innovationCovariance = [];
for iObs = 1:length(doubleDifferences)    
    % Transition model arguments
    fArgs.x0 = x0;
    fArgs.obsConst = doubleDifferences(iObs).constel;
    fArgs.pivSatPrn = doubleDifferences(iObs).pivSatPrn;
    fArgs.varSatPrn = doubleDifferences(iObs).varSatPrn;
    
    %% Phase DD observation
    sigmaDD = doubleDifferences(iObs).pivSatSigmaL + doubleDifferences(iObs).varSatSigmaL;
    if ~isnan(doubleDifferences(iObs).L)    && ...
            sigmaDD > Constants.MIN_L_SIGMA && ...
            sigmaDD < Constants.MAX_L_SIGMA
        iValidObs = iValidObs + 1;
        
        idxSat(iValidObs) = PVTUtils.getSatFreqIndex(      ...
            doubleDifferences(iObs).varSatPrn,  ...
            doubleDifferences(iObs).constel,    ...
            doubleDifferences(iObs).freqHz);
        
        % (Re)initialization of ambiguities
        idxStatePivSat = PVTUtils.getStateIndex(PVTUtils.ID_SD_AMBIGUITY,   ...
            doubleDifferences(iObs).pivSatPrn,                              ...
            doubleDifferences(iObs).constel,                                ...
            doubleDifferences(iObs).freqHz);
        idxStateVarSat = PVTUtils.getStateIndex(PVTUtils.ID_SD_AMBIGUITY,   ...
            doubleDifferences(iObs).varSatPrn,                              ...
            doubleDifferences(iObs).constel,                                ...
            doubleDifferences(iObs).freqHz);
        % Ambiguities: set to CMC if it's 0 (not estimated yet for this sat)
        % TODO: what if N is estimated as 0
        if ekf.x(idxStatePivSat) == 0, ekf.x(idxStatePivSat) = doubleDifferences(iObs).pivSatCmcSd; end
        if ekf.x(idxStateVarSat) == 0, ekf.x(idxStateVarSat) = doubleDifferences(iObs).varSatCmcSd; end
        % Reinitialize ambiguity if LLI is on (pivot sat has always LLI = 0)
        if doubleDifferences(iObs).varSatIsLLI
            ekf.x(idxStateVarSat) = doubleDifferences(iObs).varSatCmcSd;
            ekf.P(idxStateVarSat, idxStateVarSat) = config.SIGMA_P0_SD_AMBIG^2;
        end
        
        lambda = Constants.CELERITY / doubleDifferences(iObs).freqHz;
        % Observation
        hArgs.z(iValidObs, 1) = doubleDifferences(iObs).L;
        
        % Observation estimation
        hArgs.y(iValidObs, 1) =                                             ...
            norm(statPos - doubleDifferences(iObs).pivSatPos)           -   ... % |stat - sat1|
            norm(x0(idxStatePos) - doubleDifferences(iObs).pivSatPos)   -   ... % |user - sat1|
            norm(statPos - doubleDifferences(iObs).varSatPos)           +   ... % |stat - sat2|
            norm(x0(idxStatePos) - doubleDifferences(iObs).varSatPos)   +   ... % |user - sat2|
            lambda * ekf.x(idxStatePivSat)                              -   ... % lambda * N_piv
            lambda * ekf.x(idxStateVarSat);                                     % lambda * N_var
        
        % Difference between LOS vectors of satellites towards receiver
        pivSatLosVec = unitVector(statPos - doubleDifferences(iObs).pivSatPos);
        varSatLosVec = unitVector(statPos - doubleDifferences(iObs).varSatPos);
        ddLosVec = varSatLosVec - pivSatLosVec;
        
        % Jacobian matrix
        hArgs.H(iValidObs, :) = zeros(1, PVTUtils.getNumStates);
        hArgs.H(iValidObs, idxStatePos) = ddLosVec;
        hArgs.H(iValidObs, idxStatePivSat) = lambda;
        hArgs.H(iValidObs, idxStateVarSat) = -lambda;
        
        sigmaObs = [doubleDifferences(iObs).pivSatSigmaL
            doubleDifferences(iObs).varSatSigmaL];
        satElDeg = [doubleDifferences(iObs).pivSatElDeg
            doubleDifferences(iObs).varSatElDeg];
        diagR(iValidObs, 1) = config.COV_FACTOR_L * computeRtkMeasCovariance(...
            satElDeg, sigmaObs, config.SIGMA_L_M, doubleDifferences(iObs).constel);
        
        if config.UPDATE_MODE == 1
            hArgs.z = hArgs.z(iValidObs, 1);
            hArgs.y = hArgs.y(iValidObs, 1);
            hArgs.H = hArgs.H(iValidObs, :);
            hArgs.R = diagR(iValidObs, 1);
            % Process code observation
            [ekf, innovation(iValidObs), innovationCovariance(iValidObs), isPhsRejected(iObs), ~, ~] =   ...
                EKF.processObservation(ekf, thisUtcSeconds,                 ...
                @fTransition, fArgs,                                        ...
                @hPhaseDD, hArgs,                                           ...
                label,                                                      ...
                true); % true for sequential update
            
            % Update total-state with absolute position
            x0 = updateTotalState(ekf.x, statPos);
        end
        
        % If Phase DD is rejected, reinitialize ambiguity estimations and
        % covariances
        if isPhsRejected(iObs)
            ekf.x(idxStateVarSat) = -doubleDifferences(iObs).varSatCmcSd / lambda;
            ekf.P(idxStateVarSat, idxStateVarSat) = config.SIGMA_P0_SD_AMBIG^2;
        end
        
        % Update total-state with absolute position
        x0 = updateTotalState(ekf.x, statPos);
    else
        isPhsInvalid(iObs) = 1;
    end
end

if ~all(isPhsInvalid) && (config.UPDATE_MODE == 2 || config.UPDATE_MODE == 3)
    hArgs.R = diag(diagR);
    % Process code observation
    [ekf, innovation, innovationCovariance, isPhsRejected(~isPhsInvalid), ~, ~] =           ...
        EKF.processObservation(ekf, thisUtcSeconds,                         ...
        @fTransition, fArgs,                                                ...
        @hPhaseDD, hArgs,                                                    ...
        label,                                                              ...
        config.UPDATE_MODE == 2);
    
    % Update total-state with absolute position
    x0 = updateTotalState(ekf.x, statPos);
end

result.phsInnovations(idxSat, idxEst) = innovation;
result.phsInnovationCovariances(idxSat, idxEst) = innovationCovariance;
result.phsRejectedHist(idxEst) = sum(isPhsRejected);
result.phsInvalidHist(idxEst) = sum(isPhsInvalid);
end %end of function updateWithPhaseDD
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [z, y, H, R] = hPhaseDD(~, hArgs)
% HCODEDD provides the measurement model for the sequential code
% double-differenced observations
% Observation
z = hArgs.z;
% Observation estimation
y = hArgs.y;
% Jacobian matrix
H = hArgs.H;
% Measurement covariance matrix, consider DD sigmas
R = hArgs.R;
end %end of function hPhaseDD
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%