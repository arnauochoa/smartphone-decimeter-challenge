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

% Initializations
config = Config.getInstance;
idxStatePos = PVTUtils.getStateIndex(PVTUtils.ID_POS);
isPrRejected = zeros(1, length(doubleDifferences));
isPrInvalid = zeros(1, length(doubleDifferences));
hArgs.z = [];
hArgs.y = [];
hArgs.H = [];
idxPivVarR = [];     % Indices of pivot and variable satellite for matrix D and R_SD
covPivVarR = [];          % Vector of diagonal R_SD
numValidDD = 0;

% Transition model arguments
fArgs.x0 = x0;

% Loop over all DDs to build y, z, H and R
for iObs = 1:length(doubleDifferences)    
    % Consider only non-NaN DDs with sigma between bounds
    sigmaDD = doubleDifferences(iObs).pivSatSigmaC + doubleDifferences(iObs).varSatSigmaC;
    %% Code DD observation
    if ~isnan(doubleDifferences(iObs).C)        && ...
            sigmaDD < Constants.MAX_C_SIGMA     && ...
            sigmaDD > Constants.MIN_C_SIGMA
        numValidDD = numValidDD + 1;
        
        % Total indices of the (constellation, satellite, frequency) 
        idxPivSat(numValidDD) = PVTUtils.getSatFreqIndex(   ...
            doubleDifferences(iObs).pivSatPrn,              ...
            doubleDifferences(iObs).constel,                ...
            doubleDifferences(iObs).freqHz);
        idxVarSat(numValidDD) = PVTUtils.getSatFreqIndex(   ...
            doubleDifferences(iObs).varSatPrn,              ...
            doubleDifferences(iObs).constel,                ...
            doubleDifferences(iObs).freqHz);
        
        % Observation
        hArgs.z(numValidDD, 1) = doubleDifferences(iObs).C;
        
        % Observation estimation
        hArgs.y(numValidDD, 1) = ...
            norm(statPos - doubleDifferences(iObs).pivSatPos)           - ...   % |stat - sat1|
            norm(x0(idxStatePos) - doubleDifferences(iObs).pivSatPos)   - ...   % |user - sat1|
            norm(statPos - doubleDifferences(iObs).varSatPos)           + ...   % |stat - sat2|
            norm(x0(idxStatePos) - doubleDifferences(iObs).varSatPos);          % |user - sat2|
        
        % Difference between LOS vectors of satellites towards receiver
        pivSatLosVec = unitVector(statPos - doubleDifferences(iObs).pivSatPos);
        varSatLosVec = unitVector(statPos - doubleDifferences(iObs).varSatPos);
        ddLosVec = varSatLosVec - pivSatLosVec;
        
        % Jacobian matrix
        hArgs.H(numValidDD, :) = zeros(1, PVTUtils.getNumStates);
        hArgs.H(numValidDD, idxStatePos) = ddLosVec;
        
        % D matrix to convert from R_SD to R_DD
        idxPivVarR(numValidDD, :) = [idxPivSat(numValidDD) idxVarSat(numValidDD)];
        % Pivot satellite covariance
        covPivVarR(numValidDD, 1) = config.COV_FACTOR_C *   ...
            computeRtkMeasCovariance(                       ...
            doubleDifferences(iObs).pivSatElDeg,            ... % Elevation in degrees
            doubleDifferences(iObs).pivSatSigmaC,           ... % Code uncertainty
            config.SIGMA_C_M,                               ... % Default code STD
            doubleDifferences(iObs).constel);                   % Constellation
        % Variable satellite covariance
        covPivVarR(numValidDD, 2) = config.COV_FACTOR_C *   ...
            computeRtkMeasCovariance(                       ...
            doubleDifferences(iObs).varSatElDeg,            ... % Elevation in degrees
            doubleDifferences(iObs).varSatSigmaC,           ... % Code uncertainty
            config.SIGMA_C_M,                               ... % Default code STD
            doubleDifferences(iObs).constel);                   % Constellation
    else
        isPrInvalid(iObs) = 1;
    end
end
if ~all(isPrInvalid)
    idxPivVarR = reassignUnique(idxPivVarR);
    hArgs.R = buildRdd(idxPivVarR, covPivVarR, numValidDD);
    
    % Process code observation
    [ekf, innovations, innovationCovariances, rejections, ~, ~] = ...
        EKF.processObservation(ekf, thisUtcSeconds,           ...
        @fTransition, fArgs,                                    ...
        @hCodeDD, hArgs,                                        ...
        'Code DD');
    % Update total-state with absolute position
    x0 = updateTotalState(ekf.x, statPos);
    % Add rejections to vector with size of all observations
    isPrRejected(~isPrInvalid) = rejections;
    
    result.prInnovations(idxVarSat, idxEst) = innovations;
    result.prInnovationCovariances(idxVarSat, idxEst) = innovationCovariances;
end

% TODO check how to deal with rejections
result.prRejectedHist(idxEst) = sum(isPrRejected);
result.prInvalidHist(idxEst) = sum(isPrInvalid);
end %end of function updateWithCodeDD
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [x0, ekf, result, isPhsRejected] = updateWithPhaseDD(x0, ekf, thisUtcSeconds, idxEst, statPos, doubleDifferences, result)
% UPDATEWITHCODEDD performs the KF sequential update with all code DD's

% Initializations
config = Config.getInstance;
idxStatePos = PVTUtils.getStateIndex(PVTUtils.ID_POS);
isPhsRejected = zeros(1, length(doubleDifferences));
isPhsInvalid = zeros(1, length(doubleDifferences));
hArgs.z = [];
hArgs.y = [];
hArgs.H = [];
idxPivVarR = [];     % Indices of pivot and variable satellite for matrix D and R_SD
covPivVarR = [];          % Vector of diagonal R_SD
numValidDD = 0;

% Transition model arguments
fArgs.x0 = x0;

% Loop over all DDs to build y, z, H and R
for iObs = 1:length(doubleDifferences)    
    % Consider only non-NaN DDs with sigma between bounds
    sigmaDD = doubleDifferences(iObs).pivSatSigmaL + doubleDifferences(iObs).varSatSigmaL;
    %% Phase DD observation
    if ~isnan(doubleDifferences(iObs).L)        && ...
            sigmaDD < Constants.MAX_L_SIGMA     && ...
            sigmaDD > Constants.MIN_L_SIGMA
        numValidDD = numValidDD + 1;
        
        % Total indices of the (constellation, satellite, frequency) 
        idxPivSat(numValidDD) = PVTUtils.getSatFreqIndex(   ...
            doubleDifferences(iObs).pivSatPrn,              ...
            doubleDifferences(iObs).constel,                ...
            doubleDifferences(iObs).freqHz);
        idxVarSat(numValidDD) = PVTUtils.getSatFreqIndex(   ...
            doubleDifferences(iObs).varSatPrn,              ...
            doubleDifferences(iObs).constel,                ...
            doubleDifferences(iObs).freqHz);
        
        % Index of ambiguities in state vector
        idxStatePivSat = PVTUtils.getStateIndex(PVTUtils.ID_SD_AMBIGUITY,   ...
            doubleDifferences(iObs).pivSatPrn,                              ...
            doubleDifferences(iObs).constel,                                ...
            doubleDifferences(iObs).freqHz);
        idxStateVarSat = PVTUtils.getStateIndex(PVTUtils.ID_SD_AMBIGUITY,   ...
            doubleDifferences(iObs).varSatPrn,                              ...
            doubleDifferences(iObs).constel,                                ...
            doubleDifferences(iObs).freqHz);
        
        %         % Ambiguities: set to CMC if it's 0 (not estimated yet for this sat)
        % TODO: what if N is estimated as 0
        if ekf.x(idxStatePivSat) == 0, ekf.x(idxStatePivSat) = doubleDifferences(iObs).pivSatCmcSd; end
        if ekf.x(idxStateVarSat) == 0, ekf.x(idxStateVarSat) = doubleDifferences(iObs).varSatCmcSd; end
        % Reinitialize ambiguity if LLI is on (pivot sat has always LLI = 0)
        if doubleDifferences(iObs).varSatIsLLI
            ekf.x(idxStateVarSat) = doubleDifferences(iObs).varSatCmcSd;
            ekf.P(idxStateVarSat, idxStateVarSat) = config.SIGMA_P0_SD_AMBIG^2;
        end
        
        % Observation
        hArgs.z(numValidDD, 1) = doubleDifferences(iObs).L;
        
        % Observation estimation
        lambda = Constants.CELERITY / doubleDifferences(iObs).freqHz;
        hArgs.y(numValidDD, 1) = ...
            norm(statPos - doubleDifferences(iObs).pivSatPos)           - ...   % |stat - sat1|
            norm(x0(idxStatePos) - doubleDifferences(iObs).pivSatPos)   - ...   % |user - sat1|
            norm(statPos - doubleDifferences(iObs).varSatPos)           + ...   % |stat - sat2|
            norm(x0(idxStatePos) - doubleDifferences(iObs).varSatPos)   + ...   % |user - sat2|
            lambda * ekf.x(idxStatePivSat)                              - ...   % lambda * N_piv
            lambda * ekf.x(idxStateVarSat);                                     % lambda * N_var
        
        % Difference between LOS vectors of satellites towards receiver
        pivSatLosVec = unitVector(statPos - doubleDifferences(iObs).pivSatPos);
        varSatLosVec = unitVector(statPos - doubleDifferences(iObs).varSatPos);
        ddLosVec = varSatLosVec - pivSatLosVec;
        
        % Jacobian matrix
        hArgs.H(numValidDD, :) = zeros(1, PVTUtils.getNumStates);
        hArgs.H(numValidDD, idxStatePos) = ddLosVec;
        hArgs.H(numValidDD, idxStatePivSat) = lambda;
        hArgs.H(numValidDD, idxStateVarSat) = -lambda;
        
        % D matrix to convert from R_SD to R_DD
        idxPivVarR(numValidDD, :) = [idxPivSat(numValidDD) idxVarSat(numValidDD)];
        % Pivot satellite covariance
        covPivVarR(numValidDD, 1) = config.COV_FACTOR_L *   ...
            computeRtkMeasCovariance(                       ...
            doubleDifferences(iObs).pivSatElDeg,            ... % Elevation in degrees
            doubleDifferences(iObs).pivSatSigmaL,           ... % Code uncertainty
            config.SIGMA_L_M,                               ... % Default code STD
            doubleDifferences(iObs).constel);                   % Constellation
        % Variable satellite covariance
        covPivVarR(numValidDD, 2) = config.COV_FACTOR_L *   ...
            computeRtkMeasCovariance(                       ...
            doubleDifferences(iObs).varSatElDeg,            ... % Elevation in degrees
            doubleDifferences(iObs).varSatSigmaL,           ... % Code uncertainty
            config.SIGMA_L_M,                               ... % Default code STD
            doubleDifferences(iObs).constel);                   % Constellation
    else
        isPhsInvalid(iObs) = 1;
    end
end

if ~all(isPhsInvalid)
    idxPivVarR = reassignUnique(idxPivVarR);
    hArgs.R = buildRdd(idxPivVarR, covPivVarR, numValidDD);
    
    % Process code observation
    [ekf, innovations, innovationCovariances, rejections, ~, ~] = ...
        EKF.processObservation(ekf, thisUtcSeconds,           ...
        @fTransition, fArgs,                                    ...
        @hPhaseDD, hArgs,                                        ...
        'Phase DD');
    % Update total-state with absolute position
    x0 = updateTotalState(ekf.x, statPos);
    % Add rejections to vector with size of all observations
    isPhsRejected(~isPhsInvalid) = rejections;
    
    result.phsInnovations(idxVarSat, idxEst) = innovations;
    result.phsInnovationCovariances(idxVarSat, idxEst) = innovationCovariances;
end

% TODO check how to deal with rejections
result.phsRejectedHist(idxEst) = sum(isPhsRejected);
result.phsInvalidHist(idxEst) = sum(isPhsInvalid);
end %end of function updateWithCodeDD
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%




% function [x0, ekf, result, isPhsRejections] = updateWithPhaseDD(x0, ekf, thisUtcSeconds, idxEst, statPos, doubleDifferences, result)
% % UPDATEWITHPHASEDD performs the KF sequential update with all phase DD's
% config = Config.getInstance;
% isPhsRejections = zeros(1, length(doubleDifferences));
% for iObs = 1:length(doubleDifferences)
%     idxSat = PVTUtils.getSatFreqIndex(      ...
%         doubleDifferences(iObs).varSatPrn,  ...
%         doubleDifferences(iObs).constel,    ...
%         doubleDifferences(iObs).freqHz);
%
%     % Transition model arguments
%     fArgs.x0 = x0;
%     fArgs.obsConst = doubleDifferences(iObs).constel;
%     fArgs.pivSatPrn = doubleDifferences(iObs).pivSatPrn;
%     fArgs.varSatPrn = doubleDifferences(iObs).varSatPrn;
%     %     fArgs.statPos = statPos;
%     % Measurement model arguments
%     hArgs.x0 = x0;
%     hArgs.statPos = statPos;
%     hArgs.obsConst = doubleDifferences(iObs).constel;
%     hArgs.freqHz = doubleDifferences(iObs).freqHz;
%     hArgs.pivSatPrn = doubleDifferences(iObs).pivSatPrn;
%     hArgs.varSatPrn = doubleDifferences(iObs).varSatPrn;
%     hArgs.pivSatPos = doubleDifferences(iObs).pivSatPos;
%     hArgs.varSatPos = doubleDifferences(iObs).varSatPos;
%     hArgs.satElDeg = [doubleDifferences(iObs).pivSatElDeg
%         doubleDifferences(iObs).varSatElDeg];
%
%     %% Phase DD observation
%     if ~isnan(doubleDifferences(iObs).L) && ...
%             (doubleDifferences(iObs).pivSatSigmaL + doubleDifferences(iObs).varSatSigmaL) < Constants.MAX_L_SIGMA && ...
%             (doubleDifferences(iObs).pivSatSigmaL + doubleDifferences(iObs).varSatSigmaL) > Constants.MIN_L_SIGMA
%
%         idxStatePivSat = PVTUtils.getStateIndex(PVTUtils.ID_SD_AMBIGUITY, hArgs.pivSatPrn, hArgs.obsConst, hArgs.freqHz);
%         idxStateVarSat = PVTUtils.getStateIndex(PVTUtils.ID_SD_AMBIGUITY, hArgs.varSatPrn, hArgs.obsConst, hArgs.freqHz);
%         % Ambiguities: set to CMC if it's 0 (not estimated yet for this sat)
%         % TODO: what if N is estimated as 0
%         if ekf.x(idxStatePivSat) == 0, ekf.x(idxStatePivSat) = doubleDifferences(iObs).pivSatCmcSd; end
%         if ekf.x(idxStateVarSat) == 0, ekf.x(idxStateVarSat) = doubleDifferences(iObs).varSatCmcSd; end
%         % Reinitialize ambiguity if LLI is on (pivot sat has always LLI = 0)
%         if doubleDifferences(iObs).varSatIsLLI
%             ekf.x(idxStateVarSat) = doubleDifferences(iObs).varSatCmcSd;
%             ekf.P(idxStateVarSat, idxStateVarSat) = config.SIGMA_P0_SD_AMBIG^2;
%         end
%
%         hArgs.obs = doubleDifferences(iObs).L;
%         hArgs.sigmaObs = [doubleDifferences(iObs).pivSatSigmaL
%             doubleDifferences(iObs).varSatSigmaL];
%
%         % Label to show on console when outliers are detected
%         label = sprintf('Phase DD (%c%d-%c%d, f = %g)', ...
%             doubleDifferences(iObs).constel,        ...
%             doubleDifferences(iObs).pivSatPrn,      ...
%             doubleDifferences(iObs).constel,        ...
%             doubleDifferences(iObs).varSatPrn,      ...
%             doubleDifferences(iObs).freqHz);
%         % Process code observation
%         [ekf, innovation, innovationCovariance, isPhsRejections(iObs), ~, ~] = ...
%             EKF.processObservation(ekf, thisUtcSeconds,           ...
%             @fTransition, fArgs,                                    ...
%             @hPhaseDD, hArgs,                                        ...
%             label);
%
%         % If Phase DD is rejected, reinitialize ambiguity estimations and
%         % covariances
%         if isPhsRejections(iObs)
%             lambda = Constants.CELERITY / hArgs.freqHz;
%             ekf.x(idxStateVarSat) = -doubleDifferences(iObs).varSatCmcSd / lambda;
%             ekf.P(idxStateVarSat, idxStateVarSat) = config.SIGMA_P0_SD_AMBIG^2;
%         end
%
%         result.phsInnovations(idxSat, idxEst) = innovation;
%         result.phsInnovationCovariances(idxSat, idxEst) = innovationCovariance;
%
%         % Update total-state with absolute position
%         x0 = updateTotalState(ekf.x, statPos);
%     else
%         isPhsRejections(iObs) = 1;
%     end
% end
% result.phsRejectedHist(idxEst) = sum(isPhsRejections);
% end %end of function updateWithPhaseDD
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [z, y, H, R] = hCodeDD(~, hArgs)
% HCODEDD provides the measurement model for the sequential code
% double-differenced observations
z = hArgs.z;
y = hArgs.y;
H = hArgs.H;
R = hArgs.R;
end %end of function hCodeDD
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [z, y, H, R] = hPhaseDD(~, hArgs)
% HCODEDD provides the measurement model for the sequential code
% double-differenced observations
z = hArgs.z;
y = hArgs.y;
H = hArgs.H;
R = hArgs.R;
end %end of function hPhaseDD
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%