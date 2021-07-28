function [x0, ekf, result] = updateWithDD(phoneInfo, x0, ekf, thisUtcSeconds, idxEst, statPos, doubleDifferences, result)
% UPDATEWITHDD Performs the KF update with the double differenced
% observations

% Initializations
config = Config.getInstance;
% Sequentally update with all DDs
if config.USE_CODE_DD
    [x0, ekf, result] = updateWithCodeDD(phoneInfo, x0, ekf, thisUtcSeconds, idxEst, statPos, doubleDifferences, result);
    result.prNumDD(idxEst) = length([doubleDifferences(:).C]);
end

if config.USE_PHASE_DD% && phoneInfo.idx == 1
    [x0, ekf, result, isPhsRejections] = updateWithPhaseDD(phoneInfo, x0, ekf, thisUtcSeconds, idxEst, statPos, doubleDifferences, result);
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
                phoneInfo.idx, ...
                doubleDifferences(idxConstFreq(1)).pivSatPrn,   ... % PRN of piv sat for this const+freq
                doubleDifferences(idxConstFreq(1)).constel,     ... % Constellation
                constFreqs(i, 2));                                  % Freq of this set of DDs
            lambda = Constants.CELERITY / constFreqs(i, 2);
            ekf.x(idxStatePivSat) = - doubleDifferences(idxConstFreq(1)).pivSatCmcSd / lambda;
            ekf.P(idxStatePivSat, idxStatePivSat) = config.SIGMA_P0_SD_AMBIG^2;
            [x0, ekf, result] = updateWithPhaseDD(phoneInfo, x0, ekf, thisUtcSeconds, idxEst, statPos, doubleDifferences, result);
        end
    end
    result.phsNumDD(idxEst) = length([doubleDifferences(:).L]);
end
end %end of function updateWithDD
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [x0, ekf, result] = updateWithCodeDD(phoneInfo, x0, ekf, thisUtcSeconds, idxEst, statPos, doubleDifferences, result)
% UPDATEWITHCODEDD performs the KF sequential update with all code DD's
config = Config.getInstance;
isPrRejected = zeros(1, length(doubleDifferences));
isPrInvalid = zeros(1, length(doubleDifferences));

% Phone information for measurement model
hArgs.phoneInfo  = phoneInfo;
for iObs = 1:length(doubleDifferences)
    % Transition model arguments
    fArgs.x0        = x0;
    fArgs.obsConst  = doubleDifferences(iObs).constel;
    fArgs.pivSatPrn = doubleDifferences(iObs).pivSatPrn;
    fArgs.varSatPrn = doubleDifferences(iObs).varSatPrn;
    %     fArgs.statPos = statPos;
    % Measurement model arguments
    hArgs.x0        = x0;
    hArgs.statPos   = statPos;
    hArgs.obsConst  = doubleDifferences(iObs).constel;
    hArgs.freqHz    = doubleDifferences(iObs).freqHz;
    hArgs.pivSatPrn = doubleDifferences(iObs).pivSatPrn;
    hArgs.varSatPrn = doubleDifferences(iObs).varSatPrn;
    hArgs.pivSatPos = doubleDifferences(iObs).pivSatPos;
    hArgs.varSatPos = doubleDifferences(iObs).varSatPos;
    hArgs.satElDeg  = [doubleDifferences(iObs).pivSatElDeg
        doubleDifferences(iObs).varSatElDeg];
    
    %% Code DD observation
    sigmaDD = doubleDifferences(iObs).pivSatSigmaC + doubleDifferences(iObs).varSatSigmaC;
    if ~isnan(doubleDifferences(iObs).C)    && ...
            sigmaDD > Constants.MIN_C_SIGMA && ...
            sigmaDD < Constants.MAX_C_SIGMA
        
        idxSat = PVTUtils.getSatFreqIndex(      ...
            doubleDifferences(iObs).varSatPrn,  ...
            doubleDifferences(iObs).constel,    ...
            doubleDifferences(iObs).freqHz);
        
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
        [ekf, innovation, innovationCovariance, isPrRejected(iObs), ~, ~] = ...
            EKF.processObservation(ekf, thisUtcSeconds,                     ...
            @fTransition, fArgs,                                            ...
            @hCodeDD, hArgs,                                                ...
            label);
        
        result.prInnovations(idxSat, idxEst, phoneInfo.idx) = innovation;
        result.prInnovationCovariances(idxSat, idxEst, phoneInfo.idx) = innovationCovariance;
        
        % Update total-state with absolute position
        x0 = updateTotalState(ekf.x, statPos);
    else
        isPrInvalid(iObs) = 1;
    end
end
result.prRejectedHist(idxEst) = sum(isPrRejected);
result.prInvalidHist(idxEst) = sum(isPrInvalid);
end %end of function updateWithCodeDD
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [z, y, H, R] = hCodeDD(~, hArgs)
% HCODEDD provides the measurement model for the sequential code
% double-differenced observations

% Initializations
config          = Config.getInstance;
idxStatePos     = PVTUtils.getStateIndex(PVTUtils.ID_POS);
idxStateAttXyz  = PVTUtils.getStateIndex(PVTUtils.ID_ATT_XYZ);
rxPos           = hArgs.x0(idxStatePos);
pitch           = hArgs.x0(idxStateAttXyz(1));
roll            = hArgs.x0(idxStateAttXyz(2));
yaw             = hArgs.x0(idxStateAttXyz(3));

% Observation
z = hArgs.obs;

% Difference between LOS vectors of satellites towards receiver
pivSatLosVec = unitVector(hArgs.statPos - hArgs.pivSatPos);
varSatLosVec = unitVector(hArgs.statPos - hArgs.varSatPos);
ddLosVec = varSatLosVec - pivSatLosVec;

% Term to account for geometry of smartphones
Rx = [1 0 0; 0 cos(pitch) -sin(pitch); 0 sin(pitch) cos(pitch)];
Ry = [cos(roll) 0 sin(roll); 0 1 0; -sin(roll) 0 cos(roll)];
Rz = [cos(yaw) -sin(yaw) 0; sin(yaw) cos(yaw) 0; 0 0 1];
rotBody2Ltp = Rx * Rz * Ry;
phoneGeometry = ddLosVec' * rotBody2Ltp * hArgs.phoneInfo.posBody;

% Observation estimation
y = norm(hArgs.statPos - hArgs.pivSatPos)   - ...   % |stat - sat1|
    norm(rxPos - hArgs.pivSatPos)           - ...   % |user - sat1|
    norm(hArgs.statPos - hArgs.varSatPos)   + ...   % |stat - sat2|
    norm(rxPos - hArgs.varSatPos)           - ...   % |user - sat2|
    phoneGeometry;                                  % geometry

% Jacobian matrix
H = zeros(1, PVTUtils.getNumStates);
H(idxStatePos) = ddLosVec;
H(idxStateAttXyz(2)) = -ddLosVec' * norm(hArgs.phoneInfo.posBody) * ...         TODO: generalize 3 angles in state vector
    [cos(yaw)*sin(roll); sin(yaw)*sin(roll); cos(roll)];
H(idxStateAttXyz(3)) = -ddLosVec' *  norm(hArgs.phoneInfo.posBody) * ...        TODO: generalize 3 angles in state vector
    [sin(yaw)*cos(roll); -cos(yaw)*cos(roll); 0];

% Measurement covariance matrix, consider DD sigmas
R = config.COV_FACTOR_C * computeRtkMeasCovariance(hArgs.satElDeg, ...
    hArgs.sigmaObs, config.SIGMA_C_M, hArgs.obsConst);
end %end of function hCodeDD
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [x0, ekf, result, isPhsRejected] = updateWithPhaseDD(phoneInfo, x0, ekf, thisUtcSeconds, idxEst, statPos, doubleDifferences, result)
% UPDATEWITHPHASEDD performs the KF sequential update with all phase DD's
config = Config.getInstance;
isPhsRejected = zeros(1, length(doubleDifferences));
isPhsInvalid = zeros(1, length(doubleDifferences));

% Phone information for measurement model
hArgs.phoneInfo  = phoneInfo;
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
    sigmaDD = doubleDifferences(iObs).pivSatSigmaL + doubleDifferences(iObs).varSatSigmaL;
    if ~isnan(doubleDifferences(iObs).L)    && ...
            sigmaDD > Constants.MIN_L_SIGMA && ...
            sigmaDD < Constants.MAX_L_SIGMA
        
        idxStatePivSat = PVTUtils.getStateIndex(PVTUtils.ID_SD_AMBIGUITY, phoneInfo.idx, hArgs.pivSatPrn, hArgs.obsConst, hArgs.freqHz);
        idxStateVarSat = PVTUtils.getStateIndex(PVTUtils.ID_SD_AMBIGUITY, phoneInfo.idx, hArgs.varSatPrn, hArgs.obsConst, hArgs.freqHz);
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
        [ekf, innovation, innovationCovariance, isPhsRejected(iObs), ~, ~] =    ...
            EKF.processObservation(ekf, thisUtcSeconds,                         ...
            @fTransition, fArgs,                                                ...
            @hPhaseDD, hArgs,                                                   ...
            label);
        
        % If Phase DD is rejected, reinitialize ambiguity estimations and
        % covariances
        if isPhsRejected(iObs)
            lambda = Constants.CELERITY / hArgs.freqHz;
            ekf.x(idxStateVarSat) = -doubleDifferences(iObs).varSatCmcSd / lambda;
            ekf.P(idxStateVarSat, idxStateVarSat) = config.SIGMA_P0_SD_AMBIG^2;
        end
        
        result.phsInnovations(idxSat, idxEst, phoneInfo.idx) = innovation;
        result.phsInnovationCovariances(idxSat, idxEst, phoneInfo.idx) = innovationCovariance;
        
        % Update total-state with absolute position
        x0 = updateTotalState(ekf.x, statPos);
    else
        isPhsInvalid(iObs) = 1;
    end
end
result.phsRejectedHist(idxEst) = sum(isPhsRejected);
result.phsInvalidHist(idxEst) = sum(isPhsInvalid);
end %end of function updateWithPhaseDD
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [z, y, H, R] = hPhaseDD(x, hArgs)
% HCODEDD provides the measurement model for the sequential code
% double-differenced observations

% Initializations
config          = Config.getInstance;
idxStatePos     = PVTUtils.getStateIndex(PVTUtils.ID_POS);
idxStateAttXyz  = PVTUtils.getStateIndex(PVTUtils.ID_ATT_XYZ);
idxStatePivSat  = PVTUtils.getStateIndex(PVTUtils.ID_SD_AMBIGUITY, hArgs.phoneInfo.idx, hArgs.pivSatPrn, hArgs.obsConst, hArgs.freqHz);
idxStateVarSat  = PVTUtils.getStateIndex(PVTUtils.ID_SD_AMBIGUITY, hArgs.phoneInfo.idx, hArgs.varSatPrn, hArgs.obsConst, hArgs.freqHz);
rxPos           = hArgs.x0(idxStatePos);
pitch           = hArgs.x0(idxStateAttXyz(1));
roll            = hArgs.x0(idxStateAttXyz(2));
yaw             = hArgs.x0(idxStateAttXyz(3));
lambda          = Constants.CELERITY / hArgs.freqHz;

% Observation
z = hArgs.obs;

% Difference between LOS vectors of satellites towards receiver
pivSatLosVec = unitVector(hArgs.statPos - hArgs.pivSatPos);
varSatLosVec = unitVector(hArgs.statPos - hArgs.varSatPos);
ddLosVec = varSatLosVec - pivSatLosVec;

% Term to account for geometry of smartphones
Rx = [1 0 0; 0 cos(pitch) -sin(pitch); 0 sin(pitch) cos(pitch)];
Ry = [cos(roll) 0 sin(roll); 0 1 0; -sin(roll) 0 cos(roll)];
Rz = [cos(yaw) -sin(yaw) 0; sin(yaw) cos(yaw) 0; 0 0 1];
rotBody2Ltp = Rx * Rz * Ry;
phoneGeometry = ddLosVec' * rotBody2Ltp * hArgs.phoneInfo.posBody;

% Observation estimation
y = norm(hArgs.statPos - hArgs.pivSatPos)   - ...   % |stat - sat1|
    norm(rxPos - hArgs.pivSatPos)           - ...   % |user - sat1|
    norm(hArgs.statPos - hArgs.varSatPos)   + ...   % |stat - sat2|
    norm(rxPos - hArgs.varSatPos)           - ...   % |user - sat2|
    phoneGeometry                           + ...   % geometry
    lambda * x(idxStatePivSat)              - ...   % lambda * N_piv
    lambda * x(idxStateVarSat);                     % lambda * N_var

% Jacobian matrix
H = zeros(1, PVTUtils.getNumStates);
H(idxStatePos) = ddLosVec;
H(idxStateAttXyz(2)) = -ddLosVec' * norm(hArgs.phoneInfo.posBody) * ...         TODO: generalize 3 angles in state vector
    [cos(yaw)*sin(roll); sin(yaw)*sin(roll); cos(roll)];
H(idxStateAttXyz(3)) = -ddLosVec' *  norm(hArgs.phoneInfo.posBody) * ...        TODO: generalize 3 angles in state vector
    [sin(yaw)*cos(roll); -cos(yaw)*cos(roll); 0];
H(idxStatePivSat) = lambda;
H(idxStateVarSat) = -lambda;

% Measurement covariance matrix, consider DD sigmas
R = config.COV_FACTOR_L * computeRtkMeasCovariance(hArgs.satElDeg, ...
    hArgs.sigmaObs, config.SIGMA_L_M, hArgs.obsConst);
end %end of function hPhaseDD
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%