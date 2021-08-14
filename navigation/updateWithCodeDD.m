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
        
        if config.MULTI_RX
            hCodeDD = @hCodeDDMultiRx;
        else
            hCodeDD = @hCodeDDSingleRx;
        end
        
        % Process code observation
        [ekf, innovation, innovationCovariance, isPrRejected(iObs), ~, ~] = ...
            EKF.processObservation(ekf, thisUtcSeconds,                     ...
            @fTransition, fArgs,                                            ...
            hCodeDD, hArgs,                                                ...
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

function [z, y, H, R] = hCodeDDSingleRx(~, hArgs)
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

function [z, y, H, R] = hCodeDDMultiRx(~, hArgs)
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
% Rx = [1 0 0; 0 cos(pitch) -sin(pitch); 0 sin(pitch) cos(pitch)];
Ry = [cos(roll) 0 sin(roll); 0 1 0; -sin(roll) 0 cos(roll)];
Rz = [cos(yaw) -sin(yaw) 0; sin(yaw) cos(yaw) 0; 0 0 1];
Rbody2ltp = Rz * Ry;%Rx * Rz * Ry;
phoneGeometry = ddLosVec' * Rbody2ltp * hArgs.phoneInfo.posBody;

% Observation estimation
y = norm(hArgs.statPos - hArgs.pivSatPos)   - ...   % |stat - sat1|
    norm(rxPos - hArgs.pivSatPos)           - ...   % |user - sat1|
    norm(hArgs.statPos - hArgs.varSatPos)   + ...   % |stat - sat2|
    norm(rxPos - hArgs.varSatPos)           - ...   % |user - sat2|
    phoneGeometry;                                  % geometry

% Jacobian matrix
H = zeros(1, PVTUtils.getNumStates);
H(idxStatePos) = ddLosVec;

% LTP(ENU)->ECEF rotation matrix
Rned2enu = [0 1 0; 1 0 0; 0 0 -1];
Rltp2ecef = Rned2enu * compute_Rn2e(rxPos(1), rxPos(2), rxPos(3));

% Body->LTP rotation matrix differentiated over roll
Rbody2ltp_diffRoll = ...
    [-cos(roll)*sin(yaw)    0   cos(roll)*cos(yaw); ...
    -sin(roll)*sin(yaw)     0   cos(roll)*sin(yaw); ...
    -cos(yaw)               0   -sin(yaw)];

% Body->LTP rotation matrix differentiated over yaw
Rbody2ltp_diffYaw = ...
    [-sin(roll)*cos(yaw)    -cos(roll)  -sin(roll)*sin(yaw);    ...
    cos(roll)*cos(yaw)      -sin(roll)  cos(roll)*sin(yaw);     ...
    0                       0           0];

H(idxStateAttXyz(2)) = -ddLosVec' * Rltp2ecef * Rbody2ltp_diffRoll * hArgs.phoneInfo.posBody;
H(idxStateAttXyz(3)) = -ddLosVec' * Rltp2ecef * Rbody2ltp_diffYaw * hArgs.phoneInfo.posBody;

% Measurement covariance matrix, consider DD sigmas
R = config.COV_FACTOR_C * computeRtkMeasCovariance(hArgs.satElDeg, ...
    hArgs.sigmaObs, config.SIGMA_C_M, hArgs.obsConst);
end %end of function hCodeDD
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%