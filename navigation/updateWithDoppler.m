function [x0, ekf, result] = updateWithDoppler(x0, ekf, thisUtcSeconds, idxEst, statPos, phoneGnss, sat, result)
% UPDATEWITHDD Performs the KF update with the double differenced
% observations

% Initializations

% Sequentally update with all Doppler observations
for iObs = 1:length(phoneGnss.obs)
    thisObs = phoneGnss.obs(iObs);
    idxSat = PVTUtils.getSatelliteIndex(thisObs.prn, thisObs.constellation);
    if ~isempty(thisObs.D_Hz) && ~isnan(thisObs.D_Hz)
        % Pack arguments that are common for all observations
        fArgs.x0 = x0;
        fArgs.statPos = statPos;
        hArgs.x0 = x0;
        hArgs.satPos = sat.pos(:, iObs);
        hArgs.satVel = sat.vel(:, iObs);
        hArgs.satClkDrift = sat.clkDrift(iObs);
        hArgs.satElDeg = sat.elDeg(iObs);
        hArgs.obsConst = thisObs.constellation;

        % Convert Doppler from Hz to mps (pseudorange-rate)
        hArgs.obs = -hz2mps(thisObs.D_Hz, thisObs.D_fcarrier_Hz);
        hArgs.sigmaObs = hz2mps(thisObs.D_sigma, thisObs.D_fcarrier_Hz);
        
        % Label to show on console when outliers are detected
        label = sprintf('Doppler (%c%d, f = %g)',   ...
            thisObs.constellation,                  ...
            thisObs.prn,                      ...
            thisObs.D_fcarrier_Hz);
        
        % Process code observation
        [ekf, innovation, innovationCovariance, rejected, ~, ~] = ...
            EKF.processObservation(ekf, thisUtcSeconds,           ...
            @fTransition, fArgs,                                    ...
            @hDoppler, hArgs,                                        ...
            label);
        
        result.dopInnovations(idxSat, idxEst) = innovation;
        result.dopInnovationCovariances(idxSat, idxEst) = innovationCovariance;
        result.dopRejectedHist(idxEst) = result.dopRejectedHist(idxEst) + rejected;
        
        % Update total-state with absolute position
        x0 = updateTotalState(ekf.x, statPos);
    end
end
result.dopRejectedHist(idxEst) = 100*result.dopRejectedHist(idxEst) / length(phoneGnss.obs);
end %end of function updateWithDoppler
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [z, y, H, R] = hDoppler(~, hArgs)
% HDOPPLER provides the measurement model for the sequential Doppler
% observations

% Initializations
idxStatePos = PVTUtils.getStateIndex(PVTUtils.ID_POS);
idxStateVel = PVTUtils.getStateIndex(PVTUtils.ID_VEL);
idxStateClkDrift = PVTUtils.getStateIndex(PVTUtils.ID_CLK_DRIFT);

% Observation
z = hArgs.obs;

% Distance between receiver and satellite. Second term is Sagnac effect.
dist = norm(hArgs.x0(idxStatePos) - hArgs.satPos) + ...
    Constants.OMEGA_E/Constants.CELERITY *          ...
    ( hArgs.satPos(1) * hArgs.x0(idxStatePos(2)) -  ...
    hArgs.satPos(2) * hArgs.x0(idxStatePos(1)) );

% Unit vector from user to satellite
vUS = unitVector(hArgs.satPos - hArgs.x0(idxStatePos));

% Observation estimation
y = dot(hArgs.satVel - hArgs.x0(idxStateVel), vUS) + ...    % Radial velocity from user to sat
    hArgs.x0(idxStateClkDrift) -                     ...    % Receiver clock drift
    Constants.CELERITY * hArgs.satClkDrift;                 % Satellite clock drift

% Jacobian matrix
H = zeros(1, PVTUtils.getNumStates);
H(idxStateVel) = (hArgs.x0(idxStatePos) - hArgs.satPos)/dist;
H(idxStateClkDrift) = 1;

% Measurement covariance matrix
R = Config.COV_FACTOR_D * computeMeasCovariance(hArgs.satElDeg, ...
    hArgs.sigmaObs, Config.SIGMA_D_MPS, hArgs.obsConst);
end %end of function hDopplerObs
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
