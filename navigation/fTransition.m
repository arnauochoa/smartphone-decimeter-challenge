% State transition function
function [x2, F, Q] = fTransition(x1, t1, t2, fArgs)
% Initializations
config = Config.getInstance;
idxStatePos = PVTUtils.getStateIndex(PVTUtils.ID_POS);
idxStateVel = PVTUtils.getStateIndex(PVTUtils.ID_VEL);
idxStateClkDrift = PVTUtils.getStateIndex(PVTUtils.ID_CLK_DRIFT);
idxStateAllSdAmb = PVTUtils.getStateIndex(PVTUtils.ID_SD_AMBIGUITY);

% Transition time step
dt = t2 - t1;

% State transition matrix
F = eye(PVTUtils.getNumStates);
% dp/dv
F(idxStatePos, idxStateVel) = dt * eye(3);

% State prediction
x2 = F * x1;

% Process noise covariance matrix
Q = zeros(PVTUtils.getNumStates);
Q(idxStateVel, idxStateVel) = diag(config.SIGMA_Q_VEL_XYZ.^2);
Q(idxStateClkDrift, idxStateClkDrift) = config.SIGMA_Q_CLK_DRIFT.^2;
Q(idxStateAllSdAmb, idxStateAllSdAmb) = (config.SIGMA_Q_SD_AMBIG.^2) * eye(PVTUtils.getNumSatelliteIndices);
end %end of function fTransition
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%