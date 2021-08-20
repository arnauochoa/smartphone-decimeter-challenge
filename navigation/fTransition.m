% State transition function
function [x2, F, Q] = fTransition(x1, t1, t2, fArgs)
% Initializations
config = Config.getInstance;
nPhones = length(config.phoneNames);
idxStatePos = PVTUtils.getStateIndex(PVTUtils.ID_POS);
idxStateVel = PVTUtils.getStateIndex(PVTUtils.ID_VEL);
idxStateClkDrift = PVTUtils.getStateIndex(PVTUtils.ID_CLK_DRIFT, 1:nPhones);
idxStateIFClkDrift = PVTUtils.getStateIndex(PVTUtils.ID_IF_CLK_DRIFT, 1:nPhones);
idxStateISClkDrift = PVTUtils.getStateIndex(PVTUtils.ID_IS_CLK_DRIFT, 1:nPhones);
idxStateAllSdAmb = PVTUtils.getStateIndex(PVTUtils.ID_SD_AMBIGUITY, 1:nPhones);

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
% Q(idxStatePos, idxStatePos) = 10*eye(3);
Q(idxStateVel, idxStateVel) = diag(config.SIGMA_Q_VEL_XYZ.^2);
if ~isempty(idxStateClkDrift)
    Q(idxStateClkDrift, idxStateClkDrift) = config.SIGMA_Q_CLK_DRIFT.^2;
end
if ~isempty(idxStateIFClkDrift)
    Q(idxStateIFClkDrift, idxStateIFClkDrift) = config.SIGMA_Q_IS_CLK_DRIFT.^2;
end
if ~isempty(idxStateISClkDrift)
    Q(idxStateISClkDrift, idxStateISClkDrift) = config.SIGMA_Q_IS_CLK_DRIFT.^2;
end
if ~isempty(idxStateAllSdAmb)
    Q(idxStateAllSdAmb, idxStateAllSdAmb) = (config.SIGMA_Q_SD_AMBIG.^2) * eye(length(idxStateAllSdAmb));
end
end %end of function fTransition
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%