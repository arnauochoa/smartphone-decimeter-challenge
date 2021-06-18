function [x0, ekf, result, thisUtcSeconds] = updateWithRefPos(x0, ekf, thisUtcSeconds, ref, osrRnx, result)
% UPDATEWITHDD Performs the KF update with the double differenced
% observations

[idxRef] = find(ref.utcSeconds >= thisUtcSeconds, 1, 'first');
fArgs.x0 = x0;
hArgs.x0 = x0;
hArgs.statPos = osrRnx.statPos;
hArgs.obs = geodetic2ecefVector(ref.posLla(idxRef, :))' - hArgs.statPos;
label = 'ref';
[ekf, innovation, innovationCovariance, rejected, ~, ~] =   ...
    EKF.processObservation(ekf, thisUtcSeconds,             ...
    @fTransition, fArgs,                                    ...
    @hRefObs, hArgs,                                        ...
    label);
x0 = updateTotalState(ekf.x, osrRnx.statPos);
end


function [z, y, H, R] = hRefObs(~, hArgs)
idxStatePos = PVTUtils.getStateIndex(PVTUtils.ID_POS);

z = hArgs.obs;
y = hArgs.x0(idxStatePos) - hArgs.statPos;
% Jacobian matrix
H = zeros(3, PVTUtils.getNumStates);
H(idxStatePos,idxStatePos) = eye(3);
R = 0.1*eye(3);
end