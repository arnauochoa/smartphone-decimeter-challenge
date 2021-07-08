function [x0, ekf, result, thisUtcSeconds] = updateWithRefPos(x0, ekf, thisUtcSeconds, idxEst, ref, osrRnx, result)
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
    label,                                                  ...
    config.SEQUENTIAL_UPDATE);
x0 = updateTotalState(ekf.x, osrRnx.statPos);
result.refRejectedHist(idxEst) = rejected;
end


function [z, y, H, R] = hRefObs(~, hArgs)
idxStatePos = PVTUtils.getStateIndex(PVTUtils.ID_POS);

z = hArgs.obs;
y = hArgs.x0(idxStatePos) - hArgs.statPos;
% Jacobian matrix
H = zeros(3, PVTUtils.getNumStates);
H(idxStatePos,idxStatePos) = eye(3);
R = 1e-1*eye(3);
end