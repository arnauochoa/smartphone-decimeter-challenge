function [score] = computeScore(ref, result)
%COMPUTESCORE Summary of this function goes here
%   Detailed explanation goes here

idxStatePos = PVTUtils.getStateIndex(PVTUtils.ID_POS);
estPosLla = ecef2geodeticVector(result.xRTK(idxStatePos, :)');

% Interpolate groundtruth at the computed position's time
refInterpLla = interp1(ref.utcSeconds, ref.posLla, result.utcSeconds);
assert(size(refInterpLla, 1) == size(result.xRTK, 2), 'Reference and computed position vectors are not the same size');
% Position error
hError = Lla2Hd(refInterpLla, estPosLla);

% Compute score
hErr95 = prctile(abs(hError),95);
hErr50 = prctile(abs(hError),50);
score = mean([hErr95 hErr50]);
end

