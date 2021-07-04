function [Rdd] = buildRdd(idxPivVarR, covPivVarR, numValidDD)
numSD = length(unique(idxPivVarR));
D = zeros(numValidDD, numSD);
for iDD = 1:numValidDD
    D(iDD, idxPivVarR(iDD, 1)) = 1;   % Pivot satellite
    D(iDD, idxPivVarR(iDD, 2)) = -1;  % Variable satellite
end
Rsd = zeros(numSD, numSD);
Rsd(idxPivVarR(:, 1), idxPivVarR(:, 1)) = diag(covPivVarR(:, 1)); % Pivot sat covariances
Rsd(idxPivVarR(:, 2), idxPivVarR(:, 2)) = diag(covPivVarR(:, 2)); % Variable sat covariances
Rdd = D*Rsd*D';
end