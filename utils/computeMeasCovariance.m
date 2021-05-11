function [R] = computeMeasCovariance(elev, sigmas, defaultSigma, constel)
%COMPUTEMEASCOVARIANCE Summary of this function goes here
%   Detailed explanation goes here
constCovFactors = Config.CONST_COV_FACTORS;
if length(Config.CONSTELLATIONS) ~= length(constCovFactors)
    warning('Config.CONSTELLATIONS and Config.CONST_COV_FACTORS do not have have the same length. Setting all covariance factors to 1.')
    constCovFactors = ones(size(Config.CONSTELLATIONS));
end

covWeights = nan(size(constel));
for iConst = 1:length(Config.CONSTELLATIONS)
    isConst = constel == Config.CONSTELLATIONS(iConst);
    covWeights(isConst) = constCovFactors(iConst);
end

switch Config.MEAS_COV_SRC
    case 'elevation'
        variances = covWeights .* ((defaultSigma ./ sind(elev)).^2);
    case 'uncertainty'
        idxInv = isnan(sigmas);
        if any(idxInv)
            warning('Some uncertainties are nan, using elevation model for these uncertainties');
            sigmas(idxInv) = defaultSigma ./ sind(elev(idxInv));
        end
        variances = covWeights .* (sigmas.^2);
    otherwise
        error('The selected Config.MEAS_COV_SRC is not valid.');
end
R = diag(variances);
end

