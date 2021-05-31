function [R] = computeRtkMeasCovariance(elev, sigmas, defaultSigma, constel)
%COMPUTERTKMEASCOVARIANCE Compute the measurement covariance for a RTK
%double-difference observation (only for sequential updates)
constCovFactors = Config.CONST_COV_FACTORS;
if length(Config.CONSTELLATIONS) ~= length(constCovFactors)
    warning('Config.CONSTELLATIONS and Config.CONST_COV_FACTORS do not have the same length. Setting all covariance factors to 1.')
    constCovFactors = ones(size(Config.CONSTELLATIONS));
end

covWeight = constCovFactors(constel == Config.CONSTELLATIONS);

switch Config.MEAS_COV_SRC
    case 'elevation'
        variances = covWeight .* ((defaultSigma ./ sind(elev)).^2);
    case 'uncertainty'
        idxInv = isnan(sigmas);
        if any(idxInv)
            warning('Some uncertainties are nan, using elevation model for these uncertainties');
            sigmas(idxInv) = defaultSigma ./ sind(elev(idxInv));
        end
        variances = covWeight .* (sigmas.^2);
    otherwise
        error('The selected Config.MEAS_COV_SRC is not valid.');
end
R = sum(variances);
end

