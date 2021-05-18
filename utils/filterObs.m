function [obsGnss, satPos, satClkBias, satClkDrift, satVel] = ...
    filterObs(obsGnss, satPos, satClkBias, satClkDrift, satVel, rxPos)
% FILTEROBS Removes invalid observations and applies elevation filter

% check if ephemeris is available
[obsGnss, satPos, satClkBias, idxRmv] = ...
    clean_obs_vector(obsGnss,satPos,satClkBias);
satClkDrift(idxRmv) = [];
satVel(:, idxRmv) = [];

% apply elevation mask
[obsGnss, satPos, satClkBias, idxMasked] = ...
    apply_elevation_mask(obsGnss,rxPos,satPos,satClkBias,Config.ELEVATION_MASK);
satClkDrift(idxMasked) = [];
satVel(:, idxMasked) = [];
end