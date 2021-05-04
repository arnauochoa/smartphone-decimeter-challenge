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

% remove invalid pr? which thresholds ?? -> set as nan (like doppler)

% remove doppler measurements with extremely large values
idxLrg = abs([obsGnss.D_Hz]) > Config.MAX_DOPPLER_MEAS | ...
    [obsGnss.D_sigma] > Config.MAX_DOPPLER_UNCERT;
if any(idxLrg) % Invalid dopplers are set to nan to maintain array size
    [obsGnss(idxLrg).D_Hz] = deal(nan);
end
end