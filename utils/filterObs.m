function [obsGnss, sat] = ...
    filterObs(obsGnss, satPos, satClkBias, satClkDrift, satVel, rxPos)
% FILTEROBS Removes invalid observations and applies elevation filter

config = Config.getInstance;

% check if ephemeris is available
[obsGnss, satPos, satClkBias, idxRmv] = ...
    clean_obs_vector(obsGnss,satPos,satClkBias);
satClkDrift(idxRmv) = [];
satVel(:, idxRmv) = [];

% apply elevation mask
[obsGnss, satPos, satClkBias, idxMasked] = ...
    apply_elevation_mask(obsGnss, rxPos, satPos, satClkBias, config.ELEVATION_MASK);
satClkDrift(idxMasked) = [];
satVel(:, idxMasked) = [];

sat.pos = satPos;
sat.vel = satVel;
sat.clkBias = satClkBias;
sat.clkDrift = satClkDrift;
end