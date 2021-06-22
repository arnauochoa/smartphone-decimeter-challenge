function [obsGnss, sat] = ...
    filterObs(obsGnss, satPos, satClkBias, satClkDrift, satVel, rxPos)
% FILTEROBS Removes invalid observations and applies elevation filter

config = Config.getInstance;

% check if ephemeris is available
[obsGnss, satPos, satClkBias, idxRmv] = ...
    clean_obs_vector(obsGnss,satPos,satClkBias);
satVel(:, idxRmv) = [];
satClkDrift(idxRmv) = [];


% apply elevation mask
[obsGnss, satPos, satClkBias, idxMasked] = ...
    apply_elevation_mask(obsGnss, rxPos, satPos, satClkBias, config.ELEVATION_MASK);
satVel(:, idxMasked) = [];
satClkDrift(idxMasked) = [];

% filter by uncertainty
% idxRmv = [obsGnss(:).C_sigma] > 100;
% obsGnss(idxRmv) = [];
% satPos(:, idxRmv) = [];
% satVel(:, idxRmv) = [];
% satClkBias(idxRmv) = [];
% satClkDrift(idxRmv) = [];

sat.pos = satPos;
sat.vel = satVel;
sat.clkBias = satClkBias;
sat.clkDrift = satClkDrift;
end