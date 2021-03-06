function [gnss, osr] = getNextGnss(lastUtcSeconds, gnssRnx, osrRnx, whichObs)
% GETNEXTGNSS returns a structure with the next GNSS observations 
%
%   gnss = GETNEXTGNSS(lastUtcMillis, gnssRnx, [osrRnx, whichObs])
%
%   This function returns a structure with the next GNSS observations 
%   received at:
%       - the epoch given by lastUtcSeconds (if whichObs = 'this'), or
%       - the next epoch of lastUtcSeconds (if whichObs = 'next')

config = Config.getInstance;

if ~exist('whichObs', 'var')
    whichObs = 'next';
end
if ~exist('osrRnx', 'var')
    osrRnx = [];
end

switch whichObs
    case 'this'
        firstObsIdx = find(gnssRnx.utcSeconds == lastUtcSeconds, 1, 'first');
    case 'next'
        firstObsIdx = find(gnssRnx.utcSeconds > lastUtcSeconds, 1, 'first');
    otherwise
        error('Invalid ''whichObs'' specifier, the possible values are ''this'' and ''next''')
end

if ~isempty(firstObsIdx) % If there are more observations
    % TOW of epoch to return
    nextTow = gnssRnx.obs(firstObsIdx, 2);
    % Obs vector of epoch to return
    gnss.obs = get_obs_vector(nextTow,  ... 
        config.CONSTELLATIONS,  ...
        config.OBS_USED,        ...
        {},                     ...
        gnssRnx.obs,            ...
        gnssRnx.type);
    % Timestamps of epoch to return
    gnss.utcSeconds = gnssRnx.utcSeconds(firstObsIdx);
    gnss.weekN = gnssRnx.obs(firstObsIdx, 1);
    gnss.tow = nextTow;
    
    if ~isempty(osrRnx)
    % Obs vector of epoch to return
    osr.obs = get_obs_vector(nextTow,  ... 
        config.CONSTELLATIONS,  ...
        config.OSR_OBS_USED,    ...
        {},                     ...
        osrRnx.obs,             ...
        osrRnx.type);
    osr.utcSeconds = gnss.utcSeconds;
    osr.weekN = gnss.weekN;
    osr.tow = gnss.tow;
    else
        osr = [];
    end
else % If there aren't more observations
    gnss = [];
    osr = [];
end
end