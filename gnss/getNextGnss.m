function gnss = getNextGnss(lastUtcSeconds, gnssRnx, whichObs)
% GETNEXTGNSS returns a structure with the next GNSS observations 
%
%   gnss = GETNEXTGNSS(lastUtcMillis, gnssRnx, [whichObs])
%
%   This function returns a structure with the next GNSS observations 
%   received at:
%       - the epoch given by lastUtcSeconds (if whichObs = 'this'), or
%       - the next epoch of lastUtcSeconds (if whichObs = 'next')

if ~exist('whichObs', 'var')
    whichObs = 'next';
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
        Config.CONSTELLATIONS,  ...
        Config.OBS_USED,        ...
        Config.OBS_COMBINATION, ...
        gnssRnx.obs,            ...
        gnssRnx.type);
    % Timestamps of epoch to return
    gnss.utcSeconds = gnssRnx.utcSeconds(firstObsIdx);
    gnss.tow = nextTow;
else % If there aren't more observations
    gnss = [];
end
end