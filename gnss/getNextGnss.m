function gnss = getNextGnss(lastUtcMillis, gnssRnx, whichObs)
% GETNEXTGNSS returns a structure with the next GNSS observations 
%
%   gnss = GETNEXTGNSS(lastUtcMillis, gnssRnx, [whichObs])
%
%   This function returns a structure with the next GNSS observations 
%   received at the time lastUtcMillis (if whichObs = 'this') or after the 
%   time lastUtcMillis (if whichObs = 'next')

if ~exist('whichObs', 'var')
    whichObs = 'next';
end

switch whichObs
    case 'this'
        nextIdx = find(gnssRnx.utcMillis == lastUtcMillis, 1, 'first');
    case 'next'
        nextIdx = find(gnssRnx.utcMillis > lastUtcMillis, 1, 'first');
    otherwise
        error('Invalid ''whichObs'' specifier, the possible values are ''this'' and ''next''')
end

if ~isempty(nextIdx) % If there are more observations
    nextUtcMillis = gnssRnx.utcMillis(nextIdx);
    nextTow = gnssRnx.obs(nextIdx, 2);
    gnss.obs = get_obs_vector(nextTow,           ...
        Config.CONSTELLATIONS,  ...
        Config.OBS_USED,        ...
        Config.OBS_COMBINATION, ...
        gnssRnx.obs,            ...
        gnssRnx.type);
    gnss.utcMillis = nextUtcMillis;
    gnss.tow = nextTow;
else % If there aren't more observations
    gnss = [];
end
end