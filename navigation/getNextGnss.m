function [gnssEpoch, osrEpoch, phoneInfo] = getNextGnss(lastUtcSeconds, phones, osr, whichObs)
% GETNEXTGNSS returns a structure with the next GNSS observations 
%
%   gnss = GETNEXTGNSS(lastUtcMillis, gnssRnx, [osrRnx, whichObs])
%
%   This function returns a structure with the next GNSS observations 
%   received at:
%       - the epoch given by lastUtcSeconds (if whichObs = 'this'), or
%       - the next epoch of lastUtcSeconds (if whichObs = 'next')

config = Config.getInstance;
nPhones = length(config.phoneNames);

if ~exist('whichObs', 'var')
    whichObs = 'next';
end
if ~exist('osr', 'var')
    osr = [];
end

nextObsIndices = nan(1, nPhones);
nextObsUtc = nan(1, nPhones);
for iPhone = 1:nPhones
    gnssRnx = phones(iPhone).gnss;
    switch whichObs
        case 'this'
            nextObsIdx = find(gnssRnx.utcSeconds == lastUtcSeconds, 1, 'first');
        case 'next'
            nextObsIdx = find(gnssRnx.utcSeconds > lastUtcSeconds, 1, 'first');
        otherwise
            error('Invalid ''whichObs'' specifier, the possible values are ''this'' and ''next''')
    end
    if ~isempty(nextObsIdx)
        nextObsIndices(iPhone) = nextObsIdx;
        nextObsUtc(iPhone) = gnssRnx.utcSeconds(nextObsIdx);
    end
end

if ~all(isnan(nextObsIndices)) % If there are more observations
    % Find phone with next earlier observation
    [~, idxPhoneNextObs] = min(nextObsUtc);
    gnssRnx = phones(idxPhoneNextObs).gnss;
    nextObsIdx = nextObsIndices(idxPhoneNextObs);
    
    % Fill phone information
    phoneInfo.idx = idxPhoneNextObs;
    phoneInfo.phoneName = config.phoneNames{idxPhoneNextObs};
    phoneInfo.posBody = phones(idxPhoneNextObs).posBody;
    
    % TOW of epoch to return
    nextTow = gnssRnx.obs(nextObsIdx, 2);
    % Obs vector of epoch to return
    gnssEpoch.obs = get_obs_vector(nextTow,  ... 
        config.CONSTELLATIONS,  ...
        config.OBS_USED,        ...
        {},                     ...
        gnssRnx.obs,            ...
        gnssRnx.type);
    % Timestamps of epoch to return
    gnssEpoch.utcSeconds = gnssRnx.utcSeconds(nextObsIdx);
    gnssEpoch.weekN = gnssRnx.obs(nextObsIdx, 1);
    gnssEpoch.tow = nextTow;
    
    if ~isempty(osr)
    osrRnx = osr.interpRnx(idxPhoneNextObs);
    % Obs vector of epoch to return
    osrEpoch.obs = get_obs_vector(nextTow,  ... 
        config.CONSTELLATIONS,  ...
        config.OSR_OBS_USED,    ...
        {},                     ...
        osrRnx.obs,             ...
        osrRnx.type);
    osrEpoch.utcSeconds = gnssEpoch.utcSeconds;
    osrEpoch.weekN = gnssEpoch.weekN;
    osrEpoch.tow = gnssEpoch.tow;
    else
        osrEpoch = [];
    end
else % If there aren't more observations
    gnssEpoch = [];
    osrEpoch = [];
    phoneInfo = [];
end
end