function estPosLla = saveResults(xEst, utcSecondsHist)
%SAVERESULTS Summary of this function goes here
%   Detailed explanation goes here
nEpochs = length(utcSecondsHist);
assert(size(xEst, 2) == nEpochs, 'Inputs do not have the same number of epochs');

%% Initializations
idxStatePos = PVTUtils.getStateIndex(PVTUtils.ID_POS);
resultsPath = [workspacePath 'data' filesep 'results' filesep Config.DATASET_TYPE filesep];
resultsHeader = 'phone,millisSinceGpsEpoch,latDeg,lngDeg\n';

%% Data preparation
% Convert UTC (sec) timestamp to GPS (millis)
utcDateVec = datevec(Constants.DAYS_TO_UTC+utcSecondsHist/Constants.SECONDS_IN_DAY);
[~, secondsSinceGpsEpoch, ~] = Utc2Gps(utcDateVec);
millisSinceGpsEpoch = secondsSinceGpsEpoch * 1e3;

% Convert estimated positions from ECEF to Geodetic
[posLat, posLon, posAlt] = ecef2geodetic(wgs84Ellipsoid, ...
    xEst(idxStatePos(1), :)', ...
    xEst(idxStatePos(2), :)', ...
    xEst(idxStatePos(3), :)');
estPosLla = [posLat, posLon, posAlt];

%% Write data to file
% Check if folder exists, otherwise create it
if ~exist(resultsPath, 'dir'), mkdir(resultsPath); end
fid = fopen([resultsPath Config.RES_FILENAME], 'w');
fprintf(fid, resultsHeader);
for iEpoch = 1:nEpochs
    fprintf(fid, '%s_%s,', Config.CAMPAIGN_NAME, Config.PHONE_NAME);    % phone
    fprintf(fid, '%d,', millisSinceGpsEpoch(iEpoch));                   % millisSinceGpsEpoch
    fprintf(fid, '%.15f,%.14f', estPosLla(iEpoch, 1), estPosLla(iEpoch, 2)); % latDeg,lngDeg
    fprintf(fid, '\n');
end
fclose(fid);
end

