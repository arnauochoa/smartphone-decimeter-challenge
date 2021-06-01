function estPosLla = saveResults(result)
%SAVERESULTS Summary of this function goes here
%   Detailed explanation goes here

config = Config.getInstance;
nEpochs = length(result.utcSeconds);
assert(size(result.xEst, 2) == nEpochs, 'Inputs do not have the same number of epochs');

%% Initializations
idxStatePos = PVTUtils.getStateIndex(PVTUtils.ID_POS);
resultsPath = [workspacePath 'data' filesep 'results' filesep config.DATASET_TYPE filesep];
switch config.EVALUATE_DATASETS
    case 'single'
        resultsPath = [resultsPath config.campaignName filesep];
        resultsFilename = [config.phoneName '_' config.RES_FILENAME '_' config.resFileTimestamp '.csv'];
    case 'all'
        resultsPath = [resultsPath 'all' filesep];
        resultsFilename = [config.RES_FILENAME '_' config.resFileTimestamp '.csv'];
    otherwise
        error('Invalid field for Config.EVALUATE_DATASETS, choose among ''single'' and ''all''');
end
resultsHeader = 'phone,millisSinceGpsEpoch,latDeg,lngDeg\n';

%% Data preparation
% Convert UTC (sec) timestamp to GPS (millis)
utcDateVec = utcSeconds2datevec(result.utcSeconds);
[~, secondsSinceGpsEpoch, ~] = Utc2Gps(utcDateVec);
millisSinceGpsEpoch = secondsSinceGpsEpoch * 1e3;

% load('ref_time_2020-05-15-US-MTV-1_Pixel4.mat', 'ref')
% millisSinceGpsEpoch2 = (result.gpsWeekN*7*Constants.SECONDS_IN_DAY + result.gpsTow)*1000;
% dif1 = millisSinceGpsEpoch - ref;
% dif2 = millisSinceGpsEpoch2 - ref;

% Convert estimated positions from ECEF to Geodetic
[posLat, posLon, posAlt] = ecef2geodetic(wgs84Ellipsoid, ...
    result.xEst(idxStatePos(1), :)', ...
    result.xEst(idxStatePos(2), :)', ...
    result.xEst(idxStatePos(3), :)');
estPosLla = [posLat, posLon, posAlt];

%% Write data to file
% Check if folder exists, otherwise create it
if ~exist(resultsPath, 'dir'), mkdir(resultsPath); end
% Flag to write header if file does not exist
writeHeader = ~exist([resultsPath resultsFilename], 'file');
% Open file in append mode
fid = fopen([resultsPath resultsFilename], 'a');
% Write header if necessary
if writeHeader, fprintf(fid, resultsHeader); end
for iEpoch = 1:nEpochs
    fprintf(fid, '%s_%s,', config.campaignName, config.phoneName);      % phone
    fprintf(fid, '%d,', millisSinceGpsEpoch(iEpoch));                   % millisSinceGpsEpoch
    fprintf(fid, '%.15f,%.14f', estPosLla(iEpoch, 1), estPosLla(iEpoch, 2)); % latDeg,lngDeg
    fprintf(fid, '\n');
end
fclose(fid);
end

