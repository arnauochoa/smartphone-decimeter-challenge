function [estPosLla, resultsFilePath] = saveResults(result)
%SAVERESULTS Summary of this function goes here
%   Detailed explanation goes here

config = Config.getInstance;
assert(size(result.xEst, 2) == length(result.utcSeconds), 'Inputs do not have the same number of epochs');

%% Initializations
idxStatePos = PVTUtils.getStateIndex(PVTUtils.ID_POS);
resultsDir = [workspacePath 'data' filesep 'results' filesep config.DATASET_TYPE filesep];
switch config.EVALUATE_DATASETS
    case 'single'
        resultsDir = [resultsDir config.campaignName filesep];
        resultsFilename = [config.phoneName '_' config.RES_FILENAME '_' config.resFileTimestamp '.csv'];
    case 'all'
        resultsDir = [resultsDir 'all' filesep];
        resultsFilename = [config.RES_FILENAME '_' config.resFileTimestamp '.csv'];
    otherwise
        error('Invalid field for Config.EVALUATE_DATASETS, choose among ''single'' and ''all''');
end

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
resultsFilePath = writeResult(resultsDir, resultsFilename, millisSinceGpsEpoch, estPosLla);

%% Interpolate to match sample submission time
if strcmp(config.EVALUATE_DATASETS, 'all') && strcmp(config.DATASET_TYPE, 'test')
    resultsFilename = [config.RES_FILENAME '_interp_' config.resFileTimestamp '.csv'];
    refTable = readtable('data/sample_submission.csv');
    refTableThis = refTable(strcmp(refTable.phone, [config.campaignName '_' config.phoneName]), :);

    estPosLlaInt = interp1(millisSinceGpsEpoch, estPosLla, refTableThis.millisSinceGpsEpoch, 'spline', 'extrap');
    resultsFilePath = writeResult(resultsDir, resultsFilename, refTableThis.millisSinceGpsEpoch, estPosLlaInt);
end
end


function resultsFilePath = writeResult(resultsDir, resultsFilename, millisSinceGpsEpoch, estPosLla)
config = Config.getInstance;
resultsHeader = 'phone,millisSinceGpsEpoch,latDeg,lngDeg\n';

% Check if folder exists, otherwise create it
if ~exist(resultsDir, 'dir'), mkdir(resultsDir); end
resultsFilePath = [resultsDir resultsFilename];
% Flag to write header if file does not exist
writeHeader = ~exist(resultsFilePath, 'file');
% Open file in append mode
fid = fopen(resultsFilePath, 'a');
% Write header if necessary
if writeHeader, fprintf(fid, resultsHeader); end
for iEpoch = 1:length(millisSinceGpsEpoch)
    fprintf(fid, '%s_%s,', config.campaignName, config.phoneName);      % phone
    fprintf(fid, '%d,', millisSinceGpsEpoch(iEpoch));                   % millisSinceGpsEpoch
    fprintf(fid, '%.15f,%.14f', estPosLla(iEpoch, 1), estPosLla(iEpoch, 2)); % latDeg,lngDeg
    fprintf(fid, '\n');
end
fclose(fid);
end
