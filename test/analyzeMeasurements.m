clearvars %-except gnssRaw gnssAnalysis accRaw gyrRaw magRaw sensorAnalysis obsRef obsTypeRef obsExtraRef; 
close all; clc;

%% Constants
LIGHTSPEED = 299792458;

%% Configuration
% Measurement campaign name
campaignName = '2021-04-29-US-MTV-1'; % '2021-04-26-US-SVL-1'; % '2021-04-29-US-MTV-1'; %
% Phone name
phoneName = {'Pixel4', 'Pixel5', 'SamsungS20Ultra'}; % {'Mi8', 'Pixel5'}; %{'Pixel4', 'Pixel5', 'SamsungS20Ultra'}; %
% Dataset type
datasetType = 'train';
% Filter flag: set to 1 to apply filters to measurements
filter = 1;

constellations = 'GEC';
% prns = [1 2 3];
obsLabels = {'C1+L1', 'C1+L1', 'C2+L2'};

%% Load dataset
% Dataset path
datasetsPath = [workspacePath 'data/sdc-data/' datasetType filesep];

for i = 1:length(phoneName)
%     dirName = [datasetsPath campaignName '/' phoneName{i} filesep];
%     rawFileName = [phoneName{i} '_GnssLog.txt'];
%     disp('Reading Android Raw Log...')
%     [gnssRaw, gnssAnalysis, accRaw, gyrRaw, magRaw, sensorAnalysis] = readGnssLog(dirName,rawFileName);
%     disp('Processing Android GnssLog...')
%     [phoneGnss(i).obs, phoneGnss(i).obsType] = processGnssRaw(gnssRaw, filter);
    
    dirName = [datasetsPath campaignName '/' phoneName{i} filesep 'supplemental' filesep];
    rawFileName = [phoneName{i} '_GnssLog.21o'];
    [phoneGnss(i).obs, phoneGnss(i).obsType] = rinex_v3_obs_parser([dirName rawFileName]);
end

%% Analysis
% Constellations in each phone
for i = 1:length(phoneName)
    constels = i2c(unique(phoneGnss(i).obs(:,3)));
    fprintf('  %s - %s \n', phoneName{i}, constels);
end

for i = 1:length(phoneName)
    for iConst = 1:length(constellations)
        constel = constellations(iConst);
        obsConstLbls = obsLabels{iConst};
        obsConstLblSplit = split(obsConstLbls, '+');
        for iObs = 1:length(obsConstLblSplit)
            obsConstLbl = obsConstLblSplit{iObs};
%             uncConstLbl = ['U' obsConstLbl(2) obsConstLbl(1)];
            obsCol = 4 + (strfind(phoneGnss(i).obsType(c2i(constel)).type_str, obsConstLbl)-1)/3 + 1;
%             uncCol = 4 + (strfind(phoneGnss(i).obsType(c2i(constel)).type_str, uncConstLbl)-1)/3 + 1;
            isConstel = phoneGnss(i).obs(:,3) == c2i(constel);
            prns = unique(phoneGnss(i).obs(isConstel,4))';
            figure; hold on;
            for prn = prns
                idxObsPrn = find(isConstel & phoneGnss(i).obs(:,4) == prn);
                timeLine = phoneGnss(i).obs(idxObsPrn, 2) - phoneGnss(i).obs(1, 2);
                obsVec = phoneGnss(i).obs(idxObsPrn, obsCol);
%                 uncVec = phoneGnss(i).obs(idxObsPrn, uncCol);
                subplot(2, 1, 1); hold on;
                plot(timeLine, obsVec);
                xlabel('Time since start (s)');
                ylabel(obsConstLbl);
%                 subplot(2, 1, 2); hold on;
%                 plot(timeLine, uncVec);
%                 xlabel('Time since start (s)');
%                 ylabel(uncConstLbl);
            end
            subplot(2, 1, 1);
            title([phoneName{i} ' - ' constel]);
        end
    end
end