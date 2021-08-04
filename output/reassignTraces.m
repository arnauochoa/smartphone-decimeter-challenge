function reassignTraces(matFilePath)
% REASSIGNTRACES Plots the computed trajectories and allows to select which
% traces should be substited by one from another phone
% IMPORTANT: Make sure that the fields EVALUATE_DATASETS and DATASET_TYPE 
% in the configuration file match with the selected matFilePath 
if nargin < 1
    matFilePath = 'data/results/test/all/result_20210804_092740.mat';%'data/results/test/all/result_20210623_091453.mat';
end
load(matFilePath, 'datasetResults');

%% Initialization
isTrain = ~isempty(datasetResults(1).ref);
nTraces = length(datasetResults);
basemap = 'none';
score = [];
[~, filename, ~] = fileparts(matFilePath);
selectedTracesTable = table('Size',[nTraces 3],     ...
    'VariableTypes',{'string','string','logical'},  ...
    'VariableNames',{'Campaign','Phone','IsCorrect'});
fileNamePreamble = 'reassigned_';
config = Config.getInstance;
delete(config); % Delete previous instance of Config
config = Config.getInstance;
config.resFileTimestamp = filename(end-14:end);

%% Trace selection
for iTrace = 1:nTraces
    f = figure;
    if isTrain
        geoplot(datasetResults(iTrace).ref.posLla(:, 1), datasetResults(iTrace).ref.posLla(:, 2), '.-k', ...
            datasetResults(iTrace).result.estPosWLSLla(:, 1), datasetResults(iTrace).result.estPosWLSLla(:, 2), '.b');
    else
        geoplot(datasetResults(iTrace).result.estPosWLSLla(:, 1), datasetResults(iTrace).result.estPosWLSLla(:, 2), '.b');
    end
    geobasemap(basemap);
    % Plot estimation with color depending on covariance
    stdHor = vecnorm(datasetResults(iTrace).result.posStdNed(:, 1:2), 2, 2);
    hold on; colormap summer;
    geoscatter(datasetResults(iTrace).result.estPosLla(:, 1), datasetResults(iTrace).result.estPosLla(:, 2), 5, stdHor, 'fill');
    c = colorbar;
    c.Label.String = 'Horizontal position STD (m)';
    if isTrain, legend('Groundtruth', 'WLS', 'RTK');
    else, legend('WLS', 'RTK'); end
    title = strcat(num2str(iTrace), '_', datasetResults(iTrace).campaignName, '_', datasetResults(iTrace).phoneName);
    figureWindowTitle(f, title);

    answer = '';
    while ~strcmp(answer, 'y') && ~strcmp(answer, 'n')
        answer = input('Is this result correct (y/n) ? ', 's');
    end
    selectedTracesTable.Campaign{iTrace} = datasetResults(iTrace).campaignName;
    selectedTracesTable.Phone{iTrace} = datasetResults(iTrace).phoneName;
    selectedTracesTable.IsCorrect(iTrace) = strcmp(answer, 'y');
    close(f);
end
% selectedTracesTable = readtable('test_selectedTracesTable.txt');


%% Trace reassignation
if isTrain
    resultsDir = getResultsDir(config);
    fidScore = fopen([resultsDir 'score_' config.resFileTimestamp '.csv'], 'w');
    fprintf(fidScore, 'campaign,phone,score\n');
end
for iTrace = 1:nTraces
    config.campaignName = selectedTracesTable.Campaign{iTrace};
    config.phoneNames = selectedTracesTable.Phone(iTrace); % 1x1 cell with char array
    if ~selectedTracesTable.IsCorrect(iTrace)
        thisCampaign = selectedTracesTable.Campaign{iTrace};
        % Find first correct trace in this campaign
        idxAllCorrectTrace = find(strcmp(selectedTracesTable.Campaign, thisCampaign) &    ...
            selectedTracesTable.IsCorrect);
        maxSize = 0;
        for i = idxAllCorrectTrace'
            if maxSize < length(datasetResults(i).result.gpsTow)
                maxSize = length(datasetResults(i).result.gpsTow);
                idxCorrectTrace = i;
            end
        end
        if ~isempty(idxCorrectTrace)
            fprintf('%s: Substituting %s solution by %s solution\n', ...
                thisCampaign, selectedTracesTable.Phone{iTrace}, selectedTracesTable.Phone{idxCorrectTrace});
            
            newResult =                     ...
                reassignResult(datasetResults(iTrace).result,   ...
                datasetResults(idxCorrectTrace).result);
            
            f = figure;
            if isTrain
                geoplot(datasetResults(iTrace).ref.posLla(:, 1), datasetResults(iTrace).ref.posLla(:, 2), '.-k');
            end
            geobasemap(basemap);
            hold on;
            % Plot old result
            geoplot(datasetResults(iTrace).result.estPosLla(:, 1), datasetResults(iTrace).result.estPosLla(:, 2), '.b');
            % Plot new reassigned result
            geoplot(newResult.estPosLla(:, 1), newResult.estPosLla(:, 2), '.r');
            geobasemap(basemap);
            if isTrain
                legend('Groundtruth', 'RTK old', 'RTK new');
            else
                legend('RTK old', 'RTK new');
            end
            title = strcat(num2str(iTrace), '_', datasetResults(iTrace).campaignName, '_', datasetResults(iTrace).phoneName);
            figureWindowTitle(f, title);
            pause;
            close(f);
            
            datasetResults(iTrace).result = newResult;
        else
            error('Campaign ''%s'' has no correct trace', thisCampaign);
        end
    end
    if isTrain
        score(iTrace) = computeScore(datasetResults(iTrace).ref, datasetResults(iTrace).result);
        fprintf('-> %s_%s - Score: %.4f \n\n', ...
            datasetResults(iTrace).campaignName, datasetResults(iTrace).phoneName, score(iTrace));
        fprintf(fidScore, '%s,%s,%.4f\n', ...
            datasetResults(iTrace).campaignName, datasetResults(iTrace).phoneName, score(iTrace));
    end
    resultsFilePath = saveResults(datasetResults(iTrace).result, fileNamePreamble);
end
if isTrain
    finalScore = mean(score);
    fprintf('\n ==== FINAL SCORE: %.4f ====\n', finalScore);
    fprintf(fidScore, '\nfinal score, %.4f\n', finalScore);
    fclose(fidScore);
end

fprintf('\n File with reassigned results generated: \n\t %s \n', resultsFilePath);

end


function resultNew = reassignResult(resultOld, resultRef)
% REASSIGNRESULT assigns the resulting trace of resultRef to resultOld
% maintaining the duration of resultOld

% Find indices in resultNew closest in time to trace of resultOld
firstIdx = find(resultRef.utcSeconds <= resultOld.utcSeconds(1), 1, 'last');
if isempty(firstIdx)
    firstIdx = find(resultRef.utcSeconds >= resultOld.utcSeconds(1), 1, 'first');
end
lastIdx = find(resultRef.utcSeconds >= resultOld.utcSeconds(end), 1, 'first');
if isempty(lastIdx)
    lastIdx = find(resultRef.utcSeconds <= resultOld.utcSeconds(end), 1, 'last');
end

% Reassign all fields
fnames = fieldnames(resultOld);
traceLength = length(resultRef.utcSeconds);
for i = 1:length(fnames)
    % Find dimension of field
    dim = find(size(resultRef.(fnames{i})) == traceLength);
    
    switch dim
        case 1
            resultNew.(fnames{i}) = resultRef.(fnames{i})(firstIdx:lastIdx, :);
        case 2
            resultNew.(fnames{i}) = resultRef.(fnames{i})(:, firstIdx:lastIdx);
        otherwise
            error('Invalid dimension');
    end
    
end
end

