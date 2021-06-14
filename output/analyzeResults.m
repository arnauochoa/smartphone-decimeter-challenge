function analyzeResults(matFilePath)
close all;
if nargin < 1
    matFilePath = 'data/results/test/all/result_20210611_174823.mat';
end
load(matFilePath, 'datasetResults');

%% Initializations
isTrain = ~isempty(datasetResults(1).ref);
nDatasets = length(datasetResults);
figures = [];
basemap = 'none';

idxCampaignsToPlot = [nDatasets-5:nDatasets]; % [1:5] [nDatasets-5:nDatasets]

%% Map plots
for iSet = idxCampaignsToPlot    
    figures = [figures figure];
    if isTrain
        % Map plot
        geoplot(datasetResults(iSet).ref.posLla(:, 1), datasetResults(iSet).ref.posLla(:, 2), '.-', ...
            datasetResults(iSet).result.estPosWLSLla(:, 1), datasetResults(iSet).result.estPosWLSLla(:, 2), '.-',...
            datasetResults(iSet).result.estPosLla(:, 1), datasetResults(iSet).result.estPosLla(:, 2), '.-');
        legend('Groundtruth', 'WLS', 'RTK');
    else
        geoplot(datasetResults(iSet).result.estPosWLSLla(:, 1), datasetResults(iSet).result.estPosWLSLla(:, 2), '.-',...
            datasetResults(iSet).result.estPosLla(:, 1), datasetResults(iSet).result.estPosLla(:, 2), '.-');
        legend('WLS', 'RTK');
    end
    geobasemap(basemap);
    title = strcat(num2str(iSet), '_', datasetResults(iSet).campaignName, '_', datasetResults(iSet).phoneName);
    figureWindowTitle(figures(end), title);
end

%% Score computation
if isTrain
    hErr95 = nan(nDatasets, 1);
    hErr50 = nan(nDatasets, 1);
    score = nan(nDatasets, 1);
    for iSet = 1:nDatasets
        % Interpolate groundtruth at the computed position's time
        refInterpLla = interp1(datasetResults(iSet).ref.utcSeconds, datasetResults(iSet).ref.posLla, datasetResults(iSet).result.utcSeconds);
        assert(size(refInterpLla, 1) == size(datasetResults(iSet).result.xEst, 2), ...
            'Reference and computed position vectors are not the same size');
        % Position error
        hError = Lla2Hd(refInterpLla, datasetResults(iSet).result.estPosLla);

        % Compute score
        hErr95(iSet) = prctile(abs(hError),95);
        hErr50(iSet) = prctile(abs(hError),50);
        score(iSet) = mean([hErr95(iSet) hErr50(iSet)]);
        title = [datasetResults(iSet).campaignName '_' datasetResults(iSet).phoneName];
        fprintf(' - %c: %.4f \n', title, score(iSet));
    end
    fprintf('\n ==== FINAL SCORE: %.4f ====\n', mean(score));
end

%% Group plots
navi = [];
if ~isfield(navi, 'nav_report_group')
    try
        navi = groupPlots(figures, []);
    catch e
        warning(['Exception while grouping plots: ' e.message]);
    end
end
end