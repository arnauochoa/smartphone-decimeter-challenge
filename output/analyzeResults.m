function analyzeResults(matFilePath)
close all;
if nargin < 1
    matFilePath = 'data/results/train/all/result_20210622_111534.mat';
end
load(matFilePath, 'datasetResults');

%% Initializations
idxStatePos = PVTUtils.getStateIndex(PVTUtils.ID_POS);
isTrain = ~isempty(datasetResults(1).ref);
nTraces = length(datasetResults);
figures = [];
basemap = 'none';

idxCampaignsToPlot = [nTraces-10:nTraces]; % [1:5] [nDatasets-5:nDatasets]

%% Map plots
for iSet = idxCampaignsToPlot
    figures = [figures figure];
    if isTrain
        geoplot(datasetResults(iSet).ref.posLla(:, 1), datasetResults(iSet).ref.posLla(:, 2), '.-k', ...
            datasetResults(iSet).result.estPosWLSLla(:, 1), datasetResults(iSet).result.estPosWLSLla(:, 2), '.b');
    else
        geoplot(datasetResults(iSet).result.estPosWLSLla(:, 1), datasetResults(iSet).result.estPosWLSLla(:, 2), '.b');
    end
    geobasemap(basemap);
    % Plot estimation with color depending on covariance
    stdHor = vecnorm(datasetResults(iSet).result.posStdNed(:, 1:2), 2, 2);
    hold on; colormap summer;
    geoscatter(datasetResults(iSet).result.estPosLla(:, 1), datasetResults(iSet).result.estPosLla(:, 2), 5, stdHor, 'fill'); 
    c = colorbar; 
    c.Label.String = 'Horizontal position STD (m)';
    if isTrain, legend('Groundtruth', 'WLS', 'RTK');
    else, legend('WLS', 'RTK'); end
    title = strcat(num2str(iSet), '_', datasetResults(iSet).campaignName, '_', datasetResults(iSet).phoneName);
    figureWindowTitle(figures(end), title);
end

%% Score computation
if isTrain
    hErr95 = nan(nTraces, 1);
    hErr50 = nan(nTraces, 1);
    score = nan(nTraces, 1);
    for iSet = 1:nTraces
        % Interpolate groundtruth at the computed position's time
        refInterpLla = interp1(datasetResults(iSet).ref.utcSeconds, datasetResults(iSet).ref.posLla, datasetResults(iSet).result.utcSeconds);
        assert(size(refInterpLla, 1) == size(datasetResults(iSet).result.xRTK, 2), ...
            'Reference and computed position vectors are not the same size');
        % Position error
        hError = Lla2Hd(refInterpLla, datasetResults(iSet).result.estPosLla);

        % Compute score
        hErr95(iSet) = prctile(abs(hError),95);
        hErr50(iSet) = prctile(abs(hError),50);
        score(iSet) = mean([hErr95(iSet) hErr50(iSet)]);
        title = [datasetResults(iSet).campaignName '_' datasetResults(iSet).phoneName];
        fprintf(' - %s: %.4f \n', title, score(iSet));
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