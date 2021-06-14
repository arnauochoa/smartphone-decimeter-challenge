clearvars -except
close all;
clc;
% Script description

% Change the configuration in Config class
config = Config.getInstance;
delete(config); % Delete previous instance of Config
config = Config.getInstance;

datasetResults = [];
score = [];
iDataset = 0;

tic
switch config.EVALUATE_DATASETS
    case 'single'
        [ref, result, ~] = evaluateDataset();
        disp('Plotting results...')
        plotResults(ref, result);
    case 'all'
        campaignNames = getValidDir(config.obsDataPath);
        if strcmp(config.DATASET_TYPE, 'train')
            resultsDir = getResultsDir(config);
            fidScore = fopen([resultsDir 'score_' config.resFileTimestamp 'csv'], 'w');
            fprintf(fidScore, 'phone,score\n');
        end
        for iCampaign = 1:length(campaignNames)
            config.campaignName = campaignNames{iCampaign};
            campaignPath = [config.obsDataPath campaignNames{iCampaign} filesep];
            phoneNames = getValidDir(campaignPath);
            for iPhone = 1:length(phoneNames)
                config.phoneName = phoneNames{iPhone};
                fprintf('Evaluating %s/%s \n', config.campaignName, config.phoneName)
                iDataset = iDataset + 1;
                [datasetResults(iDataset).ref, datasetResults(iDataset).result, resultsFilePath] ...
                    = evaluateDataset();
                
                datasetResults(iDataset).campaignName = campaignNames{iCampaign};
                datasetResults(iDataset).phoneName = phoneNames{iPhone};
                % RTK and WLS estimations to Geodetic
                idxStatePos = PVTUtils.getStateIndex(PVTUtils.ID_POS);
                datasetResults(iDataset).result.estPosLla = ...
                    ecef2geodeticVector(datasetResults(iDataset).result.xRTK(idxStatePos, :)');
                datasetResults(iDataset).result.estPosWLSLla = ...
                    ecef2geodeticVector(datasetResults(iDataset).result.xWLS(1:3, :)');
                
                if strcmp(config.DATASET_TYPE, 'train')
                    score(iDataset) = computeScore(datasetResults(iDataset).ref, datasetResults(iDataset).result);
                    fprintf(2, '\n -> %s_%s - Score: %.4f \n\n', campaignNames{iCampaign}, phoneNames{iPhone}, score(iDataset));
                    fprintf(fidScore, '%s_%s, %.4f\n', campaignNames{iCampaign}, phoneNames{iPhone}, score(iDataset));
                end
                fprintf('\n==================================================================================================================\n');
            end
        end
        % Compare output file with sample submission
        if strcmp(config.DATASET_TYPE, 'test')
            checkOutputFile(resultsFilePath);
        elseif strcmp(config.DATASET_TYPE, 'train')
            finalScore = mean(score);
            fprintf('\n ==== FINAL SCORE: %.4f ====\n', finalScore);
            fprintf(fidScore, '\nfinal score\n');
            fprintf(fidScore, '%.4f\n', finalScore);
            fclose(fidScore);
        end
        % Save all results in a mat file
        resMatPath = [workspacePath 'data' filesep 'results' filesep config.DATASET_TYPE filesep 'all' filesep];
        resMatName = [config.RES_FILENAME '_' config.resFileTimestamp '.mat'];
        save([resMatPath resMatName], 'datasetResults');
    otherwise
        error('Invalid field for Config.EVALUATE_DATASETS, choose among ''single'' and ''all''');
end
toc