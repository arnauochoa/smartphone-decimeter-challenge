clearvars -except
close all;
clc;
% Script description

% Configuration can be changed in Config class
config = Config.getInstance;
delete(config); % Delete previous instance of Config
config = Config.getInstance;

datasetResults = [];
score = [];
iTrace = 0;

tic
switch config.EVALUATE_DATASETS
    case 'single'
        [ref, result, ~] = evaluateTrace();
        disp('Plotting results...')
        plotResults(ref, result);
    case 'all'
        campaignNames = getValidDir(config.obsDataPath);
        if strcmp(config.DATASET_TYPE, 'train')
            resultsDir = getResultsDir(config);
            fidScore = fopen([resultsDir 'score_' config.resFileTimestamp '.csv'], 'w');
            fprintf(fidScore, 'campaign,phone,score\n');
        end
        for iCampaign = 1:length(campaignNames)
            config.campaignName = campaignNames{iCampaign};
            campaignPath = [config.obsDataPath campaignNames{iCampaign} filesep];
            phoneNames = getValidDir(campaignPath);
%             datasetResults = [];
            for iPhone = 1:length(phoneNames)
                config.phoneName = phoneNames{iPhone};
                fprintf('Evaluating %s/%s \n', config.campaignName, config.phoneName)
                iTrace = iTrace + 1;
                [datasetResults(iTrace).ref, datasetResults(iTrace).result, resultsFilePath] ...
                    = evaluateTrace();
                
                datasetResults(iTrace).campaignName = campaignNames{iCampaign};
                datasetResults(iTrace).phoneName = phoneNames{iPhone};
                % RTK and WLS estimations to Geodetic
                idxStatePos = PVTUtils.getStateIndex(PVTUtils.ID_POS);
                datasetResults(iTrace).result.estPosLla = ...
                    ecef2geodeticVector(datasetResults(iTrace).result.xRTK(idxStatePos, :)');
                datasetResults(iTrace).result.estPosWLSLla = ...
                    ecef2geodeticVector(datasetResults(iTrace).result.xWLS(1:3, :)');
                
                if strcmp(config.DATASET_TYPE, 'train')
                    score(iTrace) = computeScore(datasetResults(iTrace).ref, datasetResults(iTrace).result);
                    fprintf(2, '\n -> %s_%s - Score: %.4f \n\n', campaignNames{iCampaign}, phoneNames{iPhone}, score(iTrace));
                    fprintf(fidScore, '%s,%s,%.4f\n', campaignNames{iCampaign}, phoneNames{iPhone}, score(iTrace));
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
            fprintf(fidScore, '\nfinal score, %.4f\n', finalScore);
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