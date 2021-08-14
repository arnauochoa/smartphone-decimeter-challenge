clearvars -except
% close all;
clc;
% This script launches evaluation of dataset(s) following the selected
% configuration and calls the functions that plot and save the results.

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
        [ref, result] = evaluateTrace();
        disp('Navigation ended, saving results...');
        resultsFilePath = saveResults(result, config.PHONE_NAME);
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
%             campaignPath = [config.obsDataPath campaignNames{iCampaign} filesep];
            phoneNames = getPhoneNamesInCampaign(config);
            config.phoneNames = phoneNames;
%             datasetResults = [];
            evaluate = true;
            isSinglePhone = ~config.MULTI_RX;
            if strcmp(config.DATASET_TYPE, 'test')
                campaign = getGeometryForTest(config.campaignName);
                isSinglePhone = isempty(campaign);
            end
            for iPhone = 1:length(phoneNames)
                if isSinglePhone
                    config.phoneNames = phoneNames(iPhone);
                end
                fprintf('Evaluating %s/%s \n', config.campaignName, strjoin(config.phoneNames, '+'))
                iTrace = iTrace + 1;
                if evaluate
                    [datasetResults(iTrace).ref, datasetResults(iTrace).result] ...
                        = evaluateTrace();
                    if ~isSinglePhone
                        evaluate = false; 
                    end
                else
                    [phones, ~, ~, ~] = loadData();
                    if strcmp(config.DATASET_TYPE, 'train')
                        datasetResults(iTrace).ref = phones(iPhone).ref;
                    end
                    datasetResults(iTrace).result = datasetResults(1).result;
                end
                disp('Navigation ended, saving results...');
                resultsFilePath = saveResults(datasetResults(iTrace).result, phoneNames{iPhone});
                
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