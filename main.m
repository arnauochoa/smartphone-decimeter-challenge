clearvars -except
close all;
clc;
% Script description

% Change the configuration in Config class
config = Config.getInstance;
delete(config); % Delete previous instance of Config
config = Config.getInstance;

switch config.EVALUATE_DATASETS
    case 'single'
        [err, ref, estPosLla, result] = evaluateDataset();
        disp('Plotting results...')
        plotResults(ref, estPosLla, result);
    case 'all'
        datasetsPath = [config.dataPath 'datasets' filesep];
        campaignNames = getValidDir(datasetsPath);
        idxInvalid = strcmp(campaignNames, '.') | strcmp(campaignNames, '..');
        campaignNames(idxInvalid) = [];
        for iCampaign = 1:length(campaignNames)
            config.campaignName = campaignNames{iCampaign};
            campaignPath = [datasetsPath campaignNames{iCampaign} filesep];
            phoneNames = getValidDir(campaignPath);
            for iPhone = 1:length(phoneNames)
                config.phoneName = phoneNames{iPhone};
                fprintf('Evaluating %s/%s \n', config.campaignName, config.phoneName)
                err = evaluateDataset();
            end
        end
    otherwise
        error('Invalid field for Config.EVALUATE_DATASETS, choose among ''single'' and ''all''');
end
