config = Config.getInstance;

campaignNames = getValidDir(config.obsDataPath);
for iCampaign = 1:length(campaignNames)
    config.campaignName = campaignNames{iCampaign};
    campaignPath = [config.obsDataPath campaignNames{iCampaign} filesep];
    phoneNames = getValidDir(campaignPath);
    for iPhone = 1:length(phoneNames)
        config.phoneNames = {phoneNames{iPhone}};
        getNavFilepaths(config);
    end
end