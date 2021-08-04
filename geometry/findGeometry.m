function [phones, isSinglePhone]= findGeometry(phones)
% FINDGEOMETRY Computes the geometry of the smartphones in the lever arm.
% Phone 1 is considered the master receiver
%   phones = FINDGEOMETRY(phones)
%
% Input:
%   phones  =   Structure array. Contains GNSS and INS measurements and
%               groundtruth
% Output:
%   phones  =   Structure array. Contains GNSS and INS measurements,
%               groundtruth and geometry in body frame

%% Initializations
config          = Config.getInstance();
nPhones         = length(phones);
folderpath      = [projectPath filesep 'geometry' filesep 'baselines' filesep];
isSinglePhone   = false;

switch config.DATASET_TYPE
    case 'train'
        campaign        = config.campaignName;
    case 'test'
        campaign = getGeometryForTest(config.campaignName);
        isSinglePhone = isempty(campaign);
end

try
    load([folderpath campaign], 'phoneNames', 'phonePosBody');
catch ME
    if (strcmp(ME.identifier,'MATLAB:load:couldNotReadFile'))
        warning('No geometry file exists for %s. Using single-rx', campaign);
        isSinglePhone = true;
    end
end

if isSinglePhone
    phones = phones(1);
    phones.posBody = [0 0 0]';
else
    for iPhone = 1:nPhones
        phoneIdx = find(ismember(phoneNames, config.phoneNames(iPhone)));
        if ~isempty(phoneIdx)
            phones(iPhone).posBody = phonePosBody(:, phoneIdx);
        else
            phones(iPhone).posBody = [0 0 0]';
        end
    end
end
end