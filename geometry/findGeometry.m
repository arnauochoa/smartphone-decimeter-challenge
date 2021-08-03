function [phones, isKnownGeometry]= findGeometry(phones)
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
isKnownGeometry   = true;

switch config.DATASET_TYPE
    case 'train'
        campaign        = config.campaignName(1:end);
    case 'test'
        campaign        = config.campaignName(1:end-2);
end

try
    load([folderpath campaign], 'phoneNames', 'phonePosBody');
catch ME
    if (strcmp(ME.identifier,'MATLAB:load:couldNotReadFile'))
        warning('No geometry file exists for %s. Using single-rx', campaign);
        isKnownGeometry = false;
    end
end

if isKnownGeometry
    for iPhone = 1:nPhones
        phoneIdx = find(ismember(phoneNames, config.phoneNames(iPhone)));
        if ~isempty(phoneIdx)
            phones(iPhone).posBody = phonePosBody(:, phoneIdx);
        else
            isKnownGeometry = false;
        end
    end
end
if ~isKnownGeometry
    config.phoneNames = config.phoneNames(1);
    phones = phones(1);
    phones.posBody = [0 0 0]';
end
end