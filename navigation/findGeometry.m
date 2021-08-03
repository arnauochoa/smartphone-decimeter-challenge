function phones = findGeometry(phones)
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

campaign        = config.campaignName(1:end-2);
try
load([folderpath campaign], 'phoneNames', 'phonePosBody');
catch ME
    if (strcmp(ME.identifier,'MATLAB:load:couldNotReadFile'))
        error('No geometry file exists for %s', campaign);
    end
    rethrow(ME);
end

for iPhone = 1:nPhones
    phoneIdx = find(ismember(phoneNames, config.phoneNames(iPhone)));
    assert(~isempty(phoneIdx), 'Unknown geometry')
    phones(iPhone).posBody = phonePosBody(:, phoneIdx);
end
end