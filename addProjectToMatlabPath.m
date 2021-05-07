function addProjectToMatlabPath()
%ADDPROJECTTOMATLABPATH Adds smartphone-decimeter-challenge project to the Matlab path.
pathOfThisScript = mfilename('fullpath');
[pathstr, ~, ~] = fileparts(pathOfThisScript);
% Add folders in 'smartphone-decimeter-challenge' directory
addpath(genpath([pathstr filesep 'configuration']));
addpath(genpath([pathstr filesep 'gnss']));
addpath(genpath([pathstr filesep 'imu']));
addpath(genpath([pathstr filesep 'input']));
addpath(genpath([pathstr filesep 'lib']));
addpath(genpath([pathstr filesep 'navigation']));
addpath(genpath([pathstr filesep 'output']));
addpath(genpath([pathstr filesep 'utils']));
end

