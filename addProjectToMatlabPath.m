function addProjectToMatlabPath()
%ADDPROJECTTOMATLABPATH Adds smartphone-decimeter-challenge project to the Matlab path.
pathOfThisScript = mfilename('fullpath');
[pathstr, ~, ~] = fileparts(pathOfThisScript);
% Add folders in 'smartphone-decimeter-challenge' directory
addpath([pathstr filesep 'android-measurements']);
addpath([pathstr filesep 'EKF']);
addEKFToMatlabPath;
addpath([pathstr filesep 'geometry']);
addpath([pathstr filesep 'gps-measurement-tools' filesep 'NmeaUtils']);
addpath([pathstr filesep 'imu']);
addpath([pathstr filesep 'input']);
addpath([pathstr filesep 'INS']);
addINSToMatlabPath;
addpath(genpath([pathstr filesep 'lib']));
addpath([pathstr filesep 'magnitude']);
addMagnitudeToMatlabPath;
addpath([pathstr filesep 'navigation']);
addpath([pathstr filesep 'output']);
addpath([pathstr filesep 'utils']);
end

