function [imu] = preprocessImu(imu)
% PREPROCESSIMU Aligns the IMU sensors' timestamps and removes the bias in
% the measurements
%   [imu] = PREPROCESSIMU(imu)
%
% Input:
%   imu             = structure containing the imu measurements, incluiding
%                       .acc
%                       .gyr
%                       .mag
% Input:
%   imu             = structure containing the imu measurements after time
%                       alignement and bias correction
%

imuSensorNames = {'acc', 'gyr', 'mag'};

% Correct bias

% Set IMU time as the slowest sensor's so we always have all measurements
nCounts = zeros(size(imuSensorNames));
for iSensor = 1:length(imuSensorNames)
    nCounts(iSensor) = length(imu.(imuSensorNames{iSensor}).utcTimeMillis);
end
[~, indMin] = min(nCounts);
imuTime = imu.(imuSensorNames{indMin}).utcTimeMillis;

% Interpolate measurements at imuTime

end

