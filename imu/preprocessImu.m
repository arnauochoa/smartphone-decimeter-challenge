function [imuClean] = preprocessImu(imuRaw)
% PREPROCESSIMU Aligns the IMU sensors' timestamps and removes the bias in
% the measurements
%   [imu] = PREPROCESSIMU(imu)
%
% Input:
%   imuRaw  = structure containing the imu measurements, incluiding
%            	.acc
%              	.gyr
%             	.mag
% Input:
%   imuOut	= structure containing the imu measurements after time
%               alignement and bias correction
%

[dirPath, fileName] = Config.getObsDirFile();
inputSignature = DataHash([dirPath 'imuClean_' fileName], 'array');
if Fcache.hasValidCacheFile(mfilename, inputSignature)
    disp('Loading clean IMU measurements...');
    cache = Fcache.get(mfilename, inputSignature);
    imuClean = cache.imuClean;
    return;
else
    disp([mfilename '>> No cached file found, preprocessing IMU measurements']);
end


% Correct bias
accBodyMps2(:, 1) = imuRaw.acc.UncalAccelXMps2 - imuRaw.acc.BiasXMps2;
accBodyMps2(:, 2) = imuRaw.acc.UncalAccelYMps2 - imuRaw.acc.BiasYMps2;
accBodyMps2(:, 3) = imuRaw.acc.UncalAccelZMps2 - imuRaw.acc.BiasZMps2;

gyrBodyRadPerSec(:, 1) = imuRaw.gyr.UncalGyroXRadPerSec - imuRaw.gyr.DriftXRadPerSec;
gyrBodyRadPerSec(:, 2) = imuRaw.gyr.UncalGyroYRadPerSec - imuRaw.gyr.DriftYRadPerSec;
gyrBodyRadPerSec(:, 3) = imuRaw.gyr.UncalGyroZRadPerSec - imuRaw.gyr.DriftZRadPerSec;

magBodyMicroT(:, 1) = imuRaw.mag.UncalMagXMicroT - imuRaw.mag.BiasXMicroT;
magBodyMicroT(:, 2) = imuRaw.mag.UncalMagYMicroT - imuRaw.mag.BiasYMicroT;
magBodyMicroT(:, 3) = imuRaw.mag.UncalMagZMicroT - imuRaw.mag.BiasZMicroT;


% Set IMU time as the slowest sensor's so we always have all measurements
imuSensorNames = {'acc', 'gyr', 'mag'};
nCounts = zeros(size(imuSensorNames));
for iSensor = 1:length(imuSensorNames)
    nCounts(iSensor) = length(imuRaw.(imuSensorNames{iSensor}).utcTimeMillis);
end
[~, indMin] = min(nCounts);
imuClean.utcTime = imuRaw.(imuSensorNames{indMin}).utcTimeMillis;

% Interpolate measurements at imuTime
for iCoord = 1:3
    imuClean.accBodyMps2(:, iCoord) = interpMaxDist(  imuRaw.acc.utcTimeMillis,     ...
                                                    accBodyMps2(:, iCoord),         ...
                                                    imuClean.utcTime,               ...
                                                    Config.MAX_IMU_INTERP_MILLIS,   ...
                                                    'spline');
end

for iCoord = 1:3
    imuClean.gyrBodyRadPerSec(:, iCoord) = interpMaxDist(  imuRaw.gyr.utcTimeMillis,...
                                                    gyrBodyRadPerSec(:, iCoord),    ...
                                                    imuClean.utcTime,               ...
                                                    Config.MAX_IMU_INTERP_MILLIS,   ...
                                                    'spline');
end

for iCoord = 1:3
    imuClean.magBodyMicroT(:, iCoord) = interpMaxDist(  imuRaw.mag.utcTimeMillis,   ...
                                                    magBodyMicroT(:, iCoord),       ...
                                                    imuClean.utcTime,               ...
                                                    Config.MAX_IMU_INTERP_MILLIS,   ...
                                                    'spline');
end

cache = struct();
cache.imuClean = imuClean;
Fcache.cache(mfilename, inputSignature, cache);

end

