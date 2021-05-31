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

config = Config.getInstance;
if isempty(imuRaw.acc)
    imuClean = [];
else
    [dirPath, fileName] = getObsDirFile(config);
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
    if isfield(imuRaw.acc, 'BiasXMps2')
        accBodyMps2(:, 1) = imuRaw.acc.UncalAccelXMps2 - imuRaw.acc.BiasXMps2;
        accBodyMps2(:, 2) = imuRaw.acc.UncalAccelYMps2 - imuRaw.acc.BiasYMps2;
        accBodyMps2(:, 3) = imuRaw.acc.UncalAccelZMps2 - imuRaw.acc.BiasZMps2;
    else
        accBodyMps2(:, 1) = imuRaw.acc.UncalAccelXMps2; % TODO: find bias if not provided
        accBodyMps2(:, 2) = imuRaw.acc.UncalAccelYMps2;
        accBodyMps2(:, 3) = imuRaw.acc.UncalAccelZMps2;
    end
    
    if isfield(imuRaw.gyr, 'DriftXRadPerSec')
        gyrBodyRadPerSec(:, 1) = imuRaw.gyr.UncalGyroXRadPerSec - imuRaw.gyr.DriftXRadPerSec;
        gyrBodyRadPerSec(:, 2) = imuRaw.gyr.UncalGyroYRadPerSec - imuRaw.gyr.DriftYRadPerSec;
        gyrBodyRadPerSec(:, 3) = imuRaw.gyr.UncalGyroZRadPerSec - imuRaw.gyr.DriftZRadPerSec;
    else
        gyrBodyRadPerSec(:, 1) = imuRaw.gyr.UncalGyroXRadPerSec;
        gyrBodyRadPerSec(:, 2) = imuRaw.gyr.UncalGyroYRadPerSec;
        gyrBodyRadPerSec(:, 3) = imuRaw.gyr.UncalGyroZRadPerSec;
    end
    
    if isfield(imuRaw.mag, 'BiasXMicroT')
        magBodyMicroT(:, 1) = imuRaw.mag.UncalMagXMicroT - imuRaw.mag.BiasXMicroT;
        magBodyMicroT(:, 2) = imuRaw.mag.UncalMagYMicroT - imuRaw.mag.BiasYMicroT;
        magBodyMicroT(:, 3) = imuRaw.mag.UncalMagZMicroT - imuRaw.mag.BiasZMicroT;
    else
        magBodyMicroT(:, 1) = imuRaw.mag.UncalMagXMicroT;
        magBodyMicroT(:, 2) = imuRaw.mag.UncalMagYMicroT;
        magBodyMicroT(:, 3) = imuRaw.mag.UncalMagZMicroT;
    end
    
    
    % Set IMU time as the slowest sensor's so we always have all measurements
    imuSensorNames = {'acc', 'gyr', 'mag'};
    nCounts = zeros(size(imuSensorNames));
    for iSensor = 1:length(imuSensorNames)
        nCounts(iSensor) = length(imuRaw.(imuSensorNames{iSensor}).utcTimeMillis);
    end
    [~, indMin] = min(nCounts);
    imuClean.utcSeconds = imuRaw.(imuSensorNames{indMin}).utcTimeMillis / 1e3;
    
    % Interpolate measurements at imuTime
    for iCoord = 1:3
        imuClean.accBodyMps2(:, iCoord) = interp1gap(   ...
            imuRaw.acc.utcTimeMillis / 1e3,             ...
            accBodyMps2(:, iCoord),                     ...
            imuClean.utcSeconds,                        ...
            config.MAX_IMU_INTERP_GAP_SEC,              ...
            'spline',                                   ...
            'extrap',nan);
    end
    
    for iCoord = 1:3
        imuClean.gyrBodyRadPerSec(:, iCoord) = interp1gap(  ...
            imuRaw.gyr.utcTimeMillis / 1e3,                 ...
            gyrBodyRadPerSec(:, iCoord),                    ...
            imuClean.utcSeconds,                            ...
            config.MAX_IMU_INTERP_GAP_SEC,                  ...
            'spline',                                       ...
            'extrap',nan);
    end
    
    for iCoord = 1:3
        imuClean.magBodyMicroT(:, iCoord) = interp1gap( ...
            imuRaw.mag.utcTimeMillis / 1e3,             ...
            magBodyMicroT(:, iCoord),                   ...
            imuClean.utcSeconds,                        ...
            config.MAX_IMU_INTERP_GAP_SEC,              ...
            'spline',                                   ...
            'extrap',nan);
    end
    
    cache = struct();
    cache.imuClean = imuClean;
    Fcache.cache(mfilename, inputSignature, cache);
end

% figure; plot(imuClean.utcSeconds, imuClean.accBodyMps2, '.')
% figure; plot(imuClean.utcSeconds, imuClean.gyrBodyRadPerSec, '.')
% figure; plot(imuClean.utcSeconds, imuClean.magBodyMicroT, '.')
% figure; plot(imuRaw.acc.utcTimeMillis, [imuRaw.acc.UncalAccelXMps2 imuRaw.acc.UncalAccelYMps2 imuRaw.acc.UncalAccelZMps2], '.')
% figure; plot(imuRaw.gyr.utcTimeMillis, [imuRaw.gyr.UncalGyroXRadPerSec imuRaw.gyr.UncalGyroYRadPerSec imuRaw.gyr.UncalGyroZRadPerSec], '.')
% figure; plot(imuRaw.mag.utcTimeMillis, [imuRaw.mag.UncalMagXMicroT imuRaw.mag.UncalMagYMicroT imuRaw.mag.UncalMagZMicroT], '.')

end

