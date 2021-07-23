function [phones] = preprocessIns(phones)
% PREPROCESSINS Aligns the INS sensors' timestamps and removes the bias in
% the measurements
%   [phones] = PREPROCESSINS(phones)
%
% Input:
%   phones.ins  = structure containing the INS measurements, including
%            	.acc
%              	.gyr
%             	.mag
% Input:
%   phones.ins	= structure containing the imu measurements after time
%               alignement and bias correction
%
config = Config.getInstance;
nPhones = length(config.phoneNames);

for iPhone = 1:nPhones
    if isempty(phones(iPhone).ins.acc)
        insClean = [];
    else
        [dirPath, fileName] = getObsDirFile(config);
        inputSignature = DataHash([dirPath 'insClean_' fileName{iPhone}], 'array');
        if Fcache.hasValidCacheFile(mfilename, inputSignature)
            disp('Loading clean INS measurements...');
            cache = Fcache.get(mfilename, inputSignature);
            insClean = cache.insClean;
        else
            disp([mfilename '>> No cached file found, preprocessing IMU measurements']);
            insClean = preprocessing(phones(iPhone).ins);
            % Save clean INS in cache
            cache = struct();
            cache.insClean = insClean;
            Fcache.cache(mfilename, inputSignature, cache);
        end
    end
    phones(iPhone).ins = insClean;
    
    % figure; plot(imuClean.utcSeconds, imuClean.accBodyMps2, '.')
    % figure; plot(imuClean.utcSeconds, imuClean.gyrBodyRadPerSec, '.')
    % figure; plot(imuClean.utcSeconds, imuClean.magBodyMicroT, '.')
    % figure; plot(phones(iPhone).ins.acc.utcTimeMillis, [phones(iPhone).ins.acc.UncalAccelXMps2 phones(iPhone).ins.acc.UncalAccelYMps2 phones(iPhone).ins.acc.UncalAccelZMps2], '.')
    % figure; plot(phones(iPhone).ins.gyr.utcTimeMillis, [phones(iPhone).ins.gyr.UncalGyroXRadPerSec phones(iPhone).ins.gyr.UncalGyroYRadPerSec phones(iPhone).ins.gyr.UncalGyroZRadPerSec], '.')
    % figure; plot(phones(iPhone).ins.mag.utcTimeMillis, [phones(iPhone).ins.mag.UncalMagXMicroT phones(iPhone).ins.mag.UncalMagYMicroT phones(iPhone).ins.mag.UncalMagZMicroT], '.')
end

end

function insClean = preprocessing(insRaw)
config = Config.getInstance;
%% Correct bias
if isfield(insRaw.acc, 'BiasXMps2')
    accBodyMps2(:, 1) = insRaw.acc.UncalAccelXMps2 - insRaw.acc.BiasXMps2;
    accBodyMps2(:, 2) = insRaw.acc.UncalAccelYMps2 - insRaw.acc.BiasYMps2;
    accBodyMps2(:, 3) = insRaw.acc.UncalAccelZMps2 - insRaw.acc.BiasZMps2;
else
    accBodyMps2(:, 1) = insRaw.acc.UncalAccelXMps2; % TODO: find bias if not provided
    accBodyMps2(:, 2) = insRaw.acc.UncalAccelYMps2;
    accBodyMps2(:, 3) = insRaw.acc.UncalAccelZMps2;
end

if isfield(insRaw.gyr, 'DriftXRadPerSec')
    gyrBodyRadPerSec(:, 1) = insRaw.gyr.UncalGyroXRadPerSec - insRaw.gyr.DriftXRadPerSec;
    gyrBodyRadPerSec(:, 2) = insRaw.gyr.UncalGyroYRadPerSec - insRaw.gyr.DriftYRadPerSec;
    gyrBodyRadPerSec(:, 3) = insRaw.gyr.UncalGyroZRadPerSec - insRaw.gyr.DriftZRadPerSec;
else
    gyrBodyRadPerSec(:, 1) = insRaw.gyr.UncalGyroXRadPerSec;
    gyrBodyRadPerSec(:, 2) = insRaw.gyr.UncalGyroYRadPerSec;
    gyrBodyRadPerSec(:, 3) = insRaw.gyr.UncalGyroZRadPerSec;
end

if isfield(insRaw.mag, 'BiasXMicroT')
    magBodyMicroT(:, 1) = insRaw.mag.UncalMagXMicroT - insRaw.mag.BiasXMicroT;
    magBodyMicroT(:, 2) = insRaw.mag.UncalMagYMicroT - insRaw.mag.BiasYMicroT;
    magBodyMicroT(:, 3) = insRaw.mag.UncalMagZMicroT - insRaw.mag.BiasZMicroT;
else
    magBodyMicroT(:, 1) = insRaw.mag.UncalMagXMicroT;
    magBodyMicroT(:, 2) = insRaw.mag.UncalMagYMicroT;
    magBodyMicroT(:, 3) = insRaw.mag.UncalMagZMicroT;
end


%% Set IMU time as the slowest sensor's so we always have all measurements
imuSensorNames = {'acc', 'gyr', 'mag'};
nCounts = zeros(size(imuSensorNames));
for iSensor = 1:length(imuSensorNames)
    nCounts(iSensor) = length(insRaw.(imuSensorNames{iSensor}).utcTimeMillis);
end
[~, indMin] = min(nCounts);
insClean.utcSeconds = insRaw.(imuSensorNames{indMin}).utcTimeMillis / 1e3;

%% Interpolate measurements at imuTime
[insRaw.acc.utcTimeMillis, accBodyMps2] = ...
    cleanRepeatedSamplePts(insRaw.acc.utcTimeMillis, accBodyMps2);
for iCoord = 1:3
    insClean.accBodyMps2(:, iCoord) = interp1gap(   ...
        insRaw.acc.utcTimeMillis / 1e3,             ...
        accBodyMps2(:, iCoord),                     ...
        insClean.utcSeconds,                        ...
        config.MAX_IMU_INTERP_GAP_SEC,              ...
        'spline',                                   ...
        'extrap',nan);
end

[insRaw.gyr.utcTimeMillis, gyrBodyRadPerSec] = ...
    cleanRepeatedSamplePts(insRaw.gyr.utcTimeMillis, gyrBodyRadPerSec);
for iCoord = 1:3
    insClean.gyrBodyRadPerSec(:, iCoord) = interp1gap(  ...
        insRaw.gyr.utcTimeMillis / 1e3,                 ...
        gyrBodyRadPerSec(:, iCoord),                    ...
        insClean.utcSeconds,                            ...
        config.MAX_IMU_INTERP_GAP_SEC,                  ...
        'spline',                                       ...
        'extrap',nan);
end

[insRaw.mag.utcTimeMillis, magBodyMicroT] = ...
    cleanRepeatedSamplePts(insRaw.mag.utcTimeMillis, magBodyMicroT);
for iCoord = 1:3
    insClean.magBodyMicroT(:, iCoord) = interp1gap( ...
        insRaw.mag.utcTimeMillis / 1e3,             ...
        magBodyMicroT(:, iCoord),                   ...
        insClean.utcSeconds,                        ...
        config.MAX_IMU_INTERP_GAP_SEC,              ...
        'spline',                                   ...
        'extrap',nan);
end
end
