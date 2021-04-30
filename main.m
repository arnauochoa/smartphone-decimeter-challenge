clearvars -except
close all; clc;
% Script description

% Change the configuration in Config class

%% Input
[gnss, imuRaw, nav, ref] = loadData();

%% Compute geometry


%% Pre-process IMU measurements
imuClean = preprocessImu(imuRaw);

figure; plot(imuClean.utcTime, imuClean.accBodyMps2, '.')
figure; plot(imuClean.utcTime, imuClean.gyrBodyRadPerSec, '.')
figure; plot(imuClean.utcTime, imuClean.magBodyMicroT, '.')
figure; plot(imuRaw.acc.utcTimeMillis, [imuRaw.acc.UncalAccelXMps2 imuRaw.acc.UncalAccelYMps2 imuRaw.acc.UncalAccelZMps2], '.')
figure; plot(imuRaw.gyr.utcTimeMillis, [imuRaw.gyr.UncalGyroXRadPerSec imuRaw.gyr.UncalGyroYRadPerSec imuRaw.gyr.UncalGyroZRadPerSec], '.')
figure; plot(imuRaw.mag.utcTimeMillis, [imuRaw.mag.UncalMagXMicroT imuRaw.mag.UncalMagYMicroT imuRaw.mag.UncalMagZMicroT], '.')
%% Navigate


%% Output

