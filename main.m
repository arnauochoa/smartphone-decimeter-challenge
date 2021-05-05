clearvars -except
close all; clc;
% Script description

% Change the configuration in Config class

%% Input
[gnssRnx, imuRaw, nav, iono, ref] = loadData();

%% Compute geometry


%% Pre-process IMU measurements
imuClean = preprocessImu(imuRaw);

%% Navigate
[xEst] = navigate(gnssRnx, imuClean, nav, iono);

%% Output
disp('Navigation ended, plotting results...');
plotResults(xEst, ref);

