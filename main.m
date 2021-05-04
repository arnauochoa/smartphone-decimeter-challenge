clearvars -except
close all; clc;
% Script description

% Change the configuration in Config class

%% Input
[gnssRnx, imuRaw, nav, ref] = loadData();

%% Compute geometry


%% Pre-process IMU measurements
imuClean = preprocessImu(imuRaw);

%% Navigate


%% Output

