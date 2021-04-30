clearvars -except
close all; clc;
% Script description

% Change the configuration in Config class

%% Input
[gnss, imu, nav, ref] = loadData();

%% Compute geometry


%% Pre-process IMU measurements
imu = preprocessImu(imu);


%% Navigate


%% Output

