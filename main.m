clearvars -except
close all; 
clc;
% Script description

% Change the configuration in Config class

%% Input
[phoneRnx, imuRaw, nav, ~, osrRnx, ref] = loadData();

%% Compute geometry

%% Pre-process IMU measurements
imuClean = preprocessImu(imuRaw);

%% Interpolate OSR data
osrRnx = interpOSR(osrRnx, phoneRnx);

%% Navigate
disp('Computing positions...');
[xEst, sigmaHist, prInnovations, prInnovationCovariances, dopInnovations, dopInnovationCovariances, ...
    utcSecondsHist, prRejectedHist, dopRejectedHist] = ...
    navigate(phoneRnx, imuClean, nav, osrRnx, ref);

%% Output
disp('Navigation ended, plotting results...');
plotResults(ref, xEst, sigmaHist, prInnovations, prInnovationCovariances, dopInnovations, ...
    dopInnovationCovariances, utcSecondsHist, prRejectedHist, dopRejectedHist);
