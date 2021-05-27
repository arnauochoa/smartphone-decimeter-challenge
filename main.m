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
[xEst, sigmaHist, prInnovations, prInnovationCovariances, ...
    utcSecondsHist, prRejectedHist] = ...
    navigate(phoneRnx, imuClean, nav, osrRnx, ref);

%% Output
disp('Navigation ended, saving results...');
estPosLla = saveResults(xEst, utcSecondsHist);
disp('Plotting results...')
plotResults(ref, estPosLla, xEst, sigmaHist, prInnovations, prInnovationCovariances, utcSecondsHist, prRejectedHist);






