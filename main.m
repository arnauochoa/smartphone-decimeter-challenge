clearvars -except
close all; 
clc;
% Script description

% Change the configuration in Config class

%% Input
[gnssRnx, imuRaw, nav, iono, ref] = loadData();

%% Compute geometry

%% Pre-process IMU measurements
imuClean = preprocessImu(imuRaw);

%% Navigate
disp('Computing positions...');
[xEst, prInnovations, prInnovationCovariances, dopInnovations, dopInnovationCovariances, ...
    utcSecondsHist, sigmaHist, prRejectedHist, dopRejectedHist] = ...
    navigate(gnssRnx, imuClean, nav, iono, ref);

%% Output
disp('Navigation ended, plotting results...');
plotResults(ref, xEst, prInnovations, prInnovationCovariances, dopInnovations, ...
    dopInnovationCovariances, utcSecondsHist, sigmaHist, prRejectedHist, dopRejectedHist);

