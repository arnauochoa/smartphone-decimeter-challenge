function [ref, estPosLla, result] = evaluateDataset()
    %% Input
    [phoneRnx, imuRaw, nav, ~, osrRnx, ref] = loadData();

    %% Compute geometry

    %% Pre-process IMU measurements
    imuClean = preprocessImu(imuRaw);

    %% Interpolate OSR data
    osrRnx = interpOSR(osrRnx, phoneRnx);

    %% Navigate
    disp('Computing positions...');
    result = navigate(phoneRnx, imuClean, nav, osrRnx, ref);

    %% Output
    disp('Navigation ended, saving results...');
    estPosLla = saveResults(result.xEst, result.utcSeconds);
end