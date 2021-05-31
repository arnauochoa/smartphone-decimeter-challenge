function [err, ref, estPosLla, result] = evaluateDataset()
    %% Input
    [phoneRnx, imuRaw, nav, ~, osrRnx, ref] = loadData();

    %% Compute geometry

    %% Pre-process IMU measurements
%     imuClean = preprocessImu(imuRaw);
    imuClean = [];
    
    %% Interpolate OSR data
    osrRnx = interpOSR(osrRnx, phoneRnx);
    if ~isempty(osrRnx.obs)
        %% Navigate
        disp('Computing positions...');
        result = navigate(phoneRnx, imuClean, nav, osrRnx, ref);

        %% Output
        disp('Navigation ended, saving results...');
        estPosLla = saveResults(result.xEst, result.utcSeconds);
        err = Constants.NO_ERR;
    else
        err = Constants.ERR_NO_OSR;
        ref = []; estPosLla = []; result = [];
        warning('The campaing ''%s'' does not have OSR data available', config.campaignName);
    end
end