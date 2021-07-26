function [ref, result, resultsFilePath] = evaluateTrace()
% EVALUATETRACE Evaluates a single trace using the selected configuration

    config = Config.getInstance;
    
    %% Input
    [phones, nav, ~, osrRnx] = loadData();
    
    if ~isempty(osrRnx.obs)
        %% Compute geometry
        phones = findGeometry(phones);

        %% Pre-process IMU measurements
        phones = preprocessIns(phones);
    
        %% Interpolate OSR data
        phones = interpOSR(osrRnx, phones);
        
        %% Navigate
        disp('Computing positions...');
        result = navigate(phones, nav);

        %% Output
        disp('Navigation ended, saving results...');
        resultsFilePath = saveResults(result);
        ref = phones(1).ref;
    else
        ref = []; result = [];
        warning('The campaing ''%s'' does not have OSR data available', config.campaignName);
    end
end