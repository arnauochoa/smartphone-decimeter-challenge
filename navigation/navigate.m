function [outputArg1,outputArg2] = navigate(inputArg1,inputArg2)
%NAVIGATE Summary of this function goes here
%   Detailed explanation goes here

%% Initializations
% KF variables


% Loop variables
lastTimestamp = 0;
indEstimation = 1;
hasPredicted = false;
while ~hasEnded
    imuMeas = getMeas(lastTimestamp);
    gnssObs = getObs(lastTimestamp);
    
    if imuMeas.t <= gnssObs.t
        % Perform prediction -> xMinus
%         estimation(indEstimation) = estimation(indEstimation - 1) + xMinus;
%         lastTimestamp = imuMeas.t;
%         indEstimation = indEstimation + 1;
%         hasPredicted = true;
    else
        % If last estimation is a prediction and current observation is
        % close enough in time, perform update
        if hasPredicted && gnssObs.t - imuMeas.t <= maxUpdateTime
            % Perform update over last prediction -> xPlus
%             estimation(indEstimation - 1) = estimation(indEstimation - 2) + xPlus;
        else % Otherwise, estimate without IMU
            % Estimation without IMU -> xPlus
%             estimation(indEstimation) = estimation(indEstimation - 1) + xPlus;
%             indEstimation = indEstimation + 1;
        end
%         lastTimestamp = gnssObs.t;
%         hasPredicted = false;
    end
    % Check if there are more measurements/observations
%     hasEnded = isempty(getMeas(indImu)) && isempty(getObs(indGnss));
end

end

