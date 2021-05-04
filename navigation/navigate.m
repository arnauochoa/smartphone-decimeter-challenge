function [] = navigate(gnssRnx, imuRaw)
%NAVIGATE Summary of this function goes here
%   Detailed explanation goes here

%% Initializations
% KF variables
nStates = 8 + Config.getNumFreq-1 + Config.getNumConst-1;
% EKF:
esekf = EKF.build(uint16(nStates));
% Disable outlier rejection:
esekf.probabilityOfFalseOutlierRejection = 0;
% States:
% 3D pos, 3D vel, clock bias, clock drift, inter-freq. bias, inter-sys bias
esekf.x = zeros(nStates, 1);
esekf.P = Config.getP0();
esekf.tx = gnss.utcMillis(1); % TODO review this

% Loop variables
lastUtcMillis = 0;
indEstimation = 1;
hasPredicted = true;% TODO review this
while ~hasEnded
    %         imuMeas = getMeas(lastUtcMillis);
    gnss = getNextGnss(lastUtcMillis, gnssRnx);
    
    %     if imuMeas.utcMillis <= gnssObs.t % If new data is measurement
    if ~hasPredicted
        % >> Perform prediction -> xMinus
        %         >> estimation(indEstimation) = estimation(indEstimation - 1) + xMinus;
        %         >> lastTimestamp = imuMeas.t;
        %             indEstimation = indEstimation + 1;
        %             hasPredicted = true;
        
    else % If new data is observation
        % Compute satellites' states
        [satPos, satClkBias, satClkDrift, satVel] = ...
            compute_satellite_state_all(gnss.tow, gnss.obs, nav, Config.CONSTELLATIONS);
        
        % Remove invalid observations (no ephem, elevation mask, large dopplers)
        [gnss.obs, satPos, satClkBias, satClkDrift, satVel] = ...
            filterObs(gnss.obs, satPos, satClkBias, satClkDrift, satVel, xEst(1:3));
        
        % Compute elevation and azimuth of satellites
        [satAzDeg, satElDeg, rxLLH] = getSatAzEl(satPos, xEst(1:3));
        
        % >> Apply iono and tropo corrections
        switch Config.IONO_CORRECTION
            case 'Klobuchar'
                iono = compute_klobuchar_iono_correction(...
                    alpha,              ...
                    beta,               ...
                    deg2rad(satElDeg),  ...
                    deg2rad(satAzDeg),  ...
                    deg2rad(rxLLH(1)),  ...
                    deg2rad(rxLLH(2)),  ...
                    gnss.tow);
                iono = iono .* (Constants.GPS_L1_HZ./[gnss.obs(:).D_fcarrier_Hz]).^2;
            otherwise
                iono = zeros(1, length(gnss.obs));
        end
        tropo = compute_saastamoinen_tropo_correction(rxLLH(3), deg2rad(satElDeg), deg2rad(rxLLH(1)));
        % Apply pr correction and convert doppler (Hz) to pr rate (m/s)
        prCorr = [gnss.obs(:).C]' - iono' - tropo';
        prRate = -[gnss.obs(:).D_Hz]' .* Constants.CELERITY ./ [gnss.obs(:).D_fcarrier_Hz]';
        
        % Transition model arguments
        fArgs.xEst = xEst;
        % Measurement model arguments
        hArgs.xEst = xEst;
        hArgs.prCorr = prCorr;
        hArgs.prRate = prRate;
        hArgs.satPos = satPos;
        hArgs.satVel = satVel;
        hArgs.satClkBias = satClkBias;
        hArgs.satClkDrift = satClkDrift;
        hArgs.satElev = satElDeg;
        hArgs.sigmaPr = [gnss.obs(:).C_sigma];
        hArgs.sigmaPrRate = [gnss.obs(:).D_sigma] .* Constants.CELERITY ./ [gnss.obs(:).D_fcarrier_Hz];
        
        esekf = EKF.processObservation(esekf, gnss.t, ...
            @fTransition, fArgs, ...
            @hMeasurement, hArgs, ...
            'gnssObs');
        % >> reset error state
        %             % If last estimation is a prediction and current observation is
        %             % close enough in time, perform update
        %             if hasPredicted && gnssObs.t - imuMeas.t <= maxUpdateTime
        %                 % >> Perform update over last prediction -> xPlus
        %                 %             >> estimation(indEstimation - 1) = estimation(indEstimation - 2) + xPlus;
        %
        %
        %             else % Otherwise, estimate without IMU
        %                 % >> Estimation without IMU -> xPlus
        %                 %             >> estimation(indEstimation) = estimation(indEstimation - 1) + xPlus;
        %                 %             >> indEstimation = indEstimation + 1;
        %             end
        lastUtcMillis = gnss.t;
        xEst = esekf.x;
        indEstimation = indEstimation + 1;
        %         >> hasPredicted = false;
        
    end
    % Check if there are more measurements/observations
    %     >> hasEnded = isempty(getMeas(indImu)) && isempty(getObs(indGnss));
end

end

% State transition function
function [x2, F, Q] = fTransition(x1, t1, t2, fArgs)
dt = t2 - t1;
F = eye(10);
% dp/dv
F(1,4) = dt; F(2,5) = dt; F(3,6) = dt;
% d clkbias/d clkdrift
F(7,8) = dt;
% State prediction
x2 = F * x1;
% Process noise covariance matrix
rotN2E = compute_Rn2e(fArgs.xEst(1), fArgs.xEst(2), fArgs.xEst(3));
Qvel = dt * rotN2E * diag(Config.SIGMA_VEL_NED.^2) * rotN2E';
Qclk(1, 1) = dt * Config.SIGMA_CLK_BIAS^2 + 1/3 * dt^3 * Config.SIGMA_CLK_DRIFT^2;
Qclk(1, 2) = 1/2 * dt^2 * Config.SIGMA_CLK_DRIFT^2;
Qclk(2, 1) = Qclk(1, 2);
Qclk(2, 2) =  dt * Config.SIGMA_CLK_DRIFT^2;
Qif = dt * Config.SIGMA_CLK_INTERFREQ^2 * eye(Config.getNumFreq - 1);
Qis = dt * Config.SIGMA_CLK_INTERSYS^2 * eye(Config.getNumConst - 1);
Q = blkdiag(zeros(3), Qvel, Qclk, Qif, Qis);
end


% Measurement function
function [z, y, H, R] = hMeasurement(~, hArgs)
nObs = length(hArgs.obsC);
freqs = unique(hArgs.obsFreq);
nFreqs = length(freqs);

indGoodC = find(~isnan(hArgs.obsC));
nObsC = length(indGoodC);
indGoodD = find(~isnan(hArgs.obsD));
nObsD = length(indGoodD);

z = [hArgs.obsC(indGoodC) hArgs.obsD(indGoodD)];

H = zeros(nObsC + nObsD, 8  + nFreqs-1 + Config.getNumConst()-1);
y = zeros(size(z));
iRow = 1;
for iObs = 1:nObs
    % check frequency and constellation of the observable
    idxFreq = find(freqs == hArgs.obsFreq(iObs));
    idxConst = strfind(Config.CONSTELLATIONS, hArgs.obsConst(iObs));
    % measurement prediction at x0 (excluding tropo and iono delays which are already corrected on the measurements)
    dist = norm(hArgs.xEst(1:3) - hArgs.satPosC(:, iObs)) + ...
        Constants.OMEGA_E/Constants.CELERITY * ...
        (hArgs.satPosC(1, iObs)*hArgs.xEst(2) - hArgs.satPosC(2, iObs)*hArgs.xEst(1));
    % Unit vector from user to satellite
    vUS = unitVector(hArgs.satPos(:,iObs) - hArgs.xEst(1:3));
    if ~isnan(hArgs.obsC(iObs))
        % Fill y and H for Code measurements
        y(iRow) = dist + (hArgs.xEst(7) - ...
            Constants.CELERITY * hArgs.satClkBias(iObs)) + ...
            hArgs.xEst(8+idxFreq-1)*(idxFreq>1) + ...
            hArgs.xEst(8+nFreqs-1+idxConst-1)*(idxConst>1);
        % Jacobian matrix: C rows
        H(iRow, 1:3) = (hArgs.xEst(1:3) - hArgs.satPosC(:, iObs))/dist;
        H(iRow, 7) = 1;
        H(iRow, 8+idxFreq-1) = idxFreq > 1; % 1 if not 1st freq
        H(iRow, 8+nFreqs-1+idxFreq-1) = -(idxConst > 1); % -1 if not 1st const
    end
    
    if ~isnan(hArgs.obsD(iObs))
        % Fill y and H for Doppler measurements
        y(nObsC+iRow) = dot(hArgs.satVel(:, iObs) - hArgs.xEst(4:6), vUS) ...
            + hArgs.xEst(8) - MagnitudeConstants.celerity*hArgs.satClkDrift(iObs);
        % Jacobian matrix: D rows
        H(nObsC+iObsC, 4:6) = (hArgs.xEst(1:3) - hArgs.satPosD(:, iObsD))/dist;
        H(nObsC+iObsC, 8) = 1;
    end
end
assert(isequal(size(y), size(z)), 'Vectors y and z should be the same size');

Rpr = computeMeasCovariance(hArgs.obsEl(indGoodC), hArgs.sigmaPr(indGoodC), ...
    Config.SIGMA_PR_M, hArgs.constellation(indGoodC));
Rdop = computeMeasCovariance(hArgs.obsEl(indGoodD), hArgs.sigmaPrRate(indGoodD), ...
    Config.SIGMA_DOP_MPS, hArgs.constellation(indGoodD));
R = blkdiag(Rpr, Rdop);
end