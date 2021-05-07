function [xEstHist, prInnovations, prInnovationCovariances, dopInnovations, dopInnovationCovariances, refInnovations, refInnovationCovariances, utcSecondsHist] = ...
    navigate(gnssRnx, imuMeas, nav, iono, ref)
if nargin <1
   run main.m; 
end

%NAVIGATE Summary of this function goes here
%   Detailed explanation goes here

%% Initializations
nStates = PVTUtils.getNumStates();
nSatellites = PVTUtils.getNumSatelliteIndices();
gnssEpochs = unique(gnssRnx.utcSeconds);
nGnssEpochs = length(gnssEpochs);
% EKF:
esekf = EKF.build(uint16(nStates));


%% Obtain first position
[esekf.x, esekf.P, esekf.tx] = getFirstPosition(gnssRnx, nav);

% Loop variables
% thisUtcSeconds -> time ref for both IMU and GNSS.
thisUtcSeconds = esekf.tx; % First time is from the first GNSS estimation
idxEst = 1;
xEstHist = esekf.x; % TODO change to error state, prealocate if num pos is known (= groundtruth?)

% First gnss observations is the same as for the first approx position
gnss = getNextGnss(thisUtcSeconds, gnssRnx, 'this');
hasEnded = isempty(gnss); % TODO check imu

% TODO: debugging variables, check which need to be kept
prInnovations = nan(nSatellites, nGnssEpochs);
prInnovationCovariances = nan(nSatellites, nGnssEpochs);
dopInnovations = nan(nSatellites, nGnssEpochs);
dopInnovationCovariances = nan(nSatellites, nGnssEpochs);
refInnovations = nan(3, nGnssEpochs);
refInnovationCovariances = nan(3, nGnssEpochs);
utcSecondsHist = []; % TODO size is not known yet since it depends on predictions and updates
idxRef = 1;
lastTow = gnss.tow;

pp = csaps(gnssRnx.obs(:, 2), gnssRnx.utcSeconds);
ref.utcSeconds = fnval(pp, ref.gpsTime(:, 2));

while ~hasEnded % while there are more observations/measurements
    
    % First iteration: x0 is result from LS
    if idxEst == 1,     x0 = xEstHist(:, 1);
    else,               x0 = xEstHist(:, idxEst-1); end
    % Transition model arguments
    fArgs.x0 = x0;
    
    % If next ref pos is more recent, process it as observation << TODO: only for debugging
%     idxRef = find(ref.gpsTime(:, 2) > lastTow & ref.gpsTime(:, 2) < gnss.tow, 1, 'first');
    idxRef = find(ref.utcSeconds > thisUtcSeconds, 1, 'first');
    if ~isempty(idxRef) && ref.utcSeconds(idxRef) < gnss.utcSeconds && false
%     if ~isempty(idxRef)
        thisUtcSeconds = ref.utcSeconds(idxRef);
        
        % Measurement model arguments
        hArgs.x0 = x0;
        hArgs.obs = Lla2Xyz(ref.posLla(idxRef, :))';
        hArgs.sigmaObs = [1e-1 1e-1 1e-1]';
        
        [esekf, innovation, innovationCovariance, rejected, z, y] = ...
            EKF.processObservation(esekf, thisUtcSeconds, ...
            @fTransition, fArgs, ...
            @hRefObs, hArgs, ...
            'Reference');
        
        refInnovations(:, idxEst) = innovation;
        refInnovationCovariances(:, idxEst) = diag(innovationCovariance);
        
%         lastTow = ref.gpsTime(idxRef, 2);
    else
%         hasEnded = true;
        thisUtcSeconds = gnss.utcSeconds; % TODO check imu time
        
        % Get states of satellites selected in Config
        [satPos, satClkBias, satClkDrift, satVel] = ...
            compute_satellite_state_all(gnss.tow, gnss.obs, nav, Config.CONSTELLATIONS);
        
        % Remove invalid observations (no ephem, elevation mask, large dopplers)
        [gnss.obs, satPos, satClkBias, satClkDrift, satVel] = ...
            filterObs(gnss.obs, satPos, satClkBias, satClkDrift, satVel, x0(1:3));
        
        if length([gnss.obs(:).C]) < 4 + PVTUtils.getNumFrequencies + PVTUtils.getNumConstellations
            warning('TOW = %d - Not enough observations to estimate a potition. Propagating state.', gnss.tow);
            esekf = EKF.propagateState(esekf, thisUtcSeconds, @fTransition, fArgs);
        else
            % Compute elevation and azimuth of satellites
            [satAzDeg, satElDeg, rxLLH] = getSatAzEl(satPos, x0(1:3));
            
            % Apply iono and tropo corrections
            switch Config.IONO_CORRECTION
                case 'Klobuchar'
                    ionoCorr = compute_klobuchar_iono_correction(...
                        iono.alpha,              ...
                        iono.beta,               ...
                        deg2rad(satElDeg),  ...
                        deg2rad(satAzDeg),  ...
                        deg2rad(rxLLH(1)),  ...
                        deg2rad(rxLLH(2)),  ...
                        gnss.tow);
                    ionoCorr = ionoCorr .* (Constants.GPS_L1_HZ./[gnss.obs(:).D_fcarrier_Hz]).^2;
                otherwise
                    ionoCorr = zeros(1, length(gnss.obs));
            end
            tropo = compute_saastamoinen_tropo_correction(rxLLH(3), deg2rad(satElDeg), deg2rad(rxLLH(1)));
%             tropo = 0; % TODO apply tropo
            % Apply pr correction and convert doppler (Hz) to pr rate (m/s)
            prCorr = [gnss.obs(:).C]' - ionoCorr' - tropo';
            prRate = -[gnss.obs(:).D_Hz]' .* Constants.CELERITY ./ [gnss.obs(:).D_fcarrier_Hz]';
            
            
            % Measurement model arguments
            hArgs.x0 = x0;
            
            % Sequentally update with all observations
            for iObs = 1:length(prCorr)
                idxSat = PVTUtils.getSatelliteIndex(gnss.obs(iObs).prn, gnss.obs(iObs).constellation);
                % Pack arguments that are common for Code and Doppler observations
                hArgs.obsFreqHz = gnss.obs(iObs).D_fcarrier_Hz;
                hArgs.obsConst = gnss.obs(iObs).constellation;
                hArgs.satPos = satPos(:, iObs);
                hArgs.satVel = satVel(:, iObs);
                hArgs.satClkBias = satClkBias(iObs);
                hArgs.satClkDrift = satClkDrift(iObs);
                hArgs.satElev = satElDeg(iObs);
                
                % Pack code observation
                hArgs.obs = prCorr(iObs);
                hArgs.sigmaObs = gnss.obs(iObs).C_sigma;
                % Process code observation
                [esekf, innovation, innovationCovariance, rejected, z, y] = ...
                    EKF.processObservation(esekf, thisUtcSeconds, ...
                    @fTransition, fArgs, ...
                    @hCodeObs, hArgs, ...
                    'Code');
                prInnovations(idxSat, idxEst) = innovation;
                prInnovationCovariances(idxSat, idxEst) = innovationCovariance;
                hArgs.x0 = esekf.x;
                
%                 % TODO provisional outlier removal
                if abs(prRate(iObs)) < (Config.MAX_DOPPLER_MEAS * Constants.CELERITY / hArgs.obsFreqHz) && ...
                        gnss.obs(iObs).D_sigma < Config.MAX_DOPPLER_UNCERT
                    % Pack Doppler observation
                    hArgs.obs = prRate(iObs);
                    hArgs.sigmaObs = gnss.obs(iObs).D_sigma .* ...
                        Constants.CELERITY ./ hArgs.obsFreqHz; % Doppler sigma in mps
                    % Process Doppler observation
                    [esekf, innovation, innovationCovariance, rejected, z, y] = ...
                        EKF.processObservation(esekf, thisUtcSeconds, ...
                        @fTransition, fArgs, ...
                        @hDopplerObs, hArgs, ...
                        'Doppler');
                    dopInnovations(idxSat, idxEst) = innovation;
                    dopInnovationCovariances(idxSat, idxEst) = innovationCovariance;
                    %         else
                    %             a=1;
                    hArgs.x0 = esekf.x;
                end
                
            end
        end
%         lastTow = gnss.tow; % TODO: used to compare with ref time, only for testing
    end
    
    utcSecondsHist(idxEst) = thisUtcSeconds;
    
    %%%%%%%%%%%%%%%%%% DEBUG %%%%%%%%%%%%%%%% TODO
    %     esekf.tx
    %     Xyz2Lla(esekf.x(1:3)')
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    xEstHist(:, idxEst) = esekf.x; % TODO perform correction when using error-state
    idxEst = idxEst + 1;
    
    % Check if there are more measurements/observations
    gnss = getNextGnss(thisUtcSeconds, gnssRnx);
    hasEnded = isempty(gnss); % TODO check imu
    
end

end

% State transition function
function [x2, F, Q] = fTransition(x1, t1, t2, fArgs)
dt = t2 - t1;
F = eye(PVTUtils.getNumStates);
% dp/dv
F(1,4) = dt; F(2,5) = dt; F(3,6) = dt;
% d clkbias/d clkdrift
F(7,8) = dt;
% State prediction
x2 = F * x1;
% Process noise covariance matrix - Velocity
rotN2E = compute_Rn2e(fArgs.x0(1), fArgs.x0(2), fArgs.x0(3));
Qvel = dt * rotN2E * diag(Config.SIGMA_VEL_NED.^2) * rotN2E';
% Process noise covariance matrix - Clock
Qclk(1, 1) = dt * Config.SIGMA_CLK_BIAS^2 + 1/3 * dt^3 * Config.SIGMA_CLK_DRIFT^2;
Qclk(1, 2) = 1/2 * dt^2 * Config.SIGMA_CLK_DRIFT^2;
Qclk(2, 1) = Qclk(1, 2);
Qclk(2, 2) =  dt * Config.SIGMA_CLK_DRIFT^2;
% Process noise covariance matrix - Inter-Frequency bias(es)
Qif = dt * Config.SIGMA_CLK_INTERFREQ^2 * eye(PVTUtils.getNumFrequencies - 1);
% Process noise covariance matrix - Inter-System bias(es)
Qis = dt * Config.SIGMA_CLK_INTERSYS^2 * eye(PVTUtils.getNumConstellations - 1);
% Process noise covariance matrix
Q = blkdiag(zeros(3), Qvel, Qclk, Qif, Qis);
end %end of function fTransition
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [z, y, H, R] = hRefObs(~, hArgs)

idxStatePos = PVTUtils.getStateIndex(PVTUtils.ID_POS);

z = hArgs.obs;

y = hArgs.x0(idxStatePos);

% Jacobian matrix
H = zeros(3, PVTUtils.getNumStates);
H(idxStatePos,idxStatePos) = eye(3);

R = diag(hArgs.sigmaObs);
end

function [z, y, H, R] = hCodeObs(~, hArgs)
% HCODEOBS provides the measurement model for the sequential code
% observations

% Initializations
idxStatePos = PVTUtils.getStateIndex(PVTUtils.ID_POS);
idxStateClkBias = PVTUtils.getStateIndex(PVTUtils.ID_CLK_BIAS);
idxStateFreq = PVTUtils.getStateIndex(PVTUtils.ID_INTER_FREQ_BIAS);
idxStateConstel = PVTUtils.getStateIndex(PVTUtils.ID_INTER_SYS_BIAS);
idxFreq = PVTUtils.getFreqIdx(hArgs.obsFreqHz);
idxConstel = PVTUtils.getConstelIdx(hArgs.obsConst);

% Observation
z = hArgs.obs;

% Distance between receiver and satellite. Second term is Sagnac effect.
dist = norm(hArgs.x0(idxStatePos) - hArgs.satPos) + ...
    Constants.OMEGA_E/Constants.CELERITY * ...
    (hArgs.satPos(1)*hArgs.x0(idxStatePos(2)) - ...
    hArgs.satPos(2)*hArgs.x0(idxStatePos(1)));

% Inter-frequency bias if it's not ref freq
if isempty(idxStateFreq) || idxFreq == 1, interFreqBias = 0; % If only 1 freq or this is ref freq
else, interFreqBias = hArgs.x0(idxStateFreq(idxFreq-1)); end
% Inter-system bias if it's not ref system
if isempty(idxStateConstel) || idxConstel == 1, interSysBias = 0; % If only 1 const or this is ref const
else, interSysBias = hArgs.x0(idxStateConstel(idxConstel-1)); end
% Observation estimation
y = dist + ...                                  % Distance between receiver and satellite.
    hArgs.x0(idxStateClkBias) - ...             % Receiver clock bias
    Constants.CELERITY * hArgs.satClkBias + ... % Satellite clock bias
    interFreqBias - ...                         % Inter-frequency bias
    interSysBias;                               % Inter-system bias

% Jacobian matrix
H = zeros(1, PVTUtils.getNumStates);
H(idxStatePos) = (hArgs.x0(idxStatePos) - hArgs.satPos)/dist;
H(idxStateClkBias) = 1;
H(idxStateFreq) = idxFreq > 1; % 1 if not ref freq
H(idxStateConstel) = -(idxConstel > 1); % -1 if not ref const

% Measurement covariance matrix
R = Config.COV_FACTOR_C * computeMeasCovariance(hArgs.satElev, ...
    hArgs.sigmaObs, Config.SIGMA_PR_M, hArgs.obsConst);
end %end of function hCodeObs
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [z, y, H, R] = hDopplerObs(~, hArgs)
% HDOPPLEROBS provides the measurement model for the sequential Doppler
% observations

% Initializations
idxStatePos = PVTUtils.getStateIndex(PVTUtils.ID_POS);
idxStateVel = PVTUtils.getStateIndex(PVTUtils.ID_VEL);
idxStateClkDrift = PVTUtils.getStateIndex(PVTUtils.ID_CLK_DRIFT);

% Observation
z = hArgs.obs;

% Distance between receiver and satellite. Second term is Sagnac effect.
dist = norm(hArgs.x0(idxStatePos) - hArgs.satPos) + ...
    Constants.OMEGA_E/Constants.CELERITY * ...
    (hArgs.satPos(1)*hArgs.x0(idxStatePos(2)) - ...
    hArgs.satPos(2)*hArgs.x0(idxStatePos(1)));

% Unit vector from user to satellite
vUS = unitVector(hArgs.satPos - hArgs.x0(idxStatePos));

% Observation estimation
y = dot(hArgs.satVel - hArgs.x0(idxStateVel), vUS) + ...  % Radial velocity from user to sat
    hArgs.x0(idxStateClkDrift) - ...                      % Receiver clock drift
    Constants.CELERITY*hArgs.satClkDrift;                   % Satellite clock drift

% Jacobian matrix
H = zeros(1, PVTUtils.getNumStates);
H(idxStatePos) = (hArgs.x0(idxStatePos) - hArgs.satPos)/dist;
H(idxStateClkDrift) = 1;

% Measurement covariance matrix
R = Config.COV_FACTOR_D * computeMeasCovariance(hArgs.satElev, ...
    hArgs.sigmaObs, Config.SIGMA_DOP_MPS, hArgs.obsConst);
end %end of function hDopplerObs
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%