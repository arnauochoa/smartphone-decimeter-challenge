function [x0, P0, utcSeconds0, xWLS, phoneInfo] = getFirstPosition(phones, nav)
% GETFIRSTPOSITION Computes the position using LS at the first epoch with
% enough GNSS observations

% Initializations
config = Config.getInstance;
firstValidEpoch = false;
thisUtcSeconds = 0;

% Find the first epoch that has enough observations
while ~firstValidEpoch
    % Obtain observations of next epoch
    [phoneGnss, ~, phoneInfo] = getNextGnss(thisUtcSeconds, phones);
    thisUtcSeconds = phoneGnss.utcSeconds;
    
    % Get states of satellites selected in Config
    [satPos, satClkBias, ~, ~] = ...
        compute_satellite_state_all(phoneGnss.tow, phoneGnss.obs, nav, config.CONSTELLATIONS);
    % check if ephemeris is available
    [phoneGnss.obs, satPos, satClkBias, ~] = ...
        clean_obs_vector(phoneGnss.obs, satPos, satClkBias);
    
    % determine if the first epoch contains enough valid measurements
    if length(config.CONSTELLATIONS) == 1
        firstValidEpoch = (length(unique([phoneGnss.obs(:).prn])) >= 4);
    else
        % check if there are enough satellite of a single constellation
        nSat = zeros(size(config.CONSTELLATIONS));
        for idxConst = 1:length(config.CONSTELLATIONS)
            isConst = strfind([phoneGnss.obs(:).constellation], config.CONSTELLATIONS(idxConst));
            nSat(idxConst) = length(unique([phoneGnss.obs(isConst).prn]));
        end; clear ind_const
        % we can compute a solution if there is more than 1 GPS satellite and more than 3+N_const satellites
        firstValidEpoch = (nSat(1) > 0) * (sum(nSat)>3 + length(config.CONSTELLATIONS));
    end
    
    % if the current epoch is not valid, discard the corresponding observations from the observation matrix
    if ~firstValidEpoch
        % Indices of invalid observations in gnssRnx
        idxInvalid = phoneRnx.utcSeconds == thisUtcSeconds;
        disp(['Skipped epoch ' num2str(phoneGnss.tow) ]);
        phoneRnx.obs(idxInvalid,:) = [];
        phoneRnx.utcSeconds(idxInvalid) = [];
        %         gnssRnx.tow(idxInvalid) = [];
    end
end
% compute LS position
% Approximate position is at lat = 0, lon = 0, alt = 0, clk = 0, interconstellation clk bias = 0
[obsConstel, idxObsConst] = intersect(Config.CONSTELLATIONS, unique([phoneGnss.obs.constellation]), 'stable');
xLS = [Constants.EARTH_RADIUS 0 0 0 zeros(1, length(obsConstel)-1)]';
[xLS, PLS] = compute_spp_ls(phoneGnss.obs, satPos, satClkBias, xLS, obsConstel);
% Obtain satellite elevations
[~, satElDeg, ~] = getSatAzEl(satPos, xLS);
R = computeMeasCovariance(satElDeg, [phoneGnss.obs(:).C_sigma], Config.SIGMA_D_MPS, [phoneGnss.obs(:).constellation]);
[xWLSaux, ~, PWLSaux, ~, ~] = compute_spp_wls([phoneGnss.obs(:).C]', [phoneGnss.obs(:).constellation], satPos, satClkBias, xLS, R, obsConstel);
% Save all states including missing constellations
nStatesWLS = 4 + PVTUtils.getNumConstellations - 1;
idxComputedStates = [1:4, 4+idxObsConst(2:end)'-1]; % pos, clock, inter-const bias
xWLS = zeros(nStatesWLS, 1);
PWLS = zeros(nStatesWLS);
xWLS(idxComputedStates) = xWLSaux;
PWLS(idxComputedStates, idxComputedStates) = PWLSaux;

% Fill ouptuts with LS estimates
[x0, P0] = fillFullState(xWLS, PWLS);
utcSeconds0 = thisUtcSeconds;
end %end of function getFirstPosition
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [x0, P0] = fillFullState(xLS, PLS)
% FILLFULLSTATE Fills full state vector and covariance matrix with
% parameters estimated by LS and the ones set by Config
config = Config.getInstance;
nPhones = length(config.phoneNames);
idxStatePos = PVTUtils.getStateIndex(PVTUtils.ID_POS);
idxStateAtt = PVTUtils.getStateIndex(PVTUtils.ID_ATT_XYZ);
idxStateVel = PVTUtils.getStateIndex(PVTUtils.ID_VEL);
idxStateClkDrift = PVTUtils.getStateIndex(PVTUtils.ID_CLK_DRIFT, 1:nPhones);
idxStateAllSdAmb = PVTUtils.getStateIndex(PVTUtils.ID_SD_AMBIGUITY, 1:nPhones);

% Initialize state vector and cov matrix
nStates = PVTUtils.getNumStates();
x0 = zeros(nStates, 1);
x0(idxStateAllSdAmb) = nan; % All ambiguities to nan to account for sats never in view
P0 = zeros(nStates);

% Fill parameters estimated by LS
x0(idxStatePos) = xLS(1:3);
P0(idxStatePos, idxStatePos) = config.FACTOR_P0_POS*PLS(1:3,1:3);

%% Fill the rest with the Config values
% Attitude
if ~isempty(idxStateAtt)
    x0(idxStateAtt) = config.X0_ATT_XYZ;
    P0(idxStateAtt, idxStateAtt) = diag(config.SIGMA_P0_ATT_XYZ.^2);
end
% Velocity
P0(idxStateVel, idxStateVel) = diag(config.SIGMA_P0_VEL_XYZ.^2);
if ~isempty(idxStateClkDrift)
    P0(idxStateClkDrift, idxStateClkDrift) = config.SIGMA_P0_CLK_DRIFT.^2 * eye(length(idxStateClkDrift));
end
% Ambiguities
if ~isempty(idxStateAllSdAmb)
    P0(idxStateAllSdAmb, idxStateAllSdAmb) = config.SIGMA_P0_SD_AMBIG.^2 * eye(length(idxStateAllSdAmb));
end
end