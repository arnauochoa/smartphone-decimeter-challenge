function [x0, P0, utcSeconds0] = getFirstPosition(gnssRnx, nav)
% GETFIRSTPOSITION Computes the position using LS at the first epoch with
% enough GNSS observations

% Initializations
firstValidEpoch = false;
thisUtcSeconds = 0;

% Find the first epoch that has enough observations
while ~firstValidEpoch
    % Obtain observations of next epoch
    gnss = getNextGnss(thisUtcSeconds, gnssRnx);
    thisUtcSeconds = gnss.utcSeconds;
    
    % Get states of satellites selected in Config
    [satPos, satClkBias, ~, ~] = ...
        compute_satellite_state_all(gnss.tow, gnss.obs, nav, Config.CONSTELLATIONS);
    % check if ephemeris is available
    [gnss.obs, satPos, satClkBias, ~] = ...
        clean_obs_vector(gnss.obs, satPos, satClkBias);
    
    % determine if the first epoch contains enough valid measurements
    if length(Config.CONSTELLATIONS) == 1
        firstValidEpoch = (length(unique([gnss.obs(:).prn])) >= 4);
    else
        % check if there are enough satellite of a single constellation
        nSat = zeros(size(Config.CONSTELLATIONS));
        for idxConst = 1:length(Config.CONSTELLATIONS)
            isConst = strfind([gnss.obs(:).constellation], Config.CONSTELLATIONS(idxConst));
            nSat(idxConst) = length(unique([gnss.obs(isConst).prn]));
        end; clear ind_const
        % we can compute a solution if there is more than 1 GPS satellite and more than 3+N_const satellites
        firstValidEpoch = (nSat(1) > 0) * (sum(nSat)>3 + length(Config.CONSTELLATIONS));
    end
    
    % if the current epoch is not valid, discard the corresponding observations from the observation matrix
    if ~firstValidEpoch
        % Indices of invalid observations in gnssRnx
        idxInvalid = gnssRnx.utcSeconds == thisUtcSeconds;
        disp(['Skipped epoch ' num2str(gnss.tow) ]);
        gnssRnx.obs(idxInvalid,:) = [];
        gnssRnx.utcSeconds(idxInvalid) = [];
%         gnssRnx.tow(idxInvalid) = [];
    end
end
% compute LS position
% Approximate position is at lat = 0, lon = 0, alt = 0, clk = 0, interconstellation clk bias = 0
xLS = [Constants.EARTH_RADIUS 0 0 0 zeros(1, PVTUtils.getNumConstellations()-1)]';
[xLS, PLS] = compute_spp_ls(gnss.obs, satPos, satClkBias, xLS, Config.CONSTELLATIONS);

% Fill ouptuts with LS estimates
[x0, P0] = fillFullState(xLS, PLS);
utcSeconds0 = thisUtcSeconds;
end %end of function getFirstPosition
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [x0, P0] = fillFullState(xLS, PLS)
% FILLFULLSTATE Fills full state vector and covariance matrix with 
% parameters estimated by LS and the ones set by Config

% Initialize state vector and cov matrix
nStates = PVTUtils.getNumStates();
x0 = zeros(nStates, 1);
P0 = zeros(nStates); % TODO set diagonal terms of velocity and others

% Fill parameters estimated by LS
idxLS = PVTUtils.getStateIndex(PVTUtils.ID_POS);
x0(idxLS) = xLS(1:3);
P0(idxLS, idxLS) = PLS(1:3,1:3);

% Fill the rest with the Config values
P0(PVTUtils.getStateIndex(PVTUtils.ID_VEL), PVTUtils.getStateIndex(PVTUtils.ID_VEL)) = ...
    diag(Config.SIGMA_P0_VEL_XYZ.^2);
end