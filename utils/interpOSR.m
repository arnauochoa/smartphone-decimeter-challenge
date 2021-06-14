function osrRnxOut = interpOSR(osrRnxRaw, gnssRnx)
% INTERPOSR Interpolates OSR data to match gnssRnx time

config = Config.getInstance;

rxWkNum = unique(gnssRnx.obs(:, GnssLogUtils.COL_WN));
osrWkNum = unique(osrRnxRaw.obs(:, GnssLogUtils.COL_WN));

assert(length(rxWkNum) == 1 && rxWkNum == osrWkNum, 'Week numbers do not match');

rxTow = unique(gnssRnx.obs(:, GnssLogUtils.COL_TOW));
wNumVec = rxWkNum * ones(size(rxTow));

osrConstSat = unique(osrRnxRaw.obs(:, GnssLogUtils.COL_CONST:GnssLogUtils.COL_SVN), 'rows');
nOsrSats = size(osrConstSat, 1);

osrRnxNCols = 4 + max([osrRnxRaw.type.num_obs]);
osrRnxAux = [];

for iSat = 1:nOsrSats
    thisConstId = osrConstSat(iSat, 1);
    % Consider only constellations specified in Config
    if any(thisConstId == GnssLogUtils.getIdsObsConstFromStr(config.CONSTELLATIONS))
        % Number of obs columns for current constellation
        nObsConst = osrRnxRaw.type(thisConstId).num_obs;

        idxThisSat = osrRnxRaw.obs(:, GnssLogUtils.COL_CONST) == thisConstId & ...
            osrRnxRaw.obs(:, GnssLogUtils.COL_SVN) == osrConstSat(iSat, 2);
        thisSatRnx = osrRnxRaw.obs(idxThisSat, :);

        % Matrix of interp obs for current sat
        thisSatObsInterp = nan(length(rxTow), nObsConst);

        for iObs = 1:nObsConst
            % Interpolate only if there are at least two points
            if sum(~isnan(thisSatRnx(:, GnssLogUtils.COL_SVN + iObs))) < 2
                thisSatObsInterp(:, iObs) = nan(size(rxTow));
            else
                % Some sample points may be repeated due to concatenation
                % of RINEX files
                [towVec, obsVec] = ...
                    cleanRepeatedSamplePts(thisSatRnx(:, GnssLogUtils.COL_TOW), ...
                    thisSatRnx(:, GnssLogUtils.COL_SVN + iObs));
                
                thisSatObsInterp(:, iObs) = interp1gap(         ...
                    towVec,                                     ... % OSR obs time
                    obsVec,                                     ... % OSR obs
                    rxTow,                                      ... % Interp time
                    config.MAX_OSR_INTERP_GAP_SEC,              ... % Max gap
                    'spline',                                   ... % Type of interpolation
                    'extrap',nan);                                  % Do not extrapolate
            end
        end
        % TODO append thisSatRnx and fill extra cols with nan
        constSatCols = repelem(osrConstSat(iSat, :), length(rxTow), 1);
        thisSatRnxInterp = [wNumVec rxTow constSatCols thisSatObsInterp nan(length(rxTow), osrRnxNCols - 4 - nObsConst)];
        osrRnxAux = [osrRnxAux; thisSatRnxInterp];
    end
end

% Copy all fields but obs matrix
osrRnxOut = osrRnxRaw;
% Save osrRnx sorted by time, then const, then sat
osrRnxOut.obs = sortrows(osrRnxAux, [GnssLogUtils.COL_WN GnssLogUtils.COL_TOW GnssLogUtils.COL_CONST GnssLogUtils.COL_SVN]);
end