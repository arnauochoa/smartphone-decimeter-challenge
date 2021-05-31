close all;
% test for interpOSR

%% Input
[gnssRnx, imuRaw, nav, iono, osrRnxRaw, ref] = loadData();

%% Interpolate OSR data
osrRnx = interpOSR(osrRnxRaw, gnssRnx);

rxStartTow = min(gnssRnx.obs(:, 2));
rxEndTow = max(gnssRnx.obs(:, 2));

osrConstSat = unique(osrRnx.obs(:, GnssLogUtils.COL_CONST:GnssLogUtils.COL_SVN), 'rows');
nOsrSats = size(osrConstSat, 1);

obsRxInd = 6;
obsOSRInd = [6 6 9 8]; % [GPS GLO BDS GAL]

for iSat = 1:nOsrSats
    if any(osrConstSat(iSat, 1) == [GnssLogUtils.OBS_ID_GPS GnssLogUtils.OBS_ID_BDS GnssLogUtils.OBS_ID_GAL])
        idxSatRaw = osrRnxRaw.obs(:, 3) == osrConstSat(iSat, 1) & osrRnxRaw.obs(:, 4) == osrConstSat(iSat, 2);
        idxSatInterp = osrRnx.obs(:, 3) == osrConstSat(iSat, 1) & osrRnx.obs(:, 4) == osrConstSat(iSat, 2);
        idxSatRx = gnssRnx.obs(:, 3) == osrConstSat(iSat, 1) & gnssRnx.obs(:, 4) == osrConstSat(iSat, 2);

        figure;
    %     subplot(2,1,1)
        hold on;
        plot(osrRnxRaw.obs(idxSatRaw, 2), osrRnxRaw.obs(idxSatRaw, obsOSRInd(osrConstSat(iSat, 1))), 'x', 'MarkerSize', 2);
        plot(osrRnx.obs(idxSatInterp, 2), osrRnx.obs(idxSatInterp, obsOSRInd(osrConstSat(iSat, 1))), '.');
        plot(gnssRnx.obs(idxSatRx, 2), gnssRnx.obs(idxSatRx, obsRxInd), '.');
    %     xline(rxStartTow); xline(rxEndTow);
        legend('Raw', 'Interp', 'Phone');
    %     subplot(2,1,2)
        title([GnssLogUtils.OBS_CONSTELLATIONS(osrConstSat(iSat, 1)) num2str(osrConstSat(iSat, 2))])
    end
end