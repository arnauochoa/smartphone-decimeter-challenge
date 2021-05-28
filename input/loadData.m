function [phoneRnx, imuRaw, nav, iono, osrRnx, ref] = loadData()
% LOADDATA Loads the GNSS observations, IMU measurements, navigation data
% and groundtruth data

config = Config.getInstance;

%% GNSS observations and IMU measurements
if isprop(config, 'OBS_RINEX_PATH') % Use observations from rinex
    [phoneRnx.obs, phoneRnx.type] = rinex_v3_obs_parser(config.OBS_RINEX_PATH);
    imuRaw.acc = [];
    imuRaw.gyr = [];
    imuRaw.mag = [];
    phoneRnx.utcSeconds = phoneRnx.obs(:, 2);
else % Use observations from GnssLog
    [obsDirPath, obsFileName] = getObsDirFile(config);
    [phoneRnx.obs, phoneRnx.type, obsRinexUtcMillis, ~, ...   % GNSS data in Rinex-shaped matrix
        imuRaw.acc, imuRaw.gyr, imuRaw.mag, ~] = ...        % IMU data
        getGnssLogObs(obsDirPath, obsFileName, config.FILTER_RAW_MEAS);
    
    phoneRnx.utcSeconds = obsRinexUtcMillis / 1e3;
end

%% Navigation data
nav = rinex_v3_nav_parser(getNavFilepaths(config));

%% Ionospheric data
iono.alpha = [.4657E-08   .1490E-07  -.5960E-07  -.1192E-06]';
iono.beta = [.8192E+05   .9830E+05  -.6554E+05  -.5243E+06]';

%% OSR data
osrRnx.obs = [];
osrFilepaths = getOSRFilepaths(config);
for iOsr = 1:length(osrFilepaths)
    [obs, type] = rinex_v3_obs_parser(osrFilepaths{iOsr});
    osrRnx.obs = [osrRnx.obs; obs];
end
osrRnx.type = type;

%% Groundtruth data
if isprop(config, 'OBS_RINEX_REF_XYZ') % Use observations from rinex
    [ref.tow, idxUnq] = unique(phoneRnx.obs(:, 2));
    ref.wNum = phoneRnx.obs(idxUnq, 1);
    ref.utcSeconds = ref.tow;
    [refLat, refLon, refAlt] = ecef2geodetic(wgs84Ellipsoid, ...
        config.OBS_RINEX_REF_XYZ(1), ...
        config.OBS_RINEX_REF_XYZ(2), ...
        config.OBS_RINEX_REF_XYZ(3));
    ref.posLla = repelem([refLat, refLon, refAlt], length(ref.tow), 1);
else
    disp('Reading groundtruth file...');
    [refDirPath, refFileName] = getRefDirFile(config);
    refCachePath = [refDirPath refFileName(1:end-5) '.mat'];
    if exist(refCachePath, 'file')
        load(refCachePath, 'ref_nmea');
    else
        [ref_nmea,~] = ReadNmeaFile(refDirPath, refFileName);
        save(refCachePath, 'ref_nmea');
        disp('Groundtruth data saved in .mat file.')
    end
    % Extract groundtruth positions
    ggaVec = [ref_nmea.Gga];
    rmcVec = [ref_nmea.Rmc];
    ref.posLla = [[ggaVec.LatDeg]' [ggaVec.LonDeg]' [ggaVec.AltM]'];
    % Transform groundtruth's UTC time to GPS time
    [refGpsTime, ref.gpsSeconds, ~]= Utc2Gps(datevec([rmcVec.Datenum]));
    ref.wNum = refGpsTime(:, 1);
    ref.tow = refGpsTime(:, 2);
    % Obtain groundtruth's UTC time
    pp = csaps(phoneRnx.obs(:, 2), phoneRnx.utcSeconds);
    ref.utcSeconds = fnval(pp, ref.tow);
end
end