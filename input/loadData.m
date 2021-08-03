function [phones, nav, iono, osrRnx] = loadData()
% LOADDATA Loads the GNSS observations, IMU measurements, navigation data
% and groundtruth data

config = Config.getInstance;
nPhones = length(config.phoneNames);
phones(nPhones, 1) = struct('gnss', [], 'ins', [], 'ref', []);

for iPhone = 1:nPhones
%% GNSS observations and IMU measurements
    if isprop(config, 'OBS_RINEX_PATH') % Use observations from rinex
        [phones(iPhone).gnss.obs, phones(iPhone).gnss.type] = rinex_v3_obs_parser(config.OBS_RINEX_PATH);
        phones(iPhone).ins.acc = [];
        phones(iPhone).ins.gyr = [];
        phones(iPhone).ins.mag = [];
        phones(iPhone).gnss.utcSeconds = phones(iPhone).gnss.obs(:, 2);     % TODO transform to UTC (?)
    else % Use observations from GnssLog
        [obsDirPath, obsFileName] = getObsDirFile(config);
        [phones(iPhone).gnss.obs,       ... % GNSS data in Rinex-shaped matrix
            phones(iPhone).gnss.type,   ...
            obsRinexUtcMillis,          ...
            ~,                          ...             
            phones(iPhone).ins.acc,     ... % IMU data
            phones(iPhone).ins.gyr,     ...
            phones(iPhone).ins.mag,     ...
            ~] = getGnssLogObs(obsDirPath{iPhone}, obsFileName{iPhone}, config.FILTER_RAW_MEAS);
        
        phones(iPhone).gnss.utcSeconds = obsRinexUtcMillis / 1e3;
        % Correct GPS week number (wrong in some campaigns)
        gpst = Utc2Gps(utcSeconds2datevec(phones(iPhone).gnss.utcSeconds));
        phones(iPhone).gnss.obs(:, 1) = gpst(:, 1);
    end
    firstObsGPST = wntow2datetime(phones(iPhone).gnss.obs(1, 1), phones(iPhone).gnss.obs(1, 2));
    lastObsGPST = wntow2datetime(phones(iPhone).gnss.obs(end, 1), phones(iPhone).gnss.obs(end, 2));
    % firstObsGPST = Utc2Gps(utcSeconds2datevec(phones(iPhone).gnss.utcSeconds(1)));
    % firstObsGPST = wntow2datetime(firstObsGPST(1), firstObsGPST(2));
    % lastObsGPST = Utc2Gps(utcSeconds2datevec(phones(iPhone).gnss.utcSeconds(end)));
    % lastObsGPST = wntow2datetime(lastObsGPST(1), lastObsGPST(2));

    %% Groundtruth data
    if contains(config.DATASET_TYPE, 'test')
        phones(iPhone).ref = [];
    else
        if isprop(config, 'OBS_RINEX_REF_XYZ') % Use observations from rinex
            [phones(iPhone).ref.tow, idxUnq] = unique(phones(iPhone).gnss.obs(:, 2));
            phones(iPhone).ref.wNum = phones(iPhone).gnss.obs(idxUnq, 1);
            phones(iPhone).ref.utcSeconds = phones(iPhone).ref.tow;
            [refLat, refLon, refAlt] = ecef2geodetic(wgs84Ellipsoid, ...
                config.OBS_RINEX_REF_XYZ(1), ...
                config.OBS_RINEX_REF_XYZ(2), ...
                config.OBS_RINEX_REF_XYZ(3));
            phones(iPhone).ref.posLla = repelem([refLat, refLon, refAlt], length(phones(iPhone).ref.tow), 1);
        else
            disp('Reading groundtruth file...');
            [refDirPath, refFileName] = getRefDirFile(config);
            refCachePath = [refDirPath{iPhone} refFileName{iPhone}(1:end-5) '.mat'];
            if exist(refCachePath, 'file')
                load(refCachePath, 'ref_nmea');
            else
                [ref_nmea,~] = ReadNmeaFile(refDirPath{1}, refFileName{1});
                save(refCachePath, 'ref_nmea');
                disp('Groundtruth data saved in .mat file.')
            end
            % Extract groundtruth positions
            ggaVec = [ref_nmea.Gga];
            rmcVec = [ref_nmea.Rmc];
            phones(iPhone).ref.posLla = [[ggaVec.LatDeg]' [ggaVec.LonDeg]' [ggaVec.AltM]'];
            phones(iPhone).ref.trackDeg = [rmcVec.TrackDeg]';
            % Transform groundtruth's UTC time to GPS time
            [refGpsTime, phones(iPhone).ref.gpsSeconds, ~]= Utc2Gps(datevec([rmcVec.Datenum]));
            phones(iPhone).ref.wNum = refGpsTime(:, 1);
            phones(iPhone).ref.tow = refGpsTime(:, 2);
            % Obtain groundtruth's UTC time
            pp = csaps(phones(iPhone).gnss.obs(:, 2), phones(iPhone).gnss.utcSeconds);
            phones(iPhone).ref.utcSeconds = fnval(pp, phones(iPhone).ref.tow);
        end
    end
end

%% Navigation data
nav = rinex_v3_nav_parser(getNavFilepaths(config));

%% Ionospheric data
iono.alpha = [.4657E-08   .1490E-07  -.5960E-07  -.1192E-06]';
iono.beta = [.8192E+05   .9830E+05  -.6554E+05  -.5243E+06]';

%% OSR data
osrRnx.obs = [];
idxOsrSrc = 0;
% Use first OSR source with valid data
while isempty(osrRnx.obs) && idxOsrSrc < length(config.OSR_SOURCES)
    idxOsrSrc = idxOsrSrc + 1;
    osrFilepaths = getOSRFilepaths(config, config.OSR_SOURCES{idxOsrSrc});
    if isempty(osrFilepaths)
        osrRnx.type = [];
        osrRnx.statPos = [];
    else
        statPos = nan(3, 1);
        for iOsr = 1:length(osrFilepaths)
            [obs, type, auxPos] = rinex_v3_obs_parser(osrFilepaths{iOsr});
            osrFirstObsGPST = wntow2datetime(obs(1, 1), obs(1, 2));
            osrLastObsGPST = wntow2datetime(obs(end, 1), obs(end, 2));
            isValid = osrLastObsGPST > firstObsGPST && osrFirstObsGPST < lastObsGPST;
            if isValid
                if isempty(osrRnx.obs) % Display only first time
                    fprintf('Using OSR data from %s\n', config.OSR_SOURCES{idxOsrSrc});
                end
                osrRnx.obs = [osrRnx.obs; obs];
                assert(all(isnan(statPos)) || all(abs(statPos - auxPos) < 1), ...
                    'Station positions do not match between different OSR of the same campaign');
                statPos = auxPos;
            end
        end
        osrRnx.type = type;
        osrRnx.statPos = statPos;
    end
end
end