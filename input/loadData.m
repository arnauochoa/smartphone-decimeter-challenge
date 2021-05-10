function [gnssRnx, imuRaw, nav, iono, ref] = loadData()
% LOADDATA Loads the GNSS observations, IMU measurements, navigation data
% and groundtruth data

%% GNSS observations and IMU measurements
[obsDirPath, obsFileName] = Config.getObsDirFile();
[obsRinex, obsRinexType, obsRinexUtcMillis, ~, accMeas, gyrMeas, magMeas, ~] = ...
    getGnssLogObs(obsDirPath, obsFileName, Config.FILTER_RAW_MEAS);

gnssRnx.obs = obsRinex;
gnssRnx.type = obsRinexType;
gnssRnx.utcSeconds = obsRinexUtcMillis / 1e3;

imuRaw.acc = accMeas;
imuRaw.gyr = gyrMeas;
imuRaw.mag = magMeas;

%% Navigation data
nav = rinex_v3_nav_parser(Config.getNavFilepaths());

%% Ionospheric data
iono.alpha = [4.6566E-09  1.4901E-08 -5.9605E-08 -5.9605E-08]';
iono.beta = [7.7824E+04  4.9152E+04 -6.5536E+04 -3.2768E+05]';

%% Groundtruth data
disp('Reading groundtruth file...');
[refDirPath, refFileName] = Config.getRefDirFile();
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
ref.gpsTime = Utc2Gps(datevec([rmcVec.Datenum]));
% Obtain groundtruth's UTC time
pp = csaps(gnssRnx.obs(:, 2), gnssRnx.utcSeconds);
ref.utcSeconds = fnval(pp, ref.gpsTime(:, 2));

end