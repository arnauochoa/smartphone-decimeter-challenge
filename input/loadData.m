function [gnss, imuRaw, nav, ref] = loadData()
% LOADDATA Loads the GNSS observations, IMU measurements, navigation data
% and groundtruth data

%% GNSS observations and IMU measurements
[obsDirPath, obsFileName] = Config.getObsDirFile();
[obsRinex, obsRinexType, obsRinexUtcMillis, ~, accMeas, gyrMeas, magMeas, ~] = ...
    getGnssLogObs(obsDirPath, obsFileName, Config.FILTER_RAW_MEAS);

gnss.obs = obsRinex;
gnss.type = obsRinexType;
gnss.utcMillis = obsRinexUtcMillis;

imuRaw.acc = accMeas;
imuRaw.gyr = gyrMeas;
imuRaw.mag = magMeas;

%% Navigation data
nav = [];

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
[ref.gpsTime, ~, ~] = Utc2Gps(datevec([rmcVec.Datenum]));

end