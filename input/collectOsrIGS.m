function [dataLocalFileNames] = collectOsrIGS(utcTimeVec, targetFilepath)

yearStr = num2str(utcTimeVec(1));
dayNumberStr = num2str(DayOfYear(utcTimeVec), '%03d');

ftpFilepath = 'IGS/highrate/';
dataFilename = strcat('STFU00USA_S_', yearStr, dayNumberStr, '0000_01D_01S_MO.crx');
gzFilename = strcat(dataFilename, '.tar.gz');

finalTargetFilepath = targetFilepath;
targetFilepath = strcat(targetFilepath, dataFilename, '/');

ftpFilepath = strcat(ftpFilepath, yearStr, '/', dayNumberStr, '/');

tarLocalFilepath = strcat(targetFilepath, gzFilename);

if ~exist(targetFilepath, 'dir')
    disp('Connecting to igs.bkg.bund.de...');
    ftpobj = ftp('igs.bkg.bund.de');
    cd(ftpobj, ftpFilepath);
    mget(ftpobj, gzFilename, targetFilepath);
    close(ftpobj);
    untar(tarLocalFilepath, targetFilepath);
    delete(tarLocalFilepath);
end
dataLocalFileNames = convertCrxOsr(targetFilepath, finalTargetFilepath);
end