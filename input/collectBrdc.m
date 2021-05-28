function [dataLocalFilepath] = collectBrdc(utcTimeVec, const)

config = Config.getInstance;

year = utcTimeVec(1);

targetFilepath = [workspacePath 'data' filesep config.DATASET_TYPE ...
    filesep 'brdc' filesep config.campaignName filesep];

ftpFilepath = strcat('IGS/BRDC/',string(year), '/');

dayNumber = string(DayOfYear(utcTimeVec));

ftpFilepath = strcat(ftpFilepath, dayNumber, '/');

dataFilename = strcat('BRDC00WRD_R_', string(year), dayNumber, '0000_01D_', const, 'N.rnx');
dataLocalFilepath = strcat(targetFilepath, dataFilename);
gzFilename = strcat(dataFilename, '.gz');
gzLocalFilepath = strcat(targetFilepath, gzFilename);

if ~exist(dataLocalFilepath, 'file')
    ftpobj = ftp('igs.bkg.bund.de');
    cd(ftpobj, ftpFilepath);
    mget(ftpobj, gzFilename, targetFilepath);
    close(ftpobj);
    gunzip(gzLocalFilepath, targetFilepath);
    delete(gzLocalFilepath);
end
end