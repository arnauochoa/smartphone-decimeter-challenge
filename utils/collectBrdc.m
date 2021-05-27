function [dataFilepath] = collectBrdc(year, month, day, const)

filepath = [workspacePath 'data' filesep Config.DATASET_TYPE ...
    filesep 'brdc' filesep Config.CAMPAIGN_NAME filesep];

number_days = datenum(strcat(string(month), '/',string (day),'/', string (year))) - datenum(strcat('01-jan-',string (year))) +1;
if number_days <10
    number_days = strcat('00',string(number_days));
else
    if number_days < 100
        number_days = strcat('0',string(number_days));
    else
        number_days = string(number_days);
    end
end

dataFilepath = strcat(filepath, 'BRDC00WRD_R_', string(year), number_days, '0000_01D_', const, 'N.rnx');
gzFilepath = [dataFilepath '.gz'];

if ~exist(dataFilepath, 'file')
    ftpobj = ftp('igs.bkg.bund.de');
    cd(ftpobj,filepath);
    mget(ftpobj,gzFilepath);
    close(ftpobj);
    gunzip(gzFilepath);
    delete(gzFilepath);
end
end