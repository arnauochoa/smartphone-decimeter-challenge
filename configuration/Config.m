classdef Config < handle
    % CONFIG This class is used to select the desired configuration
    % parameters and to generate the different filepaths and directories of
    % the selected dataset
    
    % This code assumes that the data is structured as follows:
    %   Observations:   {workspace_path}/data/training/datasets/{campaign_name}/{phone_name}_GnssLog.txt
    %   Groundtruth:    {workspace_path}/data/training/datasets/{campaign_name}/SPAN_{phone_name}_10Hz.nmea
    %   Navigation:     {workspace_path}/data/training/brdc/{campaign_name}/BRDC00WRD_R_{datetime}_01D_GN.rnx
    
    properties (Constant)
        % Dataset selection
        CAMPAIGN_NAME =  '2020-09-04-US-SF-1';
        PHONE_NAME = 'Pixel4';
        FILTER_RAW_MEAS = true;
        NAV_FILE_DATETIME = '20201990000'; % Date in broadcasted obs RINEX filename
        
        % IMU parameters
        MAX_IMU_INTERP_MILLIS = 20;
        
        % Navigation parameters
        CONSTELLATIONS = 'GE'
        OBS_COMBINATION = {'none','none'};
        OBS_USED = {'C1C+C5X','C1C+C5X'};
        IONO_CORRECTION = 'Klobuchar'; % among 'none', 'iono-free' and 'Klobuchar'
        ELEVATION_MASK = 10;
        MEAS_COVARIANCE = 'uncertainty'; % among 'elevation' and 'uncertainty'
    end
    
    %% Public methods
    methods (Static)        
        function [dirPath, fileName] = getObsDirFile()
            %GETOBSDIRFILE Returns the directory and the filename of the
            %observation file according to the selected configuration.
            dirPath = [Config.dataPath 'datasets' filesep Config.CAMPAIGN_NAME filesep];
            fileName = [Config.PHONE_NAME '_GnssLog.txt'];
        end

        function filepaths = getNavFilepaths()
            %GETNAVFILEPATH Returns the file path(s) of the navigation
            %file(s) according to the CONSTELLATIONSs selected.
            %   Select the desired CONSTELLATIONSs and the date of the
            %   navigation file in the constant properties.
            filepaths = cell(1, length(Config.CONSTELLATIONS));
            for iConst = 1:length(Config.CONSTELLATIONS)
                filepaths{iConst} = [Config.dataPath 'brdc' filesep Config.CAMPAIGN_NAME ...
                    filesep 'BRDC00WRD_R_' Config.NAV_FILE_DATETIME '_01D_' ...
                    Config.CONSTELLATIONS(iConst) 'N.rnx'];
            end
        end
        
        function [dirPath, fileName] = getRefDirFile()
            %GETREFDIRFILE Returns the directory and the filename of the
            %groundtruth file according to the selected configuration.
            [dirPath, ~] = Config.getObsDirFile();
            fileName = ['SPAN_' Config.PHONE_NAME '_10Hz.nmea'];
        end
    end
    
    %% Private methods
    methods (Static, Access = private)
        function path = dataPath()
            % DATAPATH Returns the absolute path where all the data is saved
            path = [workspacePath 'data' filesep 'training' filesep];
        end
    end
end

