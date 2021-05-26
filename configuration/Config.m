classdef Config < handle
    % CONFIG This class is used to select the desired configuration
    % parameters and to generate the different filepaths and directories of
    % the selected dataset
    
    % This code assumes that the data is structured as follows:
    %   Observations:   {workspace_path}/data/training/datasets/{campaign_name}/{phone_name}_GnssLog.txt
    %   Groundtruth:    {workspace_path}/data/training/datasets/{campaign_name}/SPAN_{phone_name}_10Hz.nmea
    %   Navigation:     {workspace_path}/data/training/brdc/{campaign_name}/BRDC00WRD_R_{datetime}_01D_GN.rnx
    
    properties (Constant)
        %% Dataset selection
        CAMPAIGN_NAME           = '2020-08-06-US-MTV-2';
        PHONE_NAME              = 'Mi8';
        FILTER_RAW_MEAS         = true;
        NAV_FILE_DATETIME       = '20202190000'; % Date in broadcasted obs RINEX filename
        OSR_FILENAME            =  {'EAWD00XXX_R_20202192200_01H_01S_MO.rnx' 'EAWD00XXX_R_20202192300_01H_01S_MO.rnx'}; 
        % OBSERVATION RINEX - Uncomment to use, path from workspace
%         OBS_RINEX_PATH          = [Config.dataPath 'other' filesep 'igs_data' filesep ...
%                 'STFU00USA_S_20202190000_01D_01S_MO.crx' filesep 'STFU00USA_S_20202192215_15M_01S_MO.rnx'];
%         OBS_RINEX_REF_XYZ       = [-2700404.1800 -4292605.5200  3855137.4100];
        
        %% Operating mode
        OUTLIER_REJECTION       = true;
        
        %% RTK parameters
        MAX_OSR_INTERP_GAP_SEC  = 15;
        STATION_POS_XYZ         = [-2705252.9380 -4281214.6105  3864271.5517]';

        %% IMU parameters
        MAX_IMU_INTERP_GAP_SEC  = 0.02;
        
        %% Navigation parameters
        CONSTELLATIONS          = 'GE'
        OBS_COMBINATION         = {'none', 'none'};
        OBS_USED                = {'C1C', 'C1X+C5X'}; % PR Rinex code for observations
        OSR_OBS_USED            = {'C1C', 'C1X+C5X'}; % PR Rinex code for OSR data
        CONST_COV_FACTORS       = [1 1];            % Covariance factor for each constellation
        IONO_CORRECTION         = 'Klobuchar';      % among 'none' and 'Klobuchar'
        ELEVATION_MASK          = 10;
        MEAS_COV_SRC            = 'elevation';    % among 'elevation' and 'uncertainty'
        MAX_DOPPLER_MEAS        = 6e3;          % Maximum doppler measurement
        MAX_DOPPLER_UNCERT      = 10;
        
        %% KF tuning parameters
        % Process noise covariance matrix - Q
        SIGMA_Q_POS               = 1e10;         % std of XYZ position
%         SIGMA_VEL_NED           = [0.1 0.1 0.1]';  % std m/sqrt(s^3) of NED velocity
        % Measurement covariance matrix - R
        SIGMA_PR_M              = 1e1;          % Default std (m) for pseudorange meas (elevation-based model)
        SIGMA_DOP_MPS           = 1e0;          % Default std (m/s) for doppler meas (elevation-based model)
        COV_FACTOR_C            = 1e2;          % Covariance factor for pseudorange meas
        COV_FACTOR_D            = 1e4;          % Covariance factor for Doppler meas
        % State covariance matrix initialization - P0
        SIGMA_P0_VEL_XYZ        = [10 10 10];   % std m/sqrt(s^3) of initial velocity
        
    end
    
    %% Public methods
    methods (Static)
        function [dirPath, fileName] = getObsDirFile()
            %GETOBSDIRFILE Returns the directory and the filename of the
            %observation file according to the selected configuration.
            dirPath = [Config.trainPath 'datasets' filesep Config.CAMPAIGN_NAME filesep];
            fileName = [Config.PHONE_NAME '_GnssLog.txt'];
        end
        
        function filepaths = getNavFilepaths()
            %GETNAVFILEPATH Returns the file path(s) of the navigation
            %file(s) according to the CONSTELLATIONSs selected.
            %   Select the desired CONSTELLATIONSs and the date of the
            %   navigation file in the constant properties.
            filepaths = cell(1, length(Config.CONSTELLATIONS));
            for iConst = 1:length(Config.CONSTELLATIONS)
                filepaths{iConst} = [Config.trainPath 'brdc' filesep Config.CAMPAIGN_NAME ...
                    filesep 'BRDC00WRD_R_' Config.NAV_FILE_DATETIME '_01D_' ...
                    Config.CONSTELLATIONS(iConst) 'N.rnx'];
            end
        end
        
        function filepaths = getOSRFilepath()
            %GETOSRFILEPATH Returns the file path of the OSR file.
            rootPath = [Config.trainPath 'corrections' filesep 'OSR_v3.04' ...
                filesep Config.CAMPAIGN_NAME filesep];
            filepaths = cell(1, length(Config.OSR_FILENAME));
            for iOsr = 1:length(Config.OSR_FILENAME)
                if strcmp(Config.OSR_FILENAME{iOsr}(end-2:end), 'crx')
                    Config.crx2rnx([rootPath Config.OSR_FILENAME{iOsr}]);
                end
                filepaths{iOsr} = [rootPath Config.OSR_FILENAME{iOsr}(1:end-3) 'rnx'];
            end
        end
        
        function [dirPath, fileName] = getRefDirFile()
            %GETREFDIRFILE Returns the directory and the filename of the
            %groundtruth file according to the selected configuration.
            [dirPath, ~] = Config.getObsDirFile();
            fileName = ['SPAN_' Config.PHONE_NAME '_10Hz.nmea'];
        end
        
        function P0 = getP0()
            P0 = diag([ Config.SIGMA_P0_POS_NED ...
                Config.SIGMA_P0_VEL_NED ...
                Config.SIGMA_P0_CLK_BIAS       ...
                Config.SIGMA_P0_CLK_DRIFT      ...
                Config.SIGMA_P0_CLK_INTERFREQ	...
                Config.SIGMA_P0_CLK_INTERSYS]);
        end
        
    end
    
    %% Private methods
    methods (Static, Access = private)
        function path = trainPath()
            % TRAINPATH Returns the absolute path where all the training data is saved
            path = [workspacePath 'data' filesep 'training' filesep];
        end
        
        function path = dataPath()
            % DATAPATH Returns the absolute path where all the data is saved
            path = [workspacePath 'data' filesep];
        end
        
        function crx2rnx(filepath)
            cmd = [projectPath 'lib' filesep 'RNXCMP_4.0.8' filesep 'bin' filesep 'CRX2RNX ' filepath];
            system(cmd);
        end
    end
end

