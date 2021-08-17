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
        DATASET_TYPE            = 'train';
        CAMPAIGN_NAME           = '2020-08-06-US-MTV-2';
        PHONE_NAME              = 'Mi8';
        FILTER_RAW_MEAS         = true;
        NAV_FILE_DATETIME       = '20202190000'; % Date in broadcasted obs RINEX filename
        % OBSERVATION RINEX - Uncomment to use, path from workspace
%         OBS_RINEX_PATH          = 'data/other/igs_data/STFU00USA_S_20202190900_15M_01S_MO.rnx'; % 'data/other/igs_data/STFU00USA_S_20202190900_15M_01S_MO.rnx'; ||| 'data/other/ARWD219W.rnx';
%         OBS_RINEX_REF_XYZ       = [-2700404.1800 -4292605.5200  3855137.4100]; % [-2700404.1800 -4292605.5200  3855137.4100] ||| [-2687510.5240 -4290645.5230  3866179.1130]
        
        %% Operating mode
        OUTLIER_REJECTION       = true;

        %% IMU parameters
        MAX_IMU_INTERP_MILLIS   = 20;
        
        %% Navigation parameters
        CONSTELLATIONS          = 'GE'
        OBS_COMBINATION         = {'none', 'none'};
        OBS_USED                = {'C1C+C5X', 'C1X+C5X'}; % G: C1C+C5X, E: C1X+C5X, C: C2I
        CONST_COV_FACTORS       = [1 1];            % Covariance factor for each constellation
        IONO_CORRECTION         = 'Klobuchar';      % among 'none' and 'Klobuchar'
        ELEVATION_MASK          = 10;
        MEAS_COV_SRC            = 'elevation';    % among 'elevation' and 'uncertainty'
        MAX_DOPPLER_MEAS        = 6e3;          % Maximum doppler measurement
        MAX_DOPPLER_UNCERT      = 10;
        
        %% KF tuning parameters
        % Process noise covariance matrix - Q
        SIGMA_VEL_NED           = [90 90 50]';  % std m/sqrt(s^3) of NED velocity
        SIGMA_CLK_BIAS          = 12.8;         % std m/sqrt(s) of receiver clock bias
        SIGMA_CLK_DRIFT         = 0.4;          % std m/sqrt(s^3) of receiver clock drift
        SIGMA_CLK_INTERFREQ     = 0.1;         % std m/sqrt(s) of code inter-frequency clock bias
        SIGMA_CLK_INTERSYS      = 0.01;         % std m/sqrt(s) of inter-GNSS system clock bias
        % Measurement covariance matrix - R
        SIGMA_PR_M              = 1e1;          % Default std (m) for pseudorange meas (elevation-based model)
        SIGMA_DOP_MPS           = 1e0;          % Default std (m/s) for doppler meas (elevation-based model)
        COV_FACTOR_C            = 1e2;          % Covariance factor for pseudorange meas
        COV_FACTOR_D            = 1e4;          % Covariance factor for Doppler meas
        % State covariance matrix initialization - P0
        SIGMA_P0_VEL_XYZ        = [10 10 10];   % std m/sqrt(s^3) of initial velocity
        SIGMA_P0_CLK_DRIFT      = 100;          % std m/sqrt(s^3) of initial receiver clock drift
        SIGMA_P0_CLK_INTERFREQ  = 2e3;          % std m/sqrt(s) of initial code inter-frequency clock bias
        
    end
    
    %% Public methods
    methods (Static)
        function [dirPath, fileName] = getObsDirFile()
            %GETOBSDIRFILE Returns the directory and the filename of the
            %observation file according to the selected configuration.
            dirPath = [Config.dataPath Config.DATASET_TYPE filesep Config.CAMPAIGN_NAME filesep Config.PHONE_NAME filesep];
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
            dirPath = [dirPath 'supplemental' filesep];
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
        function path = dataPath()
            % DATAPATH Returns the absolute path where all the data is saved
            path = [workspacePath 'data' filesep 'sdc-data' filesep];
        end
        
    end
end

