classdef (Sealed) Config < handle
    % CONFIG This class is used to select the desired configuration
    % parameters and to generate the different filepaths and directories of
    % the selected dataset
    
    % This code assumes that the data is structured as follows:
    %   Observations:   {workspace_path}/data/train/datasets/{campaign_name}/{phone_name}_GnssLog.txt
    %   Groundtruth:    {workspace_path}/data/train/datasets/{campaign_name}/SPAN_{phone_name}_10Hz.nmea
    %   Navigation:     {workspace_path}/data/train/brdc/{campaign_name}/BRDC00WRD_R_{datetime}_01D_GN.rnx
    
    properties (Constant)
        %% Results
        RES_FILENAME            = 'result';
        
        %% Dataset selection
        EVALUATE_DATASETS       = 'single';                                 % 'single' 'all'
        DATASET_TYPE            = 'train';                                  % 'train' 'test'
        CAMPAIGN_NAME           = '2020-06-11-US-MTV-1';                    % Only if EVALUATE_DATASETS = single
        PHONE_NAME              = 'Pixel4';                                 % Only if EVALUATE_DATASETS = single
        FILTER_RAW_MEAS         = true;                                     % Enable/disable filtering of raw measurements (omited when caching)
        OSR_SOURCES             = {'Verizon', 'SwiftNav', 'IGS'}            % By order of preference
        OSR_STATION_NAME        = 'EAWD';                                   % Verizon station name
        
        % OBSERVATION RINEX - Uncomment to use, path from workspace
%         OBS_RINEX_PATH          = [workspacePath 'data' filesep 'other' ...
%             filesep 'igs_data' filesep 'STFU00USA_S_20202190000_01D_01S_MO.crx' filesep 'STFU00USA_S_20202192215_15M_01S_MO.rnx'];
%         OBS_RINEX_REF_XYZ       = [-2700404.1800 -4292605.5200  3855137.4100];
        
        %% Operating mode
        OUTLIER_REJECTION       = true;
        
        %% RTK parameters
        MAX_OSR_INTERP_GAP_SEC  = 15;

        %% IMU parameters
        MAX_IMU_INTERP_GAP_SEC  = 0.02;
        
        %% Navigation parameters
        CONSTELLATIONS          = 'GEC'
        OBS_COMBINATION         = {'none'};
        OBS_USED                = {'C1C+C5X', 'C1X+C5X', 'C2X'};            % PR Rinex code for observations
        OSR_OBS_USED            = {'C1C+C5I', 'C1B+C5I', 'C2X'};            % PR Rinex code for OSR data
        CONST_COV_FACTORS       = [1 1 2];                                  % Covariance factor for each constellation
        ELEVATION_MASK          = 10;                                       % Elevation mask in degrees
        MEAS_COV_SRC            = 'uncertainty';                            % Among 'elevation' and 'uncertainty'
%         MAX_DOPPLER_MEAS        = 6e3;                                      % Maximum doppler measurement 
%         MAX_DOPPLER_UNCERT      = 10;                                       % Maximum doppler uncertainty
        
        %% KF tuning parameters
        % Process noise covariance matrix - Q
        SIGMA_Q_VEL_XYZ         = [1e2 1e2 1e2];    % std m/sqrt(s^3) of XYZ velocity
        SIGMA_Q_CLK_DRIFT       = 0.5;              % std m/sqrt(s^3) of clock drift
        % Measurement covariance matrix - R
        SIGMA_PR_M              = 1e1;              % Default std (m) for pseudorange meas (elevation-based model)
        SIGMA_DOP_MPS           = 1e0;              % Default std (m/s) for doppler meas (elevation-based model)
        COV_FACTOR_C            = 1e2;              % Covariance factor for pseudorange meas
        COV_FACTOR_D            = 1e4;              % Covariance factor for Doppler meas
        % State covariance matrix initialization - P0
        SIGMA_P0_VEL_XYZ        = [10 10 10];          % std m/sqrt(s^3) of initial XYZ velocity
        SIGMA_P0_CLK_DRIFT      = 100;               % std m/sqrt(s^3) of initial clock drift
        
    end
    
    properties
        campaignName = Config.CAMPAIGN_NAME;
        phoneName = Config.PHONE_NAME;
        resFileTimestamp = '';
    end
    
    %% Private constructor
    methods (Access = private)
      function obj = Config()
          obj.resFileTimestamp = datestr(datetime('now'), 'yyyymmdd_HHMMSS');
      end
    end
    
    %% Public methods 
    methods (Static)
        % Singleton instantiator        
        function singleObj = getInstance()
            persistent localObj
             if isempty(localObj) || ~isvalid(localObj)
                localObj = Config;
             end
             singleObj = localObj;
        end
    end
    
    methods        
        function [dirPath, fileName] = getObsDirFile(this)
            %GETOBSDIRFILE Returns the directory and the filename of the
            %observation file according to the selected configuration.
            dirPath = [this.obsDataPath this.campaignName filesep this.phoneName filesep];
            fileName = [this.phoneName '_GnssLog.txt'];
        end
        
        function filepaths = getNavFilepaths(this)
            %GETNAVFILEPATH Returns the file path(s) of the navigation
            %file(s) according to the CONSTELLATIONSs selected.
            %   Select the desired CONSTELLATIONSs and the date of the
            %   navigation file in the constant properties.
            filepaths = cell(1, length(this.CONSTELLATIONS));
            utcTimeVec = datevec(this.campaignName(1:10), 'yyyy-mm-dd');
            targetFilepath = [Config.dataPath 'brdc' filesep this.campaignName filesep];
            for iConst = 1:length(this.CONSTELLATIONS)
                filepaths{iConst} = collectBrdc(utcTimeVec, ...
                    this.CONSTELLATIONS(iConst), targetFilepath);
            end
        end
        
        function filepaths = getOSRFilepaths(this, osrSource)
            %GETOSRFILEPATH Returns the file path of the OSR file.
            switch osrSource
                case 'Verizon'
                    rootPath = [Config.dataPath 'corrections' filesep osrSource ...
                        filesep 'OSR' filesep this.campaignName filesep];
                    filesInPath = getValidDir(rootPath);
                    idxStation = contains(filesInPath, this.OSR_STATION_NAME);
                    fileNames = filesInPath(idxStation);
                case 'SwiftNav'
                    rootPath = [Config.dataPath 'corrections' filesep osrSource ...
                        filesep 'OSR' filesep];
                    osrFileNames = getValidDir(rootPath);
                    campaignDateStr1 = this.campaignName(1:10);
                    campaignDateStr2 = erase(campaignDateStr1, '-');
                    idxFiles = contains(osrFileNames, '.obs') & ...
                        (contains(osrFileNames, campaignDateStr1) | ...
                        contains(osrFileNames, campaignDateStr2));
                    fileNames = osrFileNames(idxFiles); % TODO: test and extract from switch
                case 'IGS'
                    rootPath = [Config.dataPath 'corrections' filesep osrSource ...
                        filesep 'OSR' filesep this.campaignName(1:10) filesep];
                    if ~exist(rootPath, 'dir'), mkdir(rootPath); end
                    utcTimeVec = datevec(this.campaignName(1:10), 'yyyy-mm-dd');
                    fileNames = collectOsrIGS(utcTimeVec, rootPath);
                otherwise
                    error('Invalid Config.OSR_SOURCES');
            end
            filepaths = cell(1, length(fileNames));
            for iOsr = 1:length(fileNames)
                filepaths{iOsr} = [rootPath fileNames{iOsr}];
            end
        end
        
        function [dirPath, fileName] = getRefDirFile(this)
            %GETREFDIRFILE Returns the directory and the filename of the
            %groundtruth file according to the selected configuration.
            [dirPath, ~] = this.getObsDirFile();
            dirPath = [dirPath 'supplemental' filesep];
            fileName = ['SPAN_' this.phoneName '_10Hz.nmea'];
        end
        
        function path = obsDataPath(this)
            % TRAINPATH Returns the absolute path where the obs data is saved
            path = [Config.dataPath this.DATASET_TYPE filesep];
        end
        
    end
    
    %% Private methods
    methods (Static, Access = private)
        function path = dataPath()
            % TRAINPATH Returns the absolute path where the obs data is saved
            path = [workspacePath 'data' filesep 'sdc-data' filesep];
        end
        
        function crx2rnx(filepath)
            cmd = [projectPath 'lib' filesep 'RNXCMP_4.0.8' filesep 'bin' filesep 'CRX2RNX ' filepath];
            system(cmd);
        end
    end
end

