classdef (Sealed) Config < handle
    % CONFIG This class is used to select the desired configuration
    % parameters and to generate the different filepaths and directories of
    % the selected dataset
    
    % This code assumes that the data is structured as follows:
    %   Observations:
    %       {workspace_path}/data/sdc-data/{train or test}/{campaign_name}/{phone_name}/{phone_name}_GnssLog.txt
    %   Groundtruth:
    %       {workspace_path}/data/sdc-data/{train or test}/{campaign_name}/{phone_name}/supplemental/SPAN_{phone_name}_10Hz.nmea
    %   Navigation:
    %       {workspace_path}/data/sdc-data/brdc/{campaign_name}/BRDC00WRD_R_{datetime}_01D_GN.rnx
    %   OSR:
    %       {workspace_path}/data/sdc-data/corrections/{source_name}/OSR/[see getOSRFilepaths]
    properties (Constant)
        %% Debug options
        SHOW_DEBUG_MESSAGES     = false;
        EPOCHS_TO_RUN           = inf;                                      % Set as inf to run all epochs
        
        %% Results
        RES_FILENAME            = 'result';
        
        %% Trace selection
        EVALUATE_DATASETS       = 'all';                                 % 'single' 'all'
        CAMPAIGN_NAME           = '2021-04-15-US-MTV-1';%'2020-06-11-US-MTV-1';%'2021-04-15-US-MTV-1';                    % Only if EVALUATE_DATASETS = single
        PHONE_NAME              = 'Pixel4';                                    % Only if EVALUATE_DATASETS = single
        FILTER_RAW_MEAS         = true;                                     % Enable/disable filtering of raw measurements (omited when caching)
        OSR_SOURCES             = {'Verizon', 'SwiftNav', 'IGS'};           % By order of preference
        OSR_STATION_NAME        = 'EAWD';                                   % Verizon station name
        
        % OBSERVATION RINEX - Uncomment to use, path from workspace
        %         OBS_RINEX_PATH          = [workspacePath 'data' filesep 'other' ...
        %             filesep 'igs_data' filesep 'STFU00USA_S_20202190000_01D_01S_MO.crx' filesep 'STFU00USA_S_20202192215_15M_01S_MO.rnx'];
        %         OBS_RINEX_REF_XYZ       = [-2700404.1800 -4292605.5200  3855137.4100];
        
        %% Operating mode
        MULTI_RX                = true;                                     % If true, all phones from a campaign are used
        P_FALSE_OUTLIER_REJECT  = 0.01;                                     % Probability of false outlier rejection
        
        %% RTK parameters
        USE_REF_POS             = false;
        USE_CODE_DD             = true;
        USE_PHASE_DD            = true;
        USE_DOPPLER             = true;
        MAX_OSR_INTERP_GAP_SEC  = 15;
        
        %% IMU parameters
        MAX_IMU_INTERP_GAP_SEC  = 0.02;
        
        %% Navigation parameters
        CONSTELLATIONS          = 'GEC';
        %         OBS_COMBINATION         = {'none', 'none'};
        OBS_USED                = {'C1C+C5X', 'C1X+C5X', 'C2X'};            % PR Rinex code for observations
        OSR_OBS_USED            = {'C1C+C5I', 'C1X+C5X', 'C2X'};            % PR Rinex code for OSR data
        CONST_COV_FACTORS       = [1 1 2];                                  % Covariance factor for each constellation
        ELEVATION_MASK          = 10;                                       % Elevation mask in degrees
        MEAS_COV_SRC            = 'uncertainty';                            % Among 'elevation' and 'uncertainty'
%         MAX_DOPPLER_MEAS        = 6e3;                                      % Maximum doppler measurement
%         MAX_DOPPLER_UNCERT      = 10;                                       % Maximum doppler uncertainty
        
        %% KF tuning parameters
        X0_ATT_XYZ              = [0 0 0];          % (rad) initial XYZ attitude angles
        % Process noise covariance matrix - Q
        SIGMA_Q_ATT_XYZ         = [0 pi/72 pi/18];  % std rad/sqrt(s) of XYZ attitude angles
        SIGMA_Q_VEL_XYZ         = [1e2 1e2 1e2];    % std m/sqrt(s^3) of XYZ velocity
        SIGMA_Q_CLK_DRIFT       = 1e1;              % std m/sqrt(s^3) of clock drift
        SIGMA_Q_SD_AMBIG        = 1e-2;             % std cyc of SD phase ambiguity
        % Measurement covariance matrix - R
        SIGMA_C_M               = 1e2;              % Default std (m) for pseudorange meas          (only for elevation-based model)
        SIGMA_L_M               = 1e0;              % Default std (m) for carrier phase meas        ("")
        SIGMA_D_MPS             = 1e-1;             % Default std (m/s) for doppler meas            ("")
        COV_FACTOR_C            = 1e1;              % Covariance factor for code pseudorange meas   (useful for weighting uncertainties coming from GnssLog)
        COV_FACTOR_L            = 1e2;              % Covariance factor for carrier phase meas      ("")
        COV_FACTOR_D            = 1e5;              % Covariance factor for Doppler meas            ("")
        % State covariance matrix initialization - P0
        FACTOR_P0_POS           = 1e5;              % Factor that multiplies P0 obtained from WLS
        SIGMA_P0_ATT_XYZ        = [0 pi/36 pi];     % std rad/sqrt(s) of XYZ attitude angles
        SIGMA_P0_VEL_XYZ        = [1e2 1e2 1e2];    % std m/sqrt(s^3) of initial XYZ velocity
        SIGMA_P0_CLK_DRIFT      = 1e2;              % std m/sqrt(s^3) of initial clock drift
        SIGMA_P0_SD_AMBIG       = 1e5;              % std cyc of initial SD phase ambiguity
    end
    
    properties
        DATASET_TYPE            = 'test';                                  % 'train' 'test'
        campaignName = Config.CAMPAIGN_NAME;
        phoneNames;
        resFileTimestamp;
    end
    
    %% Private constructor
    methods (Access = private)
        function obj = Config()
            obj.resFileTimestamp = datestr(datetime('now'), 'yyyymmdd_HHMMSS');
            if obj.MULTI_RX
                obj.phoneNames = getPhoneNamesInCampaign(obj);
            else
                obj.phoneNames = {Config.PHONE_NAME};
            end
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
        function [dirPaths, fileNames] = getObsDirFile(this)
            %GETOBSDIRFILE Returns the directory and the filename of the
            %observation file according to the selected configuration.
            dirPaths = cell(size(this.phoneNames));
            fileNames = cell(size(this.phoneNames));
            for iPhone = 1:length(this.phoneNames)
                dirPaths{iPhone} = [this.obsDataPath this.campaignName filesep this.phoneNames{iPhone} filesep];
                fileNames{iPhone} = [this.phoneNames{iPhone} '_GnssLog.txt'];
            end
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
                case 'Verizon'  % Structure: [...]/Verizon/OSR/{campaign_name}/{filename}.rnx
                    rootPath = [Config.dataPath 'corrections' filesep osrSource ...
                        filesep 'OSR' filesep this.campaignName filesep];
                    filesInPath = getValidDir(rootPath);
                    idxStation = contains(filesInPath, this.OSR_STATION_NAME);
                    fileNames = filesInPath(idxStation);
                case 'SwiftNav' % Structure: [...]/SwiftNav/OSR/{campaign_name or startdate-enddate}.obs
                    rootPath = [Config.dataPath 'corrections' filesep osrSource ...
                        filesep 'OSR' filesep];
                    osrFileNames = getValidDir(rootPath);
                    campaignDateStr1 = this.campaignName(1:10);
                    campaignDateStr2 = erase(campaignDateStr1, '-');
                    idxFiles = contains(osrFileNames, '.obs') & ...
                        (contains(osrFileNames, campaignDateStr1) | ...
                        contains(osrFileNames, campaignDateStr2));
                    fileNames = osrFileNames(idxFiles);
                case 'IGS'      % Structure: [...]/Verizon/IGS/{date}/{filename}.rnx
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
        
        function [dirPaths, fileNames] = getRefDirFile(this)
            %GETREFDIRFILE Returns the directory and the filename of the
            %groundtruth file according to the selected configuration.
            [dirPaths, ~] = this.getObsDirFile();
            fileNames = cell(size(this.phoneNames));
            for iPhone = 1:length(this.phoneNames)
                dirPaths{iPhone} = [dirPaths{iPhone} 'supplemental' filesep];
                fileNames{iPhone} = ['SPAN_' this.phoneNames{iPhone} '_10Hz.nmea'];
            end
        end
        
        function path = obsDataPath(this)
            % TRAINPATH Returns the absolute path where the obs data is saved
            path = [Config.dataPath this.DATASET_TYPE filesep];
        end
        
        function resultsDir = getResultsDir(this)
            resultsDir = strcat(workspacePath, 'data', filesep, 'results', filesep, this.DATASET_TYPE, filesep);
            switch this.EVALUATE_DATASETS
                case 'single'
                    resultsDir = strcat(resultsDir, this.campaignName, filesep);
                case 'all'
                    resultsDir = strcat(resultsDir, 'all', filesep);
                otherwise
                    error('Invalid field for Config.EVALUATE_DATASETS, choose among ''single'' and ''all''');
            end
        end
        
        function phoneNames = getPhoneNamesInCampaign(this)
            campaignPath = [this.obsDataPath this.campaignName filesep];
            phoneNames = getValidDir(campaignPath);
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

