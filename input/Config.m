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
        RES_FILENAME            = 'sample_result';
        
        %% Dataset selection
        EVALUATE_DATASETS       = 'single';     % 'single' 'all'
        DATASET_TYPE            = 'train';      % 'train' 'test'
        CAMPAIGN_NAME           = '2020-05-14-US-MTV-1';    % only if EVALUATE_DATASETS = single
        PHONE_NAME              = 'Pixel4';                    % only if EVALUATE_DATASETS = single
        FILTER_RAW_MEAS         = true;
%         NAV_FILE_DATETIME       = '20202190000'; % Date in broadcasted obs RINEX filename
        OSR_STATION_NAME        =  'EAWD'; 
        % OBSERVATION RINEX - Uncomment to use, path from workspace
%         OBS_RINEX_PATH          = [workspacePath 'data' filesep 'other' ...
%             filesep 'igs_data' filesep 'STFU00USA_S_20202190000_01D_01S_MO.crx' filesep 'STFU00USA_S_20202192215_15M_01S_MO.rnx'];
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
        OBS_COMBINATION         = {'none'};
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
            dirPath = [this.dataPath 'datasets' filesep this.campaignName filesep this.phoneName filesep];
            fileName = [this.phoneName '_GnssLog.txt'];
        end
        
        function filepaths = getNavFilepaths(this)
            %GETNAVFILEPATH Returns the file path(s) of the navigation
            %file(s) according to the CONSTELLATIONSs selected.
            %   Select the desired CONSTELLATIONSs and the date of the
            %   navigation file in the constant properties.
            filepaths = cell(1, length(this.CONSTELLATIONS));
            utcTimeVec = datevec(this.campaignName(1:10), 'yyyy-mm-dd');
            for iConst = 1:length(this.CONSTELLATIONS)
%                 filepaths{iConst} = [this.dataPath 'brdc' filesep this.campaignName ...
%                     filesep 'BRDC00WRD_R_' this.NAV_FILE_DATETIME '_01D_' ...
%                     this.CONSTELLATIONS(iConst) 'N.rnx'];
                filepaths{iConst} = collectBrdc(utcTimeVec, this.CONSTELLATIONS(iConst));
            end
        end
        
        function filepaths = getOSRFilepaths(this)
            %GETOSRFILEPATH Returns the file path of the OSR file.
            rootPath = [this.dataPath 'corrections' filesep 'OSR_v3.04' ...
                filesep this.campaignName filesep];
            filesInPath = dir(rootPath);
            idxStation = contains({filesInPath.name}, this.OSR_STATION_NAME);
            fileNames = {filesInPath(idxStation).name};
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
        
        function path = dataPath(this)
            % TRAINPATH Returns the absolute path where all the training data is saved
            path = [workspacePath 'data' filesep this.DATASET_TYPE filesep];
        end
        
%         function P0 = getP0(this)
%             P0 = diag([ this.SIGMA_P0_POS_NED ...
%                 this.SIGMA_P0_VEL_NED ...
%                 this.SIGMA_P0_CLK_BIAS       ...
%                 this.SIGMA_P0_CLK_DRIFT      ...
%                 this.SIGMA_P0_CLK_INTERFREQ	...
%                 this.SIGMA_P0_CLK_INTERSYS]);
%         end
        
    end
    
    %% Private methods
    methods (Static, Access = private)
        
        function crx2rnx(filepath)
            cmd = [projectPath 'lib' filesep 'RNXCMP_4.0.8' filesep 'bin' filesep 'CRX2RNX ' filepath];
            system(cmd);
        end
    end
end

