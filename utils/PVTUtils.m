classdef PVTUtils < handle
    %PVTUTILS Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Constant)
        ID_POS = 1;
        ID_VEL = 2;
        ID_CLK_DRIFT = 3;
        ID_SD_AMBIGUITY = 4;
        
        % Data structures
        MAX_GPS_PRN = 100;
        MAX_GAL_PRN = 100;
        MAX_BDS_PRN = 100;
        
        % Map from RINEX code to frequency
        FREQUENCIES_MAP = containers.Map(...
            {'C1C' 'C1X' 'C2I' 'C5X' 'C2X'}, ...                  % Code (RINEX Format)
            [1575.42e6 1575.42e6 1561.098e6 1176.45e6 1227.6e6]);    % Frequency (Hz)
        
    end
    
    %% Public methods
    methods (Static)
        function nStates = getNumStates()
            % GETNUMSTATES Returns the number of states in the state vector
            % 3D pos, 3D vel, clk drift, SD ambiguities
            nStates = 7 + PVTUtils.getNumSatelliteIndices();
        end
        
        function idx = getStateIndex(stateID, prn, constellationLetter)
            %GETSTATEINDEX Returns the index in the state vector of the
            %given unknown.
            %   idx = GETSTATEINDEX(stateID, [prn], [constellationLetter])
            % For stateID = ID_SD_AMBIGUITY:
            %   - If no prn and constellation are given, all SD ambiguity
            %   indices are returned.
            %   - If prn and constellation are given, the index for the SD
            %   ambiguity of the requested satellite is provided
            
            switch stateID
                case PVTUtils.ID_POS
                    idx = 1:3;
                case PVTUtils.ID_VEL
                    prevIdx = PVTUtils.getStateIndex(PVTUtils.ID_POS);
                    idx = prevIdx(end) + (1:3);
                case PVTUtils.ID_CLK_DRIFT
                    prevIdx = PVTUtils.getStateIndex(PVTUtils.ID_VEL);
                    idx = prevIdx(end) + 1;
                case PVTUtils.ID_SD_AMBIGUITY
                    prevIdx = PVTUtils.getStateIndex(PVTUtils.ID_CLK_DRIFT);
                    if nargin == 1
                        idx = prevIdx(end) + (1:PVTUtils.getNumSatelliteIndices);
                    elseif nargin < 3
                        error('Both PRN and CONST must be provided.');
                    else
                        idx = prevIdx(end) + PVTUtils.getSatelliteIndex(prn, constellationLetter);
                    end
                otherwise
                    error('Invalid state ID')
            end
        end
        
        function nConst = getNumConstellations()
            config = Config.getInstance;
            nConst = length(config.CONSTELLATIONS);
        end
        
        function freqIdx = getConstelIdx(constellationLetter)
            freqIdx = strfind(Config.CONSTELLATIONS, constellationLetter);
        end
        
        function nFreq = getNumFrequencies()
            nFreq = length(PVTUtils.getUniqueFreqsHz);
        end
        
        function freqIdx = getFreqIdx(freqHz)
            freqIdx = find(PVTUtils.getUniqueFreqsHz == freqHz);
        end
        
        function numSatellites = getNumSatelliteIndices()
            numSatellites = 0;
            for iConst = 1:length(Config.CONSTELLATIONS)
                numSatellites = numSatellites + ...
                    PVTUtils.getTotalNumSatsConstellation(Config.CONSTELLATIONS(iConst));
            end
        end
        
        function satIdx = getSatelliteIndex(prn, constellationLetter)
            prevConstLetters = Config.CONSTELLATIONS(1:strfind(Config.CONSTELLATIONS, constellationLetter)-1);
            satIdx = 0;
            for iConst = 1:length(prevConstLetters)
                satIdx = satIdx + PVTUtils.getTotalNumSatsConstellation(prevConstLetters(iConst));
            end
            satIdx = satIdx + prn;
            assert(satIdx > 0 && satIdx < PVTUtils.getNumSatelliteIndices, 'Invalid prn.');
        end
        
    end
    
    %% Private methods
    methods (Static, Access = private)
        function freqsHz = getUniqueFreqsHz()
            frequencies = [];
            for iConst = 1:length(Config.OBS_USED)
                obs = split(Config.OBS_USED{iConst}, '+')';
                freqValsCells = values(PVTUtils.FREQUENCIES_MAP, obs);
                frequencies = [frequencies [freqValsCells{:}]];
            end
            freqsHz = unique(frequencies, 'stable');
        end
        
        function nSatsConstellation = getTotalNumSatsConstellation(constellationLetter)
            switch constellationLetter
                case 'G'
                    nSatsConstellation = PVTUtils.MAX_GPS_PRN;
                case 'E'
                    nSatsConstellation = PVTUtils.MAX_GAL_PRN;
                case 'C'
                    nSatsConstellation = PVTUtils.MAX_BDS_PRN;
                otherwise
                    error('Constellation %c is not supported.', Config.CONSTELLATIONS(iConst));
            end
        end
    end
end

