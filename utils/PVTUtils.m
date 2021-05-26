classdef PVTUtils < handle
    %PVTUTILS Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Constant)
        ID_POS = 1;
        ID_VEL = 2;
        ID_CLK_BIAS = 3;
        ID_CLK_DRIFT = 4;
        ID_INTER_FREQ_BIAS = 5;
        ID_INTER_SYS_BIAS = 6;
        
        % Map from RINEX code to frequency
        FREQUENCIES_MAP = containers.Map(...
            {'C1C' 'C1X' 'C2I' 'C5X' 'C2X'}, ...                  % Code (RINEX Format)
            [1575.42e6 1575.42e6 1561.098e6 1176.45e6 1227.6e6]);    % Frequency (Hz)
        
    end
    
    methods (Static)
        
        function nStates = getNumStates()
            % GETNUMSTATES Returns the number of states in the state vector
            % 3D pos, 3D vel
            nStates = 3; % TODO check
        end
        
        function idx = getStateIndex(unknownID)
            %GETSTATEINDEX Returns the index in the state vector of the
            %given unknown.
            switch unknownID
                case PVTUtils.ID_POS
                    idx = 1:3;
%                 case PVTUtils.ID_VEL
%                     prevIdx = PVTUtils.getStateIndex(PVTUtils.ID_POS);
%                     idx = prevIdx(end) + (1:3);
                otherwise
                    error('Invalid unknown ID')
            end
        end
        
        function nConst = getNumConstellations()
            nConst = length(Config.CONSTELLATIONS);
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
                    nSatsConstellation = Constants.MAX_GPS_PRN;
                case 'E'
                    nSatsConstellation = Constants.MAX_GAL_PRN;
                case 'C'
                    nSatsConstellation = Constants.MAX_BDS_PRN;
                otherwise
                    error('Constellation %c is not supported.', Config.CONSTELLATIONS(iConst));
            end
        end
    end
end

