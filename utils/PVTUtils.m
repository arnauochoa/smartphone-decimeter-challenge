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
        
        FREQUENCIES_MAP = containers.Map(...
            {'C1C' 'C1X' 'C2I' 'C5X'}, ...                  % Code (RINEX Format)
            [1575.42e6 1575.42e6 1561.098e6 1176.45e6]);    % Frequency (Hz)
        
    end
    
    methods (Static)
        
        function nStates = getNumStates()
            % GETNUMSTATES Returns the number of states in the state vector
            % 3D pos, 3D vel, clock bias, clock drift, inter-freq bias,
            % inter-system bias
            nStates = 8 + PVTUtils.getNumFrequencies-1 + PVTUtils.getNumConstellations-1;
        end
        
        function idx = getStateIndex(unknown)
            %GETSTATEINDEX Returns the index in the state vector of the
            %given unknown.
            switch unknown
                case PVTUtils.ID_POS
                    idx = 1:3;
                case PVTUtils.ID_VEL
                    prevIdx = PVTUtils.getStateIndex(PVTUtils.ID_POS);
                    idx = prevIdx(end) + (1:3);
                case PVTUtils.ID_CLK_BIAS
                    prevIdx = PVTUtils.getStateIndex(PVTUtils.ID_VEL);
                    idx = prevIdx(end) + 1;
                case PVTUtils.ID_CLK_DRIFT
                    prevIdx = PVTUtils.getStateIndex(PVTUtils.ID_CLK_BIAS);
                    idx = prevIdx(end) + 1;
                case PVTUtils.ID_INTER_FREQ_BIAS
                    prevIdx = PVTUtils.getStateIndex(PVTUtils.ID_CLK_DRIFT);
                    idx = prevIdx(end) + (1:PVTUtils.getNumFrequencies-1);
                case PVTUtils.ID_INTER_SYS_BIAS
                    prevIdx = PVTUtils.getStateIndex(PVTUtils.ID_INTER_FREQ_BIAS);
                    if isempty(prevIdx)
                        prevIdx = PVTUtils.getStateIndex(PVTUtils.ID_CLK_DRIFT);
                    end
                    idx = prevIdx(end) + (1:PVTUtils.getNumConstellations-1);
                otherwise
                    error('Invalid unknown')
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
        
        
    end
end

