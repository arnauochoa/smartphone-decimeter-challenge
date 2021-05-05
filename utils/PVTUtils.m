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
        
    end
    
    methods (Static)
        
        function nStates = getNStates()
            % GETNSTATES Returns the number of states in the state vector
            % 3D pos, 3D vel, clock bias, clock drift, inter-freq bias,
            % inter-system bias
            nStates = 8 + Config.getNumFreq-1 + Config.getNumConst-1;
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
                    idx = prevIdx(end) + (1:Config.getNumFreq-1);
                case PVTUtils.ID_INTER_SYS_BIAS
                    prevIdx = PVTUtils.getStateIndex(PVTUtils.ID_INTER_FREQ_BIAS);
                    if isempty(prevIdx)
                        prevIdx = PVTUtils.getStateIndex(PVTUtils.ID_CLK_DRIFT);
                    end
                    idx = prevIdx(end) + (1:Config.getNumConst-1);
                otherwise
                    error('Invalid unknown')
            end
        end
        
    end
end

