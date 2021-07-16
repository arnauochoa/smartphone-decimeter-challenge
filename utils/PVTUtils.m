classdef PVTUtils < handle
    %PVTUTILS Utilities class for the PVT engine
    %   This class contains functions related to the state vector,
    %   constellations, satellites, etc.
    
    properties (Constant)
        % State-vector id's
        ID_POS = 1;
        ID_VEL = 2;
        ID_CLK_DRIFT = 3;
        ID_SD_AMBIGUITY = 4;
        
        % Data structures
        MAX_GPS_PRN = 40;
        MAX_GAL_PRN = 50;
        MAX_BDS_PRN = 70;
        
        % Map from RINEX code to frequency
        FREQUENCIES_MAP = containers.Map(...
            {'C1C' 'C1X' 'C5X' 'C2I' 'C2X'}, ...                        % Code (RINEX Format)
            [1575.42e6 1575.42e6 1176.45e6 1561.098e6 1561.098e6]);     % Frequency (Hz)
    end
    
    %% Public methods
    methods (Static)
        function nStates = getNumStates()
            % GETNUMSTATES Returns the number of states in the state vector
            % 3D pos, 3D vel, clk drift, SD ambiguities
            nStates = 6;
            if Config.USE_DOPPLER % clk drift
                nStates = nStates + 1;
            end
            if Config.USE_PHASE_DD % SD ambiguities
                nStates = nStates + PVTUtils.getNumSatFreqIndices();
            end
        end
        
        function idx = getStateIndex(stateID, prn, constellationLetter, freqHz)
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
                    if Config.USE_DOPPLER % clk drift
                        prevIdx = PVTUtils.getStateIndex(PVTUtils.ID_VEL);
                        idx = prevIdx(end) + 1;
                    else
                        idx = [];
                    end
                case PVTUtils.ID_SD_AMBIGUITY
                    if Config.USE_PHASE_DD % SD ambiguities
                        if Config.USE_DOPPLER % clk drift
                            prevIdx = PVTUtils.getStateIndex(PVTUtils.ID_CLK_DRIFT);
                        else
                            prevIdx = PVTUtils.getStateIndex(PVTUtils.ID_VEL);
                        end
                        if nargin == 1
                            idx = prevIdx(end) + (1:PVTUtils.getNumSatFreqIndices);
                        elseif nargin < 3
                            error('Both PRN and CONST must be provided.');
                        else
                            idx = prevIdx(end) + ...
                                PVTUtils.getSatFreqIndex(prn, constellationLetter, freqHz);
                        end
                    else
                        idx = [];
                    end
                otherwise
                    error('Invalid state ID')
            end
        end
        
        function nConst = getNumConstellations()
            % GETNUMCONSTELLATIONS Returns the total number of 
            % constellations selected in the configuration
            config = Config.getInstance;
            nConst = length(config.CONSTELLATIONS);
        end
        
        function constIdx = getConstelIdx(constellationLetter)
            % GETCONSTELIDX Returns the index of the constellation provided
            constIdx = strfind(Config.CONSTELLATIONS, constellationLetter);
        end
        
        function nFreq = getNumFrequencies()
            % GETNUMFREQUENCIES Returns the total number of different
            % frequencies selected in the configuration
            nFreq = length(PVTUtils.getUniqueFreqsHz);
        end
        
        function freqIdx = getFreqIdx(freqHz)
            % GETFREQIDX Returns the index of the frequency provided
            freqIdx = find(PVTUtils.getUniqueFreqsHz == freqHz);
        end
        
        function numSatellites = getNumSatelliteIndices()
            % GETNUMSATELLITEINDICES Returns the total number of unique
            % satellite indices
            numSatellites = 0;
            for iConst = 1:length(Config.CONSTELLATIONS)
                numSatellites = numSatellites + ...
                    PVTUtils.getTotalNumSatsConstellation(Config.CONSTELLATIONS(iConst));
            end
        end
        
        function satIdx = getSatelliteIndex(prn, constellationLetter)
            % GETSATELLITEINDEX Returns the index of the selected satellite
            % given by its prn and constellation
            prevConstLetters = Config.CONSTELLATIONS(1:strfind(Config.CONSTELLATIONS, constellationLetter)-1);
            satIdx = 0;
            for iConst = 1:length(prevConstLetters)
                satIdx = satIdx + PVTUtils.getTotalNumSatsConstellation(prevConstLetters(iConst));
            end
            satIdx = satIdx + prn;
            assert(satIdx > 0 && satIdx < PVTUtils.getNumSatelliteIndices, 'Invalid prn.');
        end
        
        function numSatellites = getNumSatFreqIndices()
            % GETNUMSATFREQINDICES Returns the total number of indices for
            % unique satellite+frequency combinations
            numSatellites = 0;
            for iConst = 1:length(Config.CONSTELLATIONS)
                nFreq = PVTUtils.getNumFreqsConst(Config.CONSTELLATIONS(iConst));
                numSatellites = numSatellites + ...
                    nFreq * PVTUtils.getTotalNumSatsConstellation(Config.CONSTELLATIONS(iConst));
            end
        end
        
        function satIdx = getSatFreqIndex(prn, constellationLetter, freqHz)
            % GETSATFREQINDEX Returns the index of the selected pair
            % satellite+frequency given by its prn and constellation
            nSatsConst = PVTUtils.getTotalNumSatsConstellation(constellationLetter);
            assert(prn > 0 && prn <= nSatsConst, 'Invalid prn.');
            prevConstLetters = Config.CONSTELLATIONS(1:strfind(Config.CONSTELLATIONS, constellationLetter)-1);
            satIdx = 0;
            for iConst = 1:length(prevConstLetters)
                nFreqs = PVTUtils.getNumFreqsConst(prevConstLetters(iConst));
                satIdx = satIdx + nFreqs*PVTUtils.getTotalNumSatsConstellation(prevConstLetters(iConst));
            end
            freqIdx = PVTUtils.getFreqIdxInConst(constellationLetter, freqHz);
            satIdx = satIdx + (freqIdx-1)*nSatsConst + prn;
        end
        
        function nFreqs = getNumFreqsConst(constellationLetter)
            % GETNUMFREQSCONST Returns the number of frequencies selected
            % for the required constellations
            nFreqs = nan(size(constellationLetter));
            for iLetter = 1:length(constellationLetter)
                idxConst = strfind(Config.CONSTELLATIONS, constellationLetter(iLetter));
                nFreqs(iLetter) = length(split(Config.OBS_USED{idxConst}, '+'));
            end
        end
        
        function freqIdx = getFreqIdxInConst(constellationLetter, freqHz)
            % GETFREQIDXINCONST Returns the index of the provided frequency
            % among all the frequencies of the provided constellation
            idxConst = strfind(Config.CONSTELLATIONS, constellationLetter);
            codes = split(Config.OBS_USED{idxConst}, '+');
            freqs = nan(size(codes));
            for iCode = 1:length(codes)
                freqs(iCode) = PVTUtils.FREQUENCIES_MAP(codes{iCode});
            end
            freqIdx = find(freqs == freqHz);
            assert(~isempty(freqIdx), 'Invalid combination of constellation and frequency');
        end
    end
    
    %% Private methods
    methods (Static, Access = private)
        function freqsHz = getUniqueFreqsHz()
            % GETUNIQUEFREQSHZ Returns a vector with the unique frequencies
            % from the codes and constellations selected in the
            % configuration
            frequencies = [];
            for iConst = 1:length(Config.OBS_USED)
                obs = split(Config.OBS_USED{iConst}, '+')';
                freqValsCells = values(PVTUtils.FREQUENCIES_MAP, obs);
                frequencies = [frequencies [freqValsCells{:}]];
            end
            freqsHz = unique(frequencies, 'stable');
        end
        
        function nSatsConstellation = getTotalNumSatsConstellation(constellationLetter)
            % GETTOTALNUMSATSCONSTELLATION Returns the maximum number of 
            % satellites for the given constellation
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

