function [doubleDifferences] = computeDoubleDifferences(gnssRx, gnssOsr)
% COMPUTEDOUBLEDIFFERENCES Computes the double differences considering
% different constellations and frequencies

constels = unique([gnssRx.obs.constellation], 'stable');

doubleDifferences = [];
% doubleDifferences.L = [];
nDiscarded = 0;

for iConst = 1:length(constels)
    isConstRx = [gnssRx.obs.constellation] == constels(iConst);
    isConstOsr = [gnssOsr.obs.constellation] == constels(iConst);
    
    freqs = unique([gnssRx.obs(isConstRx).D_fcarrier_Hz], 'stable');
    for iFreq = 1:length(freqs)
        isConstFreqGnss = isConstRx & [gnssRx.obs.D_fcarrier_Hz] == freqs(iFreq);
        isConstFreqOsr = isConstOsr & [gnssOsr.obs.D_fcarrier_Hz] == freqs(iFreq);
        
        [dd, nDis] = getDD(gnssRx.obs(isConstFreqGnss), gnssOsr.obs(isConstFreqOsr));
        nDiscarded = nDiscarded + nDis;
        % Convert to array of structures and concatenate if not empty
        ddaos = soa2aos(dd);
        if ~isempty(fieldnames(ddaos))
            doubleDifferences = [doubleDifferences soa2aos(dd)];
        end
    end
end
% Convert from structure of arrays to array of structures
% doubleDifferences = soa2aos(doubleDifferences);
fprintf('>> TOW = %d, %d observations haven''t been found in OSR.\n', gnssRx.tow, nDiscarded);
end

function [dd, nDiscard] = getDD(rxObs, osrObs)
% GETDD Computes the DD of the two given sets of observations

% Common satellites between receiver and OSR station
[commonSats, iRxObs, iOsrObs] = intersect([rxObs.prn], [osrObs.prn]);

% Num of observations not found in OSR data
nDiscard = length(rxObs) - length(commonSats);

if length(commonSats) > 1
    % Keep only common satellites
    rxObs = rxObs(iRxObs);
    osrObs = osrObs(iOsrObs);
    
    % Single differences for each satellite
    codeSd = [osrObs(:).C]' - [rxObs(:).C]';
    phaseSd = [osrObs(:).L]' - [rxObs(:).L]';
%     dopSd = [osrObs(:).D_Hz]' - [rxObs(:).D_Hz]';
    
    % Double differences
    dd.C = codeSd(1) - codeSd(2:end);
    dd.L = phaseSd(1) - phaseSd(2:end);
%     dd.DHz = dopSd(1) - dopSd(2:end);
    nDD = length(dd.C);
    dd.constel = repmat(rxObs(1).constellation, nDD, 1);
    dd.freqHz = repmat(rxObs(1).D_fcarrier_Hz, nDD, 1);
    dd.prns = [repmat(rxObs(1).prn, nDD, 1) [rxObs(2:end).prn]'];
    dd.sigmaC = [repmat(rxObs(1).C_sigma, nDD, 1) [rxObs(2:end).C_sigma]'];
    dd.sigmaL = [repmat(rxObs(1).L_sigma, nDD, 1) [rxObs(2:end).L_sigma]'];
%     dd.sigmaD = [repmat(rxObs(1).D_sigma, nDD, 1) [rxObs(2:end).D_sigma]'];
else
    dd.C = [];
    dd.L = [];
%     dd.DHz = [];
    dd.constel = [];
    dd.freqHz = [];
    dd.prns = [];
    dd.sigmaC = [];
    dd.sigmaL = [];
%     dd.sigmaD = [];
end
end