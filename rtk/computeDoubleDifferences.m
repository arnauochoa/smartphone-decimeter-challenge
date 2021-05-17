function [doubleDifferences] = computeDoubleDifferences(gnssRx, gnssOsr)


constels = unique([gnssRx.obs.constellation], 'stable');

doubleDifferences.C = [];
doubleDifferences.L = [];
nDiscarded = 0;

for iConst = 1:length(constels)
    isConstRx = [gnssRx.obs.constellation] == constels(iConst);
    isConstOsr = [gnssOsr.obs.constellation] == constels(iConst);
    
    freqs = unique([gnssRx.obs(isConstRx).D_fcarrier_Hz], 'stable');
    for iFreq = 1:length(freqs)
        isConstFreqGnss = isConstRx & [gnssRx.obs.D_fcarrier_Hz] == freqs(iFreq);
        isConstFreqOsr = isConstOsr & [gnssOsr.obs.D_fcarrier_Hz] == freqs(iFreq);
        
        [dd, nDis] = getDD(gnssRx.obs(isConstFreqGnss), gnssOsr.obs(isConstFreqOsr));
        doubleDifferences.C = [doubleDifferences.C; dd.C];
        doubleDifferences.L = [doubleDifferences.L; dd.L];
        nDiscarded = nDiscarded + nDis;
    end
end
% Convert from structure of arrays to array of structures
doubleDifferences = soa2aos(doubleDifferences);
fprintf('>> TOW = %d, %d observations haven''t been found in OSR.\n', gnssRx.tow, nDiscarded);
end

function [dd, nDiscard] = getDD(rxObs, osrObs)

% Common satellites between receiver and OSR station
[commonSats, iRxObs, iOsrObs] = intersect([rxObs.prn], [osrObs.prn]);

% Num of observations not found in OSR data
nDiscard = length(rxObs) - length(commonSats);

if commonSats > 1
    % Keep only common satellites
    rxObs = rxObs(iRxObs);
    osrObs = osrObs(iOsrObs);
    
    % Single differences for each satellite
    codeSd = [osrObs(:).C]' - [rxObs(:).C]';
    phaseSd = [osrObs(:).L]' - [rxObs(:).L]';
    
    % Double differences
    dd.C = codeSd(1) - codeSd(2:end);
    dd.L = phaseSd(1) - phaseSd(2:end);
else
    dd.C = [];
    dd.L = [];
end
end