function [doubleDifferences] = computeDoubleDifferences(gnssOsr, gnssRx, satPos, satElDeg)
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
        isConstFreqRx = isConstRx & [gnssRx.obs.D_fcarrier_Hz] == freqs(iFreq);
        isConstFreqOsr = isConstOsr & [gnssOsr.obs.D_fcarrier_Hz] == freqs(iFreq);
        
        [dd, nDis] = getDD(gnssRx.obs(isConstFreqRx),   ... % Receiver pos
            gnssOsr.obs(isConstFreqOsr),                ... % Station pos
            satPos(:, isConstFreqRx),                   ... % Sat pos associated to rx obs
            satElDeg(isConstFreqRx));                       % Sat elev associated to rx obs
        
        nDiscarded = nDiscarded + nDis;
        doubleDifferences = appendStruct(doubleDifferences, dd, 2);
    end
end
% Convert from structure of arrays to array of structures
doubleDifferences = soa2aos(doubleDifferences);
fprintf('>> TOW = %d, %d observations haven''t been found in OSR.\n', gnssRx.tow, nDiscarded);
end

function [dd, nDiscard] = getDD(rxObs, osrObs, satPos, satElDeg)
% GETDD Computes the DD of the two given sets of observations

% Common satellites between receiver and OSR station
[commonSats, iRxObs, iOsrObs] = intersect([rxObs.prn], [osrObs.prn]);
numCommonSats = length(commonSats);

% Num of observations not found in OSR data
nDiscard = length(rxObs) - length(commonSats);

if numCommonSats > 1
    % Keep only common satellites
    rxObs = rxObs(iRxObs);
    satPos = satPos(:, iRxObs);
    satElDeg = satElDeg(iRxObs);
    osrObs = osrObs(iOsrObs);
    
    % Single differences for each satellite
    codeSd = [osrObs(:).C] - [rxObs(:).C];
    phaseSd = [osrObs(:).L] - [rxObs(:).L];
%     dopSd = [osrObs(:).D_Hz] - [rxObs(:).D_Hz];
    
    % Find indices of pivot and varying satellites
    [idxPivSat, idxVarSats] = choosePivotSat(satElDeg);
    
    % Double differences
    dd.C = codeSd(idxPivSat) - codeSd(idxVarSats);
    dd.L = phaseSd(idxPivSat) - phaseSd(idxVarSats);
%     dd.DHz = dopSd(idxPivSat) - dopSd(idxVarSats);
    nDD = length(dd.C);
    dd.constel = repmat(rxObs(idxPivSat).constellation, 1, nDD);
    dd.freqHz = repmat(rxObs(idxPivSat).D_fcarrier_Hz, 1, nDD);
    % Pivot satellite dat
    dd.pivSatPrn = repmat(rxObs(idxPivSat).prn, 1, nDD); 
    dd.pivSatPos = repmat(satPos(:, idxPivSat), 1, nDD);
    dd.pivSatElDeg = repmat(satElDeg(idxPivSat), 1, nDD);
    dd.pivSatSigmaC = repmat(rxObs(idxPivSat).C_sigma, 1, nDD);
    dd.pivSatSigmaL = repmat(rxObs(idxPivSat).L_sigma, 1, nDD);
    dd.pivSatSigmaD = repmat(rxObs(idxPivSat).D_sigma, 1, nDD);
    % Varying satellite data
    dd.varSatPrn = [rxObs(idxVarSats).prn];
    dd.varSatPos = satPos(:, idxVarSats);
    dd.varSatElDeg = satElDeg(idxVarSats);
    dd.varSatSigmaC = [rxObs(idxVarSats).C_sigma];
    dd.varSatSigmaL = [rxObs(idxVarSats).L_sigma];
    dd.varSatSigmaD = [rxObs(idxVarSats).D_sigma];
else
    dd.C = [];
    dd.L = [];
    dd.constel = [];
    dd.freqHz = [];
    % Pivot satellite dat
    dd.pivSatPrn = [];
    dd.pivSatPos = [];
    dd.pivSatElDeg = [];
    dd.pivSatSigmaC = [];
    dd.pivSatSigmaL = [];
    dd.pivSatSigmaD = [];
    % Varying satellite data
    dd.varSatPrn = [];
    dd.varSatPos = [];
    dd.varSatElDeg = [];
    dd.varSatSigmaC = [];
    dd.varSatSigmaL = [];
    dd.varSatSigmaD = [];
end
end

function [idxPivSat, idxVarSats] = choosePivotSat(satElDeg)
    satIndices = 1:length(satElDeg);
    [~, idxPivSat] = max(satElDeg);
    idxVarSats = satIndices(satIndices ~= idxPivSat); % All except pivot sat
end