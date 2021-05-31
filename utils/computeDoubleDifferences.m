function [doubleDifferences] = computeDoubleDifferences(osrGnss, phoneGnss, satPos, satElDeg)
% COMPUTEDOUBLEDIFFERENCES Computes the double differences considering
% different constellations and frequencies

if nargin<1
    selfTest();
    return;
end

constels = unique([phoneGnss.obs.constellation], 'stable');

doubleDifferences = [];
nDiscarded = 0;

for iConst = 1:length(constels)
    isConstRx = [phoneGnss.obs.constellation] == constels(iConst);
    isConstOsr = [osrGnss.obs.constellation] == constels(iConst);
    
    freqs = unique([phoneGnss.obs(isConstRx).D_fcarrier_Hz], 'stable');
    for iFreq = 1:length(freqs)
        isConstFreqRx = isConstRx & [phoneGnss.obs.D_fcarrier_Hz] == freqs(iFreq);
        isConstFreqOsr = isConstOsr & [osrGnss.obs.D_fcarrier_Hz] == freqs(iFreq);
        
        [dd, nDis] = getDD(phoneGnss.obs(isConstFreqRx),... % Receiver pos
            osrGnss.obs(isConstFreqOsr),                ... % Station pos
            satPos(:, isConstFreqRx),                   ... % Sat pos associated to rx obs
            satElDeg(isConstFreqRx));                       % Sat elev associated to rx obs
        
        nDiscarded = nDiscarded + nDis;
        doubleDifferences = appendStruct(doubleDifferences, dd, 2);
    end
end
% Convert from structure of arrays to array of structures
doubleDifferences = soa2aos(doubleDifferences, 2);
if isempty(fieldnames(doubleDifferences))
    doubleDifferences = [];
end
if nDiscarded > 0 && ~strcmp(Config.EVALUATE_DATASETS, 'all')
    fprintf('>> TOW = %d, %d observations haven''t been found in OSR.\n', phoneGnss.tow, nDiscarded);
end
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
    dd.C = (codeSd(idxPivSat) - codeSd(idxVarSats));
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

function selfTest()

obs.constellation = 'GGGGGEEEC';
obs.D_fcarrier_Hz = [1575.42e6 1575.42e6 1575.42e6 1176.45e6 1176.45e6 1575.42e6 1575.42e6 1176.45e6 1561.098e6];
obs.prn = [1 2 3 1 2 3 5 3 2];
obs.C = [10 20 15 10 19 5 9 6 25];
obs.L = [10 20 15 10 19 5 9 6 25];
obs.C_sigma = ones(size(obs.C));
obs.D_sigma = ones(size(obs.C));
obs.L_sigma = ones(size(obs.C));
gnssOsr.obs = soa2aos(obs);

clear obs
obs.constellation = 'GGGGEECC';
obs.D_fcarrier_Hz = [1575.42e6 1575.42e6 1176.45e6 1176.45e6 1575.42e6 1176.45e6 1561.098e6 1561.098e6];
obs.prn = [2 3 1 2 3 3 2 5];
obs.C = [20 14 10 18 5 6 25 1]+0.5;
obs.L = [20 14 10 18 5 6 25 1]+3.5;
obs.C_sigma = ones(size(obs.C));
obs.D_sigma = ones(size(obs.C));
obs.L_sigma = ones(size(obs.C));
gnssRx.obs = soa2aos(obs);
gnssRx.tow = 0;


satPos = repmat(obs.prn, 3, 1);
satElDeg = [90 45 20 90 50 50 70 25];

[doubleDifferences] = computeDoubleDifferences(gnssOsr, gnssRx, satPos, satElDeg, [0 0 0]);
assert(length(doubleDifferences) == 2, 'Test failed');
%
assert(doubleDifferences(1).C == -1, 'Test failed');
assert(doubleDifferences(1).L == -1, 'Test failed');
assert(doubleDifferences(1).constel == 'G', 'Test failed');
assert(doubleDifferences(1).freqHz == 1575.42e6, 'Test failed');
assert(doubleDifferences(1).pivSatPrn == 2, 'Test failed');
assert(all(doubleDifferences(1).pivSatPos == [2;2;2]), 'Test failed');
assert(all(doubleDifferences(1).pivSatElDeg == 90), 'Test failed');
assert(doubleDifferences(1).varSatPrn == 3, 'Test failed');
assert(all(doubleDifferences(1).varSatPos == [3;3;3]), 'Test failed');
assert(all(doubleDifferences(1).varSatElDeg == 45), 'Test failed');
%
assert(doubleDifferences(2).C == 1, 'Test failed');
assert(doubleDifferences(2).L == 1, 'Test failed');
assert(doubleDifferences(2).constel == 'G', 'Test failed');
assert(doubleDifferences(2).freqHz == 1176.45e6, 'Test failed');
assert(doubleDifferences(2).pivSatPrn == 2, 'Test failed');
assert(all(doubleDifferences(2).pivSatPos == [2;2;2]), 'Test failed');
assert(all(doubleDifferences(2).pivSatElDeg == 90), 'Test failed');
assert(doubleDifferences(2).varSatPrn == 1, 'Test failed');
assert(all(doubleDifferences(2).varSatPos == [1;1;1]), 'Test failed');
assert(all(doubleDifferences(2).varSatElDeg == 20), 'Test failed');

disp([mfilename '>> Function self test(s) passed']);
end