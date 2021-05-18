function [satPosOut] = getSatPositions(gnssObs, satPos, constel, prns)
%GETSATPOSITIONS Returns the satellite positions of given prns

satPosOut = nan(3, length(prns));

for iPrn = 1:length(prns)
    isSat = [gnssObs.constellation] == constel & [gnssObs.prn] == prns(iPrn);
    idxSat = find(isSat, 1, 'first');
    satPosOut(:, iPrn) = satPos(:, idxSat);
end

end

