function pGeodetic = ecef2geodeticVector(pEcef)
% ECEF2GEODETICVECTOR converts a row vector (or matrix) representing ECEF 
% position to Geodetic.

assert(size(pEcef, 2) == 3, 'Input should be a three column vector or matrix')
[pLat, pLon, pAlt] = ecef2geodetic(wgs84Ellipsoid, pEcef(:, 1), pEcef(:, 2), pEcef(:, 3));
pGeodetic = [pLat, pLon, pAlt];
end