function pEcef = geodetic2ecefVector(pGeodetic)
% GEODETIC2ECEFVECTOR converts a row vector (or matrix) representing ECEF 
% position to Geodetic.

assert(size(pGeodetic, 2) == 3, 'Input should be a three column vector or matrix')
[pX, pY, pZ] = geodetic2ecef(wgs84Ellipsoid, pGeodetic(:, 1), pGeodetic(:, 2), pGeodetic(:, 3));
pEcef = [pX, pY, pZ];
end