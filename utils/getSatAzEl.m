function [satAzDeg, satElDeg, rxLLH] = getSatAzEl(satPos, rxPos)
    [latSat, lonSat, hSat] = ecef2geodetic(wgs84Ellipsoid, satPos(1,:), satPos(2,:), satPos(3,:));
    [latRx, lonRx, hRx] = ecef2geodetic(wgs84Ellipsoid, rxPos(1), rxPos(2), rxPos(3));
    [satAzDeg, satElDeg] = geodetic2aer(latSat, lonSat, hSat, latRx, lonRx, hRx, wgs84Ellipsoid);
    rxLLH = [latRx, lonRx, hRx];
end