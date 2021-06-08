classdef Constants < handle
    %CONSTANTS Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Constant)
        % Physical
        CELERITY = 299792458;       % m/s
        % Earth
        OMEGA_E = 7.2921151467e-5;  % rad/s
        EARTH_RADIUS = 6371e3;      % m
        % Time
        SECONDS_IN_DAY = 86400;     % s
        DAYS_TO_UTC = 719529;       % days from 1-1-0000 to 1-1-1970
        
        % GNSS
        GPS_L1_HZ = 1575.42e6;      % Hz
        
        % Error indicators
        NO_ERR      = 0;
        ERR_NO_OSR  = 1;
    end
    
end

