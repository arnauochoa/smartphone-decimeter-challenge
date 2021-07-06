classdef Constants < handle
    %CONSTANTS Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Constant)
        % Physical
        CELERITY        = 299792458;        % m/s
        % Earth
        OMEGA_E         = 7.2921151467e-5;  % rad/s
        EARTH_RADIUS    = 6371e3;           % m
        % Time
        SECONDS_IN_DAY  = 86400;            % s
        DAYS_TO_UTC     = 719529;           % days from 1-1-0000 to 1-1-1970
        
        % Uncertainty thresholds
        MIN_C_SIGMA     = 1e-2;             % Minimum value for code pr uncertainty (m)
        MAX_C_SIGMA     = 1e5;              % Maximum value for code pr uncertainty (m)
        MIN_L_SIGMA     = 1e-2;              % Maximum value for carrier phase uncertainty (m)
        MAX_L_SIGMA     = 1e2;              % Maximum value for carrier phase uncertainty (m)
        MIN_D_SIGMA     = 1e-3;              % Maximum value for doppler uncertainty (mps)
        MAX_D_SIGMA     = 1e2;              % Maximum value for doppler uncertainty (mps)
        
        % GNSS
        GPS_L1_HZ       = 1575.42e6;      % Hz
    end
    
end

