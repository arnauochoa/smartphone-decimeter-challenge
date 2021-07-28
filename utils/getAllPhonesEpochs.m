function epochTimesUtc = getAllPhonesEpochs(phones)
% GETALLPHONESEPOCHS returns the unique times of the epochs from all phones
%   epochTimes = GETALLPHONESEPOCHS(phones)
% 
% Input: 
%       phones      = Array of structs. Contains the measurements of each phone
% Output: 
%       epochTimes  = Array of floats. UTC timestamps of the epochs of all phones

epochTimesUtc = [];
for iPhone = 1:length(phones)
    epochTimesUtc = [epochTimesUtc; phones(iPhone).gnss.utcSeconds];
end
epochTimesUtc = unique(epochTimesUtc);
end