function phones = findGeometry(phones)
% FINDGEOMETRY Computes the geometry of the smartphones in the lever arm
%   phones = FINDGEOMETRY(phones)
%
% Input:
%   phones  =   Structure array. Contains GNSS and INS measurements and 
%               groundtruth
% Output:
%   phones  =   Structure array. Contains GNSS and INS measurements,
%               groundtruth and geometry in body frame

% TODO: find actual geometry for each case (from groundtruth, table, etc)
phones(1).posBody   =   [0 0 0]';
phones(2).posBody   =   [-0.1 0 0]';
end