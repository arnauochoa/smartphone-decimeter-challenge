function [vq] = interpMaxDist(x, v, xq, maxDist, method)
% INTERPMAXDIST 1-D interpolation with maximum distance between query and
% sample points
%   
%   [vq] = INTERPMAXDIST(x, v, xq, maxDist, method) interpolates to find
%   the values vq at the query points xq. If the distance between a query
%   point and the previous/next sample point is larger than maxDist, the
%   output value for this query point will be NaN.
%
% Input:
%   x       = vector with sample points
%   v       = vector with sample values
%   xq      = vector with query points
%   maxDist = maximum distance between query point and nearest sample
%               points (above and below)
%   method  = method for interpolation
%
% Output:
%   vq      = vector with values at query points
%
% See also INTERP1.

% Author: Arnau Ochoa Banuelos (CS Group), April 2021
    
% Matrix of distances between data and query points
distMat = x-xq';

% Find minimum distances of query points below sample points
distMatBelow = distMat; 
distMatBelow(distMat < 0) = nan; % remove distances above
[minDistBelow, ~] = min(distMatBelow, [], 1);

% Find minimum distances of query points above sample points
distMatAbove = distMat; 
distMatAbove(distMat > 0) = nan; % remove distances below
[minDistAbove, ~] = min(-distMatAbove, [], 1);

xq(minDistBelow > maxDist) = nan; % Distance to next sample pt too large
xq(minDistAbove > maxDist) = nan; % Distance to prev sample pt too large
xq(isnan(minDistBelow)) = nan;    % No sample pt below
xq(isnan(minDistAbove)) = nan;    % No sample pt above

vq = interp1(x, v, xq, method);

end