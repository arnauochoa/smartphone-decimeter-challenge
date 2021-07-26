function [x, y] = cleanRepeatedSamplePts(x, y)
% CLEANREPEATEDSAMPLEPTS Removes the repeated values in x and sets the
% corresponding y as the average
%   [x, y] = CLEANREPEATEDSAMPLEPTS(x, y)
%
% Input:
%   x       = Vector. Independent variable
%   y       = Vector or matrix. Dependent variable
% Output:
%   x       = Vector. Independent variable
%   y       = Vector or matrix. Dependent variable
%

if nargin < 1, selfTest; return; end

[~, dim] = max(size(x));
nPts = length(x);
assert(size(y, dim) == nPts, 'x and y should have the same length')

[~, ix, ~] = unique(x);

% Find indices of non-unique x
idxRep = setdiff(1:nPts, ix);

seenX = [];
% Set y as average of occurrences
for iRep = 1:length(idxRep)
    if any(seenX == x(idxRep(iRep)))
        continue % If an x value has already been seen, go to next iteration
    else
        % Find occurrences of non-unique x
        idxOccur = find(x == x(idxRep(iRep)));
        switch dim
            case 1
                y(idxOccur(1), :) = mean(y(idxOccur, :), dim, 'omitnan');
            case 2
                y(:, idxOccur(1)) = mean(y(:, idxOccur), dim, 'omitnan');
            case 3
                y(:, :, idxOccur(1)) = mean(y(:, :, idxOccur), dim, 'omitnan');
            otherwise
                error('Handling of more than 3 array dimensions not implemented');
        end
        seenX = [seenX x(idxRep(iRep))];
    end
end

% Remove repeated x vals and their respective y vals
x(idxRep) = [];
switch dim
    case 1
        y(idxRep, :) = [];
    case 2
        y(:, idxRep) = [];
    case 3
        y(:, :, idxRep) = [];
    otherwise
        error('Handling of more than 3 array dimensions not implemented');
end
end

function selfTest()
passed = true;
% Test 1
x = [1 2 2 3 4]; y = [1 2 3 4];
try % should fail
    cleanRepeatedSamplePts(x, y);
    passed = false;
catch
end
assert(passed, 'test failed');

% Test 2
x = [1 2 2 3 4 3 5 5 5 5]; y = [1 2 3 4 5 6 7 8 8 8];
[x, y] = cleanRepeatedSamplePts(x, y);
assert(all(x == [1 2 3 4 5]) && all(y == [1 2.5 5 5 7.75]), 'test failed');

% Test 3
x = [1 2 2 3 2]; y = [1 2 3 5 4; 1 2 2 3 2; 1 1 1 1 1];
[x, y] = cleanRepeatedSamplePts(x, y);
assert(all(x == [1 2 3]) && all(y == [1 3 5; 1 2 3; 1 1 1], 'all'), 'test failed');

disp('test passed');
end