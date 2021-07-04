function [B] = reassignUnique(A)
% REASSIGNUNIQUE Generates a new matrix B where the N unique values of A
% are reassigned one-to-one to the values 1 to N
% Example:
%   A = [1 3 4; 3 6 4]  --> B = [1 2 3; 2 4 3]
if nargin < 1
    selfTest()
    return
end
unqA = unique(A);
B = nan(size(A));
for row = 1:size(A, 1)
    for col = 1:size(A, 2)
        B(row, col) = find(A(row, col) == unqA);
    end
end
end


function selfTest()
A = [1 3; 2 3; 5 4; 8 4; 10 11];
C = [1 3; 2 3; 5 4; 6 4; 7 8];
B = reassignUnique(A);
assert(all(B==C, 'all'), 'Test failed');
disp('Test passed');
end