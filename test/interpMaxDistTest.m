% Test for interpMaxDist

maxDist = 0.6;
x = [-3:5 10:20 24.5:40.5];
v = sin(x/4);

xq = [-1.5:11.5 18:42];
indGood = [1:7 13:17 22:37];

vq = interpMaxDist(x, v, xq, maxDist, 'linear');

figure; hold on;
plot(x, v, 'x-');
plot(xq, vq, 'o');
xline(xq);
legend('Source', 'Interpolation', 'Query');

isGood = false(size(xq));
isGood(indGood) = true;

assert(isequal(size(vq), size(xq)), 'Sizes do not match');
disp('Test 1 OK');
assert(~any(isnan(vq(isGood))), 'Some good value is nan');
disp('Test 2 OK');
assert(all(isnan(vq(~isGood))), 'Some value should be nan');
disp('Test 3 OK');