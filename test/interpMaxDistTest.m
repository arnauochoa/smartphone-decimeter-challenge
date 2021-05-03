% Test for interpMaxDist

maxDist = 1;
f = @(x) sin(x/4);

xRef = (-5:.1:45)';
yRef = f(xRef);

xSamp = [-3:5 10:20 24.5:40.5]';
vSamp = f(xSamp);

xq = [-1.5:11.5 18:42]';

vq = interp1gap(xSamp, vSamp, xq, maxDist, 'spline','extrap',nan);

figure; hold on;
plot(xRef, yRef);
plot(xSamp, vSamp, 'x');
plot(xq, vq, 'o');
xline(xq);
legend('f(x)','Samples', 'Interpolation', 'Query');

% indGood = [1:7 13:17 22:37]';
% isGood = false(size(xq));
% isGood(indGood) = true;
% 
% assert(isequal(size(vq), size(xq)), 'Sizes do not match');
% disp('Test 1 OK');
% assert(~any(isnan(vq(isGood))), 'Some good value is nan');
% disp('Test 2 OK');
% assert(all(isnan(vq(~isGood))), 'Some value should be nan');
% disp('Test 3 OK');