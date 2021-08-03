function R = findRotMat(p0, p1)
% calculate cross and dot products
C = cross(p0, p1) ; 
D = dot(p0, p1) ;
NP0 = norm(p0) ; % used for scaling
if ~all(C==0) % check for colinearity    
    Z = [0 -C(3) C(2); C(3) 0 -C(1); -C(2) C(1) 0] ; 
    R = (eye(3) + Z + Z^2 * (1-D)/(norm(C)^2)) / NP0^2 ; % rotation matrix
else
    R = sign(D) * (norm(p1) / NP0) ; % orientation and scaling
end
end