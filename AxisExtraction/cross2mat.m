function S = cross2mat(omega)
%CROSS2MAT Convert the cross product operator to the skew symmetric matrix
%form.
%   [ Input ]
%   omega (3 x n): n set of 3D points
%   [ Output ]
%   S (3 x 3 x n): n set of skew symmetric matrices

n = size(omega, 2);
S = zeros(3, 3, n);

S(3,2,:) = omega(1,:);
S(2,3,:) = - omega(1,:);
S(1,3,:) = omega(2,:);
S(3,1,:) = - omega(2,:);
S(2,1,:) = omega(3,:);
S(1,2,:) = - omega(3,:);

end

