function d = compute_l2p_distance(l, p)
%COMPUTE_L2L_DISTANCE Compute the shortest distance between two input
%lines.
% [ Inputs ]
%   l (6 x 1): line comprising [direction; position]
%   p (3 x n): n set of 3D points

% [ Output ]
%   d (1 x n): shortest between two lines

% Find normal to parallel planes
d = sqrt(sum(cross(l(1:3), bsxfun(@minus, p, l(4:6))).^2));

end

