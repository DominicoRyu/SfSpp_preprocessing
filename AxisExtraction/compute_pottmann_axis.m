function vt = compute_pottmann_axis(X)
%COMPUTE_SYMMETRIC_AXIS Compute symmetric axis from a set of 3D points with
%surface normal vectors (Pottmann et al., 1996)
% [ Input ]
%   X (6 x N): N set of [point; normal vector]s
% [ Output ]
%   nhat (6 x N): axis of symmetry in Plucker coordinates

% Compute the linear system matrix.
J = euc2plucker(X([4:6,1:3],:)); J = J([4:6,1:3],:)';
JTJ = J'*J;

% Compute the reduced system matrices as there are only 3 eigenvalues.
M11 = JTJ(1:3,1:3);
M12 = JTJ(1:3,4:6);
M22 = JTJ(4:6,4:6);
M = (M11 - M12 * (M22 \ M12'));

% Solve the reduced eigenvalue problem and output results.
[V, D] = svd(M);
s = diag(D);
[~, ia] = sort(s);
v = V(:, ia(1));
vbar = - M22 \ (M12' * v);
vt = [v; cross(v, vbar)];

end

