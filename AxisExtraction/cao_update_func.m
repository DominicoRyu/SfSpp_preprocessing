function vt = cao_update_func(vt, dvt)
%COMPUTE_CAO_ERROR Compute the Cao error as  defined in Cao and Mumford.
% [ Inputs ]
%   vt (6 x 1): symmetric axis [direction; position];
%   dvt (6 x 1): change in vt

% [ Output ]
%   vt (6 x 1): updated symmetric axis.

% First, make a Euclidean update (i.e. on the tangent plane)
vt = vt + dvt;

% Retract the axis direction vector back to the manifold of unit sphere.
vt(1:3) = vt(1:3) / norm(vt(1:3));

% Make an approximate retraction for the position vector to satisfy the
% orthogonality constraint.
vt(4:6) = vt(4:6) - vt(1:3) * (vt(1:3)' * vt(4:6));

end

