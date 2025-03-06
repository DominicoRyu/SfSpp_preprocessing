function [R, J] = compute_rim_errors(vt, r, X, reversed)
%COMPUTE_RIM_ERROR Compute the errors of the rim pars.
% [ Inputs ]
%   vt (6 x 1): symmetric axis [direction; position];
%   X (6 x n): n set of [3D point; surface normal]s
%   reversed (bool): whether to reverse the normal directions.

% [ Output ]
%   R (3 x n): n set of error vectors.
%   J (3 x 6 x n): Corresponding Jacobian tensor.

if nargin < 4, reversed = false;
end

if reversed, X(4:6,:) = - X(4:6,:);
    % Reverse the normal direction if requested.
end

vt = reshape(vt, 6, 1, []);
T1 = bsxfun(@minus, X(1:3,:), vt(4:6,:,:));
% Compute dot product (i.e. height w.r.t. some reference point)
R{2} = sum(bsxfun(@times, vt(1:3,:,:), T1));
T3 = T1 - bsxfun(@times, vt(1:3,:,:), R{2});
T4 = sqrt(sum(T3.^2));

% Cao's error
R{1} = r - T4;

if nargout > 1   
    J{1} = nan(size(R{1}, 1), 7, size(R{1}, 2));
    J(:,7,:) = 1;
    % derr/dv
    J(:,1:3,:) = cross_T1 - tmult(T5, tmult(T4, cross_T1, true)) ...
        - bsxfun(@times, cross_nhat, reshape(norm_T2_by_T3, [1 1 numel(norm_T2)])) ...
        + bsxfun(@times, tmult(T5, tmult(T5, cross_nhat, true)), reshape(norm_T2_by_T3, [1 1 numel(norm_T2)]));
    J(:,4:6,:) = cross_vhat - tmult(T5, tmult(T4, cross_vhat, true));    
    % Project Jacobian to the tangent space of the manifold.  
    P = eye(3) - vt(1:3) * vt(1:3)';
    J(:,1:3,:) = tmult(J(:,1:3,:), P) - tmult(tmult(J(:,4:6,:), vt(1:3)), vt(4:6)');
end

end

