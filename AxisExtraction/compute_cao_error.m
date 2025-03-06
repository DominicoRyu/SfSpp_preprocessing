function [R, J] = compute_cao_error(vt, X, reversed)
%COMPUTE_CAO_ERROR Compute the Cao error as  defined in Cao and Mumford.
% [ Inputs ]
%   vt (6 x 1): symmetric axis [direction; position];
%   X (6 x n): n set of [3D point; surface normal]s
%   reversed (bool): whether to reverse the normal directions.

% [ Output ]
%   R (3 x n): n set of error vectors.
%   J (3 x 6 x n): Corresponding Jacobian tensor.

if nargin < 3, reversed = false;
end

if reversed, X(4:6,:) = - X(4:6,:);
    % Reverse the normal direction if requested.
end

vt = reshape(vt, 6, 1, []);
T1 = bsxfun(@minus, X(1:3,:), vt(4:6,:,:));
T2 = cross(T1, vt(1:3,:,:), 1);
T3 = cross(reshape(X(4:6,:), 3, [], 1), vt(1:3,:,:), 1);

% Pottmann's error
% R = cross(cross(R, vt(1:3)), cross(X(4:6,:), vt(1:3)));

% Cao's error
% R = T2;
R = T2 - bsxfun(@times, sqrt(sum(T2.^2) ./ sum(T3.^2)), T3);

if nargout > 1
    cross_T1 = cross2mat(T1);
    cross_vhat = cross2mat(repmat(vt(1:3), 1, size(X, 2)));
    cross_nhat = cross2mat(X(4:6,:));
    norm_T2 = sqrt(sum(T2.^2)); 
    norm_T3 = sqrt(sum(T3.^2));
    norm_T2_by_T3 = norm_T2 ./ norm_T3;
    
    T4 = bsxfun(@rdivide, T2, norm_T2);
    T4 = reshape(T4, [size(T4, 1) 1 size(T4, 2)]);
    T5 = bsxfun(@rdivide, T3, norm_T3);
    T5 = reshape(T5, [size(T5, 1) 1 size(T5, 2)]);
    
    J = nan(size(R, 1), 6, size(R, 2));   
    J(:,1:3,:) = cross_T1 - tmult(T5, tmult(T4, cross_T1, true)) ...
        - bsxfun(@times, cross_nhat, reshape(norm_T2_by_T3, [1 1 numel(norm_T2)])) ...
        + bsxfun(@times, tmult(T5, tmult(T5, cross_nhat, true)), reshape(norm_T2_by_T3, [1 1 numel(norm_T2)]));
    J(:,4:6,:) = cross_vhat - tmult(T5, tmult(T4, cross_vhat, true));    
    
    % Project Jacobian to the tangent space of the manifold.  
    P = eye(3) - vt(1:3) * vt(1:3)';
    J(:,1:3,:) = tmult(J(:,1:3,:), P) - tmult(tmult(J(:,4:6,:), vt(1:3)), vt(4:6)');
end

end

