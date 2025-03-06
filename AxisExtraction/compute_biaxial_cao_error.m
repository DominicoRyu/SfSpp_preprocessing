function [R, J] = compute_biaxial_cao_error(vt, X)
%COMPUTE_CAO_ERROR Compute the Cao error as  defined in Cao and Mumford.
% [ Inputs ]
%   vt (6 x 1): symmetric axis [direction; position];
%   X (6 x n): n set of [3D point; surface normal]s
%   reversed (bool): whether to reverse the normal directions.

% [ Output ]
%   R (1 x n): n set of error vectors.
%   J (1 x 6 x n): Corresponding Jacobian tensor.

s = 100.0;

if nargout < 2

    cost_fwd = sum(compute_cao_error(vt, X).^2);
    cost_rev = sum(compute_cao_error(vt, X, true).^2);
    c = min(cost_fwd, cost_rev);    
    R = - 1.0 / s * log(exp(s * -(cost_fwd - c)) + exp(s * -(cost_rev - c))) + c;  

elseif nargout > 1
    % Compute Jacobian
    [R_fwd, J_fwd] = compute_cao_error(vt, X);
    [R_rev, J_rev] = compute_cao_error(vt, X, true);
    
    cost_fwd = sum(R_fwd.^2);
    cost_rev = sum(R_rev.^2);
    c = min(cost_fwd, cost_rev);
    base_fwd = exp(s * -(cost_fwd - c));
    base_rev = exp(s * -(cost_rev - c));
    base = base_fwd + base_rev;
    R = - 1.0 / s * log(base) + c; 
    
    R_fwd = reshape(R_fwd, [size(R_fwd,1), 1, size(R_fwd,2)]);
    R_rev = reshape(R_rev, [size(R_rev,1), 1, size(R_rev,2)]);
    g_fwd = reshape(tmult(J_fwd,  R_fwd, 1), 6, []);
    g_rev = reshape(tmult(J_rev,  R_rev, 1), 6, []);
    J = 2 * ( bsxfun(@times, g_fwd, base_fwd ./ base) + bsxfun(@times, g_rev, base_rev ./ base));
    J = reshape(J, [1 size(J,1) size(J,2)]);  
end

end

