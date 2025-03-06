%APPLY_ROBUSTIFIER
%
% err - MxN matrix of N M-dimensional residual blocks to be robustified
% Jt - DxMxN array of N transposed MxD 
function [cost, err, Jt] = apply_robustifier(robustifier, err, Jt)
% Compute the robust cost
sq_norm = sum(err.^2, 1);
rho = robustifier(sq_norm);
cost = rho(1,:);
if nargout < 2
    return;
end
% Compute the Triggs coefficients
D = sqrt(1 + max(2 * sq_norm .* rho(3,:) ./ (rho(2,:) - 1e-100), 0));
rho(2,:) = sqrt(rho(2,:));
rho(1,:) = rho(2,:) ./ D;
if nargout > 2
    rho(3,:) = max((1 - D) ./ (sq_norm - 1e-100), 0);
    % Apply the Triggs coefficient to the Jacobian
    Jt = Jt - bsxfun(@times, tmult(Jt, reshape(err, size(err, 1), 1, [])), shiftdim(bsxfun(@times, rho(3,:), err), -1));
    Jt = bsxfun(@times, Jt, shiftdim(rho(2,:), -1));
end
% Apply to the residual
err = bsxfun(@times, rho(1,:), err);
end
