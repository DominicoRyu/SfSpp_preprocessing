function Rho = robustifier_gm(s, width)
%ROBUSTIFIER_GM computes the Triggs coefficients of the Geman-McClure
%kernel for given squared norms of input residuals.
%
%   [ Inputs ]
%   s: squared norms of input residuals (1 x n).
%   
%   width: width of the robust kernel.  
%
%   [ Outputs ]
%   Rho: coefficients of the robust kernel (3 x n).
%        Each column contains rho, rho' and rho''  down the row.

if nargin < 2, width = 1.0;
end

tau_sq = width * width;

Rho = nan(3, max(size(s)));

Rho(3,:) = tau_sq ./ (s + tau_sq);
Rho(1,:) = 2 * s .* Rho(3,:);
Rho(2,:) = 2 * Rho(3,:) .^ 2;
Rho(3,:) = -2 * Rho(2,:) .* Rho(3,:) / tau_sq;

end