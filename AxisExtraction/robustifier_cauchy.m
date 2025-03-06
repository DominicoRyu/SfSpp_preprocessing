function Rho = robustifier_cauchy(s)
%ROBUSTIFIER_TRIVIAL computes the Triggs coefficients of the Geman-McClure
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

Rho = nan(3, max(size(s)));

s = s + 1.0;
Rho(1,:) = log(s);
Rho(2,:) = 1 ./ s;
Rho(3,:) = - Rho(2,:) .^ 2;

end