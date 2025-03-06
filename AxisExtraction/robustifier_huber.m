function Rho = robustifier_huber(s, width)
%ROBUSTIFIER_HUBER computes the Triggs coefficients of the Huber
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

if nargin < 2
    width = 1.0;
end

% Inliers
Rho = [s(:)'; ...
       ones(1, numel(s))
       zeros(1, numel(s))];

% Outliers
tau_sq = width * width;
outliers = s > tau_sq;
s = s(outliers);
sqrt_s = sqrt(s);
Rho(1,outliers) = 2.0 * width * sqrt_s - tau_sq;
sqrt_s = width ./ sqrt_s;
Rho(2,outliers) = sqrt_s;
Rho(3,outliers) = -0.5 * (sqrt_s ./ s);
end

