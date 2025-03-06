function [x, err, xlog] = optimize_using_lm(x, res_func, update_func, robustifier, max_iters, func_tol)
%OPTIMIZE_USING_LM Summary of this function goes here
%   x: optimization variable
%   res_func: function that outputs residual and Jacobian.
%   update_func: function for updating variables

if nargin < 5
    opts.max_iters = 300;
else
    opts.max_iters = max_iters;
end

if nargin < 4
    opts.robustifier = @robustifier_gm;
else
    opts.robustifier = robustifier;
end

if nargin < 6
    opts.func_tol = 1e-9;
else
    opts.func_tol = func_tol;
end

opts.max_trials = 20;
opts.max_lambda = 1e+14;
opts.min_lambda = 1e-14;
opts.init_lambda = 1e-1;

[err, J] = res_func(x);
[cost, err, Jt] = apply_robustifier(opts.robustifier, err, permute(J, [2 1 3]));
J = reshape(permute(Jt, [2 3 1]), [], numel(x));
err = err(:);
J(:,1:3) = J(:,1:3) * (speye(3) - x(1:3) * x(1:3)');
JTJ = J'*J;
grad = J'*err;
lambda = opts.init_lambda;
lambda_inc_factor = 2.0;
eval = 0;
fprintf('[iter][eval] cost actual_cost_change |grad| |TR| MQ\n');
fprintf('[%04d][%04d] %.6e %.3e %.2e %.2e %.2e\n', 0, 0, 0.5 * sum(cost), 0, norm(grad), 1.0/lambda, 1.0);
xlog(:,eval+1) = x;

for iter = 1:opts.max_iters
    for trial = 1:opts.max_trials
        eval = eval+1;
        dx = - (JTJ +  kron(diag([1 1]), x(1:3) * x(1:3)') + lambda * speye(size(JTJ))) \ grad;
        x_cand = update_func(x, dx);
        xlog(:,eval+1) = x_cand;
        
        [err_cand, J_cand] = res_func(x_cand);
        [cost_cand, err_cand, Jt] = apply_robustifier(opts.robustifier, err_cand, permute(J_cand, [2 1 3]));

        J_cand = reshape(permute(Jt, [2 3 1]), [], numel(x));
        J_cand(:,1:3) = J_cand(:,1:3);
        err_cand = err_cand(:);

        expected_cost_change = 0.5 * (sum(err.^2) - sum((err + reshape(J * dx, [], 1)) .^ 2));
        actual_cost_change = 0.5 * sum(cost - cost_cand);
        model_quality = actual_cost_change / expected_cost_change;        
        
        fprintf('[%04d][%04d] %.6e %.3e %.2e %.2e %.2e\n', ...
            iter, eval, 0.5 * sum(cost_cand), actual_cost_change, norm(grad), 1.0/lambda, model_quality);
        
        if model_quality > 0
            x = x_cand;
            J = J_cand;
            J(:,1:3) = J(:,1:3) * (speye(3) - x(1:3) * x(1:3)');
            JTJ = J'*J;            
            err = err_cand;
            cost_diff = actual_cost_change;
            cost = cost_cand;
            grad = J'*err;     
            lambda = max(opts.min_lambda, lambda * max(1/3, 1 - (2*model_quality - 1)^3));
            lambda_inc_factor = 2.0;
            break
        end
        lambda = min(lambda * lambda_inc_factor, opts.max_lambda);
        lambda_inc_factor = lambda_inc_factor * 2.0;
    end
    
    if trial == opts.max_trials, break
    end
    
    if cost_diff < 0.5 * opts.func_tol * sum(cost)
        break
    end
    
    if cost < opts.func_tol, break
    end
end

end

