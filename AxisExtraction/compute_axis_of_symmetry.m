function [vt, num_inliers, combined_err, initial_samples, vt_log, vt_sofar_best_log] = compute_axis_of_symmetry(C)

% Normalize normals
C(4:6,:) = C(4:6,:) ./ sqrt(sum(C(4:6,:).^2));

num_points = size(C,2);
num_sample_set = 6;

max_iters = 1000;
num_inliers = cell(max_iters, 1);
vt = cell(max_iters, 1);
inlier_threshold = 1.0;

combined_errs = cell(max_iters, 1);
kidx = cell(max_iters);

for iter = 1 : max_iters
    kidx{iter} = randsample(num_points, num_sample_set);
    Cs = C(:,kidx{iter});
    mCs = mean(Cs(1:3,:), 2);
    Cs(1:3,:) = Cs(1:3,:) - mCs;
    scale = mean(abs(Cs(:)));
    Cs(1:3,:) = Cs(1:3,:) / scale;
    vt{iter} = compute_pottmann_axis(Cs);
    vt{iter}(4:6) = vt{iter}(4:6) * scale + mCs;
    % Project displacement to the tangent space of the normal direction.
    vt{iter}(4:6) = vt{iter}(4:6) - (vt{iter}(1:3)' * vt{iter}(4:6)) * vt{iter}(1:3);   
    
    combined_err = compute_biaxial_cao_error(vt{iter}, C);
    combined_err = apply_robustifier(@robustifier_gm, combined_err);
    inliers = abs(combined_err) < inlier_threshold;
    combined_errs{iter} = sum(combined_err);

    num_inliers{iter} = sum(inliers);
end

num_inliers = cell2mat(num_inliers);
combined_err = cell2mat(combined_errs);

vt = horzcat(vt{:});
if nargout > 4
    vt_log = vt;
    cummin_log = cummin(combined_err);
    [~, loc] = ismember(cummin_log, combined_err);
    vt_sofar_best_log = vt_log(:, loc);
end

[combined_err, ia] = sort(combined_err);
num_inliers = num_inliers(ia);
vt = vt(:, ia);
initial_samples = kidx{ia};

end
