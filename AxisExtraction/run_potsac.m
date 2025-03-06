function vt = run_potsac(C1, C2)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

%% Extract 10 best axes
% Run MLESAC
if nargin < 2
    num_points = size(C1, 2);
    C = C1;
    C1 = C(:,1:(0.5*num_points));
    C2 = C(:,(0.5*num_points):1:num_points);
else
    C = [C1, C2];
    num_points = size(C, 2);
end
% [vt, ~, costs] = compute_axis_of_symmetry_v2(C1(:,1:10:end), C2(:,1:10:end));
[vt, ~, costs] = compute_axis_of_symmetry(C(:,1:10:num_points));

% Compute PotSAC 2 cost for each 20 vt.
% max_poly_order = 4;
num_candidates =10; %원래는 10
% num_points = size(C, 2);
% costs = nan(num_candidates, 2);
% for i = 1 : num_candidates
%     costs(i, 1) = compute_potsac_v2_cost(C(:,1:10:(0.5*num_points)), vt(:,i), max_poly_order, 10.0);
%     costs(i, 2) = compute_potsac_v2_cost(C(:,(0.5*num_points):10:num_points), vt(:,i), max_poly_order, 10.0);
%     costs = sum(costs, 2);
% end
% [~, ia] = sort(costs);

% Refine axis

for i = 1 : num_candidates
    vt(:,i) = refine_axis(vt(:,i), C(:,1:10:end), 2, [], [], 300, 1e-3);
end
vt = vt(:,1:num_candidates);

robustifier = @robustifier_huber;
cost = nan(num_candidates, 1);
for i = 1 : num_candidates
    residual = compute_biaxial_cao_error(vt(:,i), C(:,1:10:end));
    cost(i) = sum(apply_robustifier(robustifier, residual));
    % cost(i) = sqrt( sum(residual .^2) / numel(residual) );
end

%% Discard repetitive axes
% check the angle > 10 deg and get rid of potential repetitions
dot_vt = round(abs(vt(1:3,:)' * vt(1:3,1)), 2);
angles = triu(abs(vt(1:3,:)' * vt(1:3,:)));
% trans = triu(abs(vt(1:3,:)' * vt(1:3,:)));
% Find axes that are similar
[ii,jj] = find((angles - eye(size(angles))) > 0.9848);
repeats = false(1, num_candidates);
repeats(jj) = true;
unique_vt_idx = find(~repeats);

% check cost and discard ones with comparatively high cost
vt = vt(:,unique_vt_idx);
cost = cost(unique_vt_idx);
good_cost_idx = cost < cost(1) * 1.1; % within 10%
cost = cost(good_cost_idx);
vt = vt(:,good_cost_idx);


for i = 1 : sum(good_cost_idx)
    vt(:,i) = refine_axis(vt(:,i), C, 2, [], [], 300, 1e-9);
end

% check if any axes have merged
dot_vt = round(abs(vt(1:3,:)' * vt(1:3,1)), 2);
angles = triu(abs(vt(1:3,:)' * vt(1:3,:)));
repeats = false(1, sum(good_cost_idx));
[ii,jj] = find((angles - eye(size(angles))) > 0.9848);
repeats(jj) = true;
vt = vt(:,~repeats);
cost = cost(~repeats);
good_cost_idx = cost < cost(1) * 1.1; % within 10%
cost = cost(good_cost_idx);
vt = vt(:,good_cost_idx);

[cost, ia] = sort(cost);
vt = vt(:, ia);

end