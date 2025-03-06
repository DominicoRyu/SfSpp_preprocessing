function R = visualize_axis_of_symmetry_two(C0, C1, breakline, vt, noalign, azimuth, elev, sgn)

if nargin < 3
    breakline = [];
end

if nargin < 5
    noalign = false;
end

if nargin < 6
    azimuth = 0.0;
    elev = 0.0;
end

if nargin < 8
    sgn = 1;
end

sgn = 2 * (double(sgn > 0) - 0.5);

% Visualize the axis and the point cloud.
% clf;

% First axis = reference axis
num_input_axes = size(vt, 2);
% R = nan(3, 3);
if abs(vt(1:3,1)' * [0 0 1]' - 1) < 1e-9
    R = eye(3);
else
    R = [null(vt(1:3,1)'), vt(1:3,1)];
    if det(R) < 0.0, R(:,3) = -R(:,3);
    end
end

% Take the first one as the reference vector
R = R';
R = [sgn, 0, 0; 0 1 0;   0, 0, sgn] * R;
R = [cosd(azimuth), -sind(azimuth), 0; sind(azimuth), cosd(azimuth), 0;  0 0 1] * R;
X_transformed_0 = R * bsxfun(@minus, C0(1:3,:), vt(4:6,1));
X_transformed_1 = R * bsxfun(@minus, C1(1:3,:), vt(4:6,1));
n_transformed_0 = R * C0(4:6,:);
n_transformed_1 = R * C1(4:6,:);

% disp(size(X_transformed_0, 2) + size(X_transformed_1, 2));

if ~isempty(breakline)
    for i = 1 : numel(breakline)
        breakline{i}(1:3,:) = R * bsxfun(@minus, breakline{i}(1:3,:), vt(4:6,1));
        breakline{i}(4:6,:) = R * breakline{i}(4:6,:);
    end
end
% X_transformed = [sgn, 0, 0; 0 1 0;   0, 0, sgn] * X_transformed;
% X_transformed = [cosd(azimuth), -sind(azimuth), 0; sind(azimuth), cosd(azimuth), 0;  0 0 1] * X_transformed;
% X_transformed = [cosd(elev), 0, -sind(elev); 0, 1, 0;  sind(elev), 0, cosd(elev)] * X_transformed;

% Take away offset in the z direction.
if norm(vt(4:6)) > 1e-9
    z_median = median([X_transformed_0(3,:), X_transformed_1(3,:)]);
    X_transformed_0(3,:) = X_transformed_0(3,:) - z_median;
    X_transformed_1(3,:) = X_transformed_1(3,:) - z_median;
    if ~isempty(breakline)
        for i = 1 : numel(breakline)
            breakline{i}(3,:) = breakline{i}(3,:) - z_median;
        end   
    end
end

% compute r and theta
r0 = sqrt(sum(X_transformed_0(1:2,:).^2));
[r0, ir] = sort(r0);
theta0 = atan2d(X_transformed_0(2,ir), X_transformed_0(1,ir));
h0 = X_transformed_0(1:2,ir);

degree_of_fitness = n_transformed_0(3,ir);
end_of_base = find(acosd(degree_of_fitness) > 30);

z_max = max([X_transformed_0(3,:), X_transformed_1(3,:)]);
z_min = min([X_transformed_0(3,:), X_transformed_1(3,:)]);
% z_interval = linspace(2*z_min, 2*z_max, 100);

% z(3, :) = z_interval;
origin = zeros(3,1);
% clf
vt_transformed = nan(6, num_input_axes);
num_intervals = 100;
z = nan(3, num_intervals, num_input_axes);
for i = 1:num_input_axes
    vt_transformed(1:3,i) = R * vt(1:3,i);
    vt_transformed(4:6,i) = R * (vt(4:6,i) - vt(4:6,1));
    t_min = (z_min - 20 - vt_transformed(6,i)) / vt_transformed(3,i);
    t_max = (z_max + 20 - vt_transformed(6,i)) / vt_transformed(3,i);
    t_interval = linspace(t_min, t_max, 100);
    z(:,:,i) = vt_transformed(1:3,i) * t_interval + vt_transformed(4:6,i);
end

hold on
if num_input_axes < 2
    plot3(z(1,:,1), z(2,:,1), z(3,:,1), '-b', 'LineWidth',3);
else
%     plot3(z(1,:,1), z(2,:,1), z(3,:,1), '-g', 'LineWidth',3);
    hold on
    if num_input_axes > 2
        plot3(z(1,:,3), z(2,:,3), z(3,:,3), '-b', 'LineWidth', 2);
        plot3(z(1,:,2), z(2,:,2), z(3,:,2), '-m', 'LineWidth', 1);        
    else
        plot3(z(1,:,2), z(2,:,2), z(3,:,2), '-b', 'LineWidth', 2);
    end
end

if noalign
    z(3, :) = z(3, :) + z_median;
    origin(3) = origin(3) + z_median;

    z = R * z + vt(4:6);
    origin = R * origin + vt(4:6);
    visualize_point_cloud(C);
else
    visualize_point_cloud(X_transformed_0, 1.0, 'bo');
%     quiver3(X_transformed_0(1,1:100:end), X_transformed_0(2,1:100:end), X_transformed_0(3,1:100:end), ...
%     n_transformed_0(1,1:100:end), n_transformed_0(2,1:100:end), n_transformed_0(3,1:100:end));
    visualize_point_cloud(X_transformed_1, 1.0, 'ko');
%     quiver3(X_transformed_1(1,1:100:end), X_transformed_1(2,1:100:end), X_transformed_1(3,1:100:end), ...
%     n_transformed_1(1,1:100:end), n_transformed_1(2,1:100:end), n_transformed_1(3,1:100:end));    
    title('Blue is supposed to be inner');
    
    if ~isempty(breakline)
        hold on;
        styles = {'-ro', '-mo', '-yo', '-go', '-co'};
        for i = 1 : numel(breakline)
            visualize_point_cloud(breakline{i}(1:3,:), 1.0, styles{1});
            num_points = min(20, size(breakline{i}, 2));
            quiver3(breakline{i}(1,1:num_points), breakline{i}(2,1:num_points), breakline{i}(3,1:num_points), ...
                    breakline{i}(4,1:num_points), breakline{i}(5,1:num_points), breakline{i}(6,1:num_points));
        end
            
    end
%     if size(vt, 2) > 1
%         % If multiple vt vectors are provided, use the first one as the
%         % reference and show one or two other vectors.
%         vt = 
%     end    
end

zoom(2.0);
daspect([1 1 1]);
axis off
hold off
% zlim([-50 50]);
ylim([-100 100]);
xlim([-100 100]);   

if num_input_axes > 2
    legend('Best proposal', 'Current proposal');
else
    legend('Current solution');
end


% plot3(origin(1), origin(2), origin(3), 'g.', 'MarkerSize',50);
view([0 elev]);



end
