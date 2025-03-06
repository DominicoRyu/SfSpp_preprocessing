function [isbase, isrim] = check_base_and_rim(pot_id, frag_id, extended, outer, change_file)

if nargin < 3, extended = false;
end

if nargin < 4, outer = false;
end

if nargin < 5, change_file = false;
end

[breakline, isbase_prev, isrim_prev] = read_breakline(pot_id, frag_id, extended);
r_threshold = 1.0;
h_threshold = 1.0;
dr_threshold = 0.1;
dh_threshold = 0.1;

vt = read_axis(pot_id, frag_id, extended);

num_axes = size(vt, 2);

% visualize_fragment(pot_id, frag_id, extended, true);

h = cell(numel(breakline), num_axes);
r = cell(numel(breakline), num_axes);

mean_r_err = nan(numel(breakline), num_axes);
mean_h_err = nan(numel(breakline), num_axes);
mean_dr = nan(numel(breakline), num_axes);
mean_dh = nan(numel(breakline), num_axes);


for j = 1 : num_axes

    for i = 1 : numel(breakline)
        h{i,j} = vt(4:6,j)' * breakline{i}(1:3,:);
        r{i,j} = compute_l2p_distance([vt(4:6,j); vt(1:3,j)], breakline{i}(1:3,:));   
        mean_r_err(i,j) = std(r{i,j});
        mean_h_err(i,j) = std(h{i,j});
        dr = diff(r{i,j});    
        mean_dr(i,j) = mean(dr(2:end-1));        
        dh = diff(h{i,j});
        mean_dh(i,j) = mean(dh(2:end-1));
        
    end
end

%% check base

C0 = read_surfaces(pot_id, frag_id, extended);

radius_threshold = 25.0;
normal_angle_threshold = 10.0;
angle_diff_threshold = 30.0;
z_bin_width = 5.0; % in mm
for i = 1 : num_axes
    x = align_point_cloud(C0, [vt(4:6,i); vt(1:3,i)]);
    r_ = sqrt(x(1,:).^2 + x(2,:).^2);
    z_ = x(3,:);
    min_z = min(z_);
    max_z = max(z_);


    if max(z_) - min(z_) > 3 * z_bin_width
        [z_counts, z_idx, bin_idx] = histcounts(z_, floor(min(z_)):z_bin_width:ceil(max(z_)));
        num_bins = numel(z_counts);
        mean_angles = nan(num_bins, 1);
        for ibin = 1:num_bins
            % compute average normal angle
            % check whether min(z_) or max(z_) has smaller radius
            
            angles = acosd(abs(sum(x(4:6,bin_idx==ibin) .* [0; 0; 1])));
            mean_angles(ibin) = mean(angles);
        end

        r_of_min_z = r_(z_ == min_z);
        r_of_max_z = r_(z_ == max_z);
        if r_of_min_z < r_of_max_z
            % the base is near min_z   
            drastic_angle_change_detected = sum(diff(mean_angles(1:3)) > angle_diff_threshold) > 0;
            near_vertical_normals = min(mean_angles(1)) < 30.0;            
        else
            drastic_angle_change_detected = sum(diff(mean_angles(end-2:end)) > angle_diff_threshold) > 0;
            near_vertical_normals = min(mean_angles(end)) < 30.0;            
        end
    else
        drastic_angle_change_detected = false;
        near_vertical_normals = false;
    end
    theta = atan2d(x(1,:), x(2,:));
    normal_angles = acosd(abs(sum(x(4:6,:) .* [0; 0; 1])));
    % normal_angles(normal_angles > 90) = 180 - normal_angles(normal_angles > 90);
    base_points = r_ < radius_threshold & normal_angles < normal_angle_threshold;
    base_points_theta = theta(base_points);
    counts = histcounts(base_points_theta, -180:30:180);
end

% check the mode of the count is > 100
min_num_counts = 100;
std_threshold = 0.25;
isbase = 0;
if (num_axes == 1 && max(counts) > min_num_counts && size(C0, 2) > 2000)
   if std(counts) < mean(counts) * std_threshold
        isbase = 1;
    else
        isbase = -1;
    end
end
if ~isbase && drastic_angle_change_detected && near_vertical_normals
    % if sudden angle change is found, assume it is a part base.
    fprintf('part base detected via drastic angle change')
    isbase = -1;
end

%% Check rim
min_num_rim_points = 20;
num_breakline_points = zeros(numel(breakline), 1);
for i = 1:numel(breakline)
    num_breakline_points(i) = size(breakline{i}, 2);
end
isrim = mean_h_err < h_threshold & mean_r_err < r_threshold & num_breakline_points >= min_num_rim_points;
isrim = isrim & (abs(mean_dr) < dr_threshold) & (abs(mean_dh) < dh_threshold);
if sum(isrim(:)) > 1
    % Choose the one with least errors
    best_rim_h = mean_h_err == min(mean_h_err);
    best_rim_r = mean_r_err == min(mean_r_err);
    best_rim_dr = mean_dr == min(mean_dr);
    best_rim_dh = mean_dh == min(mean_dh);
    best_rim_count = isrim .* (best_rim_h + best_rim_r + best_rim_dr + best_rim_dh);
    isrim = best_rim_count == max(best_rim_count);


    % [rim_bl, rim_ax] = find(isrim);
    % mean_h = [];
    % for k = 1:sum(isrim)
        % check it is really the rim
%         r_ = mean(r{rim_bl(k),rim_ax(k)});
%         h_ = mean(h{rim_bl(k),rim_ax(k)});
        
        
%         greater_radius_found = 0;
%         for j = 1 : num_axes
% 
%             for i = 1:numel(breakline)
%                 nearby_points = abs(h{i,j} - h_) < h_threshold;
%                 greater_radius_found = abs(r{i,j}(:,nearby_points) - r_) > r_threshold;
%                 if sum(greater_radius_found) > 0
%                     fprintf('greater radius found')
%                     break;
%                 end
%             end
% 
%             if sum(greater_radius_found) > 0
%                 break;
%             end
%         end
        
%         if sum(greater_radius_found) > 0
%             isrim(rim_bl(k),rim_ax(k)) = 0;
%             continue;
%         end
%         mean_r(k) = r_;
%         mean_h(k) = h_;
%     end
    % check if there are two rims with inconsistent radii - then discard
    % one
%     if ~isempty(mean_h)
%         if std(mean_h) > 2 * h_threshold && std(mean_r) > 2 * r_threshold
%             max_rim = num_breakline_points(isrim) == max(num_breakline_points(isrim));
%             rim_idx = find(isrim);
%             non_rim_idx = ~isrim;
%             isrim(rim_idx(max_rim)) = true;
%             isrim(rim_idx(~max_rim)) = false;
%         end
%     end

end

if sum(isrim(:)) < 1
    fprintf('Pot %s frag %02d base %d rim 0\n', pot_id, frag_id, isbase);
else
    % fprintf('Pot %s frag %02d base: %d\n', pot_id, frag_id, isbase);

    fprintf('Pot %s frag %02d base %d rim %d\n', pot_id, frag_id, isbase, find(isrim));
end


isrim = logical(sum(isrim, 2));

% change the file if rim or base detected
if (isbase ~= isbase_prev || ~isequal(isrim, isrim_prev)) && change_file
    write_breakline(breakline, isbase, isrim, pot_id, frag_id, extended, outer);
%     % replace the base part
%     % Read txt into cell A
%     pcd_file_path = sprintf('%s/breakline/Pot_%s_Piece_%02d_Breakline_0.pcd', root_dir, pot_id, frag_id);
%     fid = fopen(pcd_file_path, 'r');
%     i = 1;
%     tline = fgetl(fid);
%     A{i} = tline;
%     while ischar(tline)
%         i = i+1;
%         tline = fgetl(fid);
%         A{i} = tline;
%     end
%     fclose(fid);
%     % change the base and rim flag
%     breakline_info = split(A{2});
%     num_breakline = str2num(breakline_info{2});
%     num_breakline_points = str2num(breakline_info{3});
%     breakline_info{end} = num2str(calculate_frag_type);
%     A{2} = ['# ', breakline_info{2}, ' ', breakline_info{3}, ' ', breakline_info{end}];
%     
%     % rims = find(isrim);
%     if sum(isrim) > 1
%         warning('More than 1 rim detected');
%     end
%     for iline = 1:num_breakline
%         % turn on the rim flag
%         rim_info = split(A{2+iline});
%         if isrim(iline)
%             rim_info{end} = '1';
%         else
%             rim_info{end} = '0';
%         end
%         A{2+iline} = ['# ', rim_info{2}, ' ', rim_info{3}, ' ', rim_info{end}];
%     end
%     
%     % Check breakline normal directions
% %     X = nan(6, 1);
% %     angle_diff = nan(num_breakline_points, 1);
% %     for ipoint = 1:num_breakline_points
% %         cur_line_data = split(A{2+num_breakline+10+ipoint});
% %         
% %         X(1) = str2double(cur_line_data{1});
% %         X(2) = str2double(cur_line_data{2});
% %         X(3) = str2double(cur_line_data{3});
% %         X(4) = str2double(cur_line_data{4});
% %         X(5) = str2double(cur_line_data{5});
% %         X(6) = str2double(cur_line_data{6});
% %         
% %         [closest_dist, closest_point_idx] = min(sqrt(sum((C0(1:3,:) - X(1:3)).^2)));
% %         
% %         % compute the normal angle between the closest point and the
% %         % current point.
% %         angle_diff(ipoint) = acosd(normalize(X(4:6))' * normalize(C0(4:6,closest_point_idx)));
% %         
% %     end
%     
%     % Write cell A into txt
%     fid = fopen(pcd_file_path, 'w');
%     % fid = fopen(sprintf('Z:/2021/ICCV/Pottery Data(Closest to ICCV - UniformSsamplingRadius1.0_18_10_4.05)/%s-%02d/breakline/%s-%02d-%02d_Surface_0_Completebreakline.pcd', ...
%     %                  namespace, pot_id, namespace, pot_id, frag_id), 'w');
%     
%     for i = 1:numel(A)
%         if A{i+1} == -1
%             fprintf(fid,'%s', A{i});
%             break
%         else
%             fprintf(fid,'%s\r\n', A{i});
%         end
%     end
%     fclose(fid);
    
end


% vt(4:6,i)

end

