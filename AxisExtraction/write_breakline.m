function write_breakline(breakline, isbase, isrim, pot_id, frag_id, extended, outer)
%UNTITLED2 이 함수의 요약 설명 위치
%   자세한 설명 위치
% 
if nargin < 6, extended = false;
end

if nargin < 7, outer = false;
end

% Variables
% root_dir = load_root_dir('D:/240718/Data_change/H(re)', extended);
root_dir = load_root_dir('D:/SFS_BB_temp/Plt_A', extended);
pcd_file_path = sprintf('%s/Breaklines/Plt_%s_Piece_%02d_Breakline_%01d.pcd', ...
                        root_dir, pot_id, frag_id, outer);
num_segments = numel(breakline);
num_points = calculate_num_points_in_breakline(breakline);
frag_type = base_and_rim_to_frag_type(isbase, isrim);

%% write file
fid = fopen(pcd_file_path, 'w');

% header
fprintf(fid,'%s\r\n', '# .PCD v0.7 - Point Cloud Data file format');

% file spec
fprintf(fid,'# %d %d %d\r\n', num_segments, num_points, frag_type);
num_points_per_segment = cellfun(@(X) size(X, 2), breakline);
point_idx = cumsum([0; num_points_per_segment]);
for i = 1:num_segments
    fprintf(fid, '# %d %d %d\r\n', point_idx(i)+1, point_idx(i+1), isrim(i));
end

% another set of headers
fprintf(fid,'%s %.1f\r\n', 'VERSION', 0.7);
fprintf(fid,'%s %s\r\n', 'FIELDS', 'x y z normal_x normal_y normal_z curvature');
fprintf(fid,'%s %s\r\n', 'SIZE', '4 4 4 4 4 4 4');
fprintf(fid,'%s %s\r\n', 'TYPE', 'F F F F F F F');
fprintf(fid,'%s %d %d %d %d %d %d %d\r\n', 'COUNT', 1, 1, 1, 1, 1, 1, 1);
fprintf(fid,'%s %d\r\n', 'WIDTH', num_points);
fprintf(fid,'%s %d\r\n', 'HEIGHT', 1);
fprintf(fid,'%s %d %d %d %d %d %d %d\r\n', 'VIEWPOINT', 0, 0, 0, 1, 0, 0, 0);
fprintf(fid,'%s %d\r\n', 'POINTS', num_points);
fprintf(fid,'%s %s\r\n', 'DATA', 'ascii');

% breakline
for i = 1:num_segments
    for j = 1:num_points_per_segment(i)
        fprintf(fid, '%.8f %.8f %.8f %.8f %.8f %.8f %e\r\n', ...
                breakline{i}(1,j), ...
                breakline{i}(2,j), ...
                breakline{i}(3,j), ...
                breakline{i}(4,j), ...
                breakline{i}(5,j), ...
                breakline{i}(6,j), ...       
                breakline{i}(7,j) ...                
                );
    end
end

fclose(fid);
return
i = 1;
tline = fgetl(fid);
A{i} = tline;
while ischar(tline)
    i = i+1;
    tline = fgetl(fid);
    A{i} = tline;
end
fclose(fid);
% change the base and rim flag
breakline_info = split(A{2});
num_breakline = str2num(breakline_info{2});
num_breakline_points = str2num(breakline_info{3});
breakline_info{end} = num2str(calculate_frag_type);
A{2} = ['# ', breakline_info{2}, ' ', breakline_info{3}, ' ', breakline_info{end}];

% rims = find(isrim);
if sum(isrim) > 1
    warning('More than 1 rim detected');
end
for iline = 1:num_breakline
    % turn on the rim flag
    rim_info = split(A{2+iline});
    if isrim(iline)
        rim_info{end} = '1';
    else
        rim_info{end} = '0';
    end
    A{2+iline} = ['# ', rim_info{2}, ' ', rim_info{3}, ' ', rim_info{end}];
end

% Write cell A into txt
fid = fopen(pcd_file_path, 'w');
% fid = fopen(sprintf('Z:/2021/ICCV/Pottery Data(Closest to ICCV - UniformSsamplingRadius1.0_18_10_4.05)/%s-%02d/breakline/%s-%02d-%02d_Surface_0_Completebreakline.pcd', ...
%                  namespace, pot_id, namespace, pot_id, frag_id), 'w');

for i = 1:numel(A)
    if A{i+1} == -1
        fprintf(fid,'%s', A{i});
        break
    else
        fprintf(fid,'%s\r\n', A{i});
    end
end
fclose(fid);

end

