function [breakline, isbase, isrim] = read_breakline(pot_id, frag_id, extended, outer)

% 기본값 설정
isbase = 0; % '0'이나 다른 적절한 기본값으로 설정
isrim = false; % 'false' 또는 다른 적절한 기본값으로 설정
breakline = []; % 빈 배열로 초기화

% 입력 인자 검사
if nargin < 3, extended = false; end
if nargin < 4, outer = false; end

% 루트 디렉토리 및 파일 경로 설정
% root_dir = load_root_dir('D:/240718/GT/Breaklines', extended);
% root_dir = load_root_dir('D:/240718/Data_change/H(re)/Breaklines_axis', extended);
root_dir = load_root_dir('D:/SFS_BB_temp/Plt_A/Breaklines', extended);
pcd_file_path = sprintf('%s/Plt_%s_Piece_%02d_Breakline_%01d.pcd', root_dir, pot_id, frag_id, outer);

% 파일 존재 여부 확인
if ~exist(pcd_file_path, 'file')
    fprintf('File does not exist: %s\n', pcd_file_path);
    return
end

% 파일 열기
fid = fopen(pcd_file_path);
if fid == -1
    fprintf('Failed to open file: %s\n', pcd_file_path);
    return
end

fgetl(fid); % PCD file format
line = split(fgetl(fid)); % num segments & points
num_segments = str2double(line{2});
num_points = str2double(line{3});
frag_type = str2double(line{4});
[isbase, ~] = frag_type_to_base_and_rim(frag_type);

breakline = cell(num_segments, 1);
bl_indices = nan(num_segments, 2);
isrim = false(num_segments, 1);
for i = 1:num_segments
    line = split(fgetl(fid));
    bl_indices(i, 1) = str2double(line{2});
    bl_indices(i, 2) = str2double(line{3});
    isrim(i) = str2double(line{4});
end

% % 추가 디버깅 출력
% fprintf('Debug - Read num_segments: %d, num_points: %d, isbase: %d\n', num_segments, num_points, isbase);
% disp(bl_indices);
% disp(isrim);

% skip through lines
for i = 1:10
    fgetl(fid);
end

for i = 1:num_segments
    num_points_in_segment = bl_indices(i,2) - bl_indices(i,1) + 1;
    breakline{i} = nan(7, num_points_in_segment);
    for j = 1:num_points_in_segment
        line = split(fgetl(fid));
        if length(line) < 7
            fprintf('Incomplete data at segment %d, point %d. Expected 7, got %d.\n', i, j, length(line));
            continue;  % Skip this point due to insufficient data
        end
        for k = 1:7
            breakline{i}(k,j) = str2double(line{k});
        end
    end
end

% % 추가 디버깅 출력
% fprintf('Debug - Read breakline for Pot: %s, Frag: %d\n', pot_id, frag_id);
% for i = 1:num_segments
%     fprintf('Segment %d: %s\n', i, mat2str(breakline{i}));
% end

fclose(fid);
end