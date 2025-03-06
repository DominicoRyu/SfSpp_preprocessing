function vt = read_axis(pot_id, frag_id, extended)

if nargin < 3, extended = false;
end

% 
% C0 = dlmread(sprintf('Z:/2020/Pottery Data/%s-%02d/Surfaces/%s-%02d-%02d.obj_samples_Int.xyz', ...
%                       namespace, pot_id, namespace, pot_id, frag_id))';
% C1 = dlmread(sprintf('Z:/2020/Pottery Data/%s-%02d/Surfaces/%s-%02d-%02d.obj_samples_Ext.xyz', ...
%                       namespace, pot_id, namespace, pot_id, frag_id))'; 

% root_dir = load_root_dir('D:/240718/GT', extended);
% root_dir = load_root_dir('D:/240718/Data_change/H(re)', extended);
root_dir = load_root_dir('D:/SFS_BB_temp/Plt_A', extended);
vt = readmatrix(sprintf('%s/Axes/Plt_%s_Piece_%02d_Axis.xyz', root_dir, pot_id, frag_id), 'Delimiter', ' ', 'FileType', 'text')';

end

