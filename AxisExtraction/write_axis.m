function write_axis(vt, pot_id, frag_id, extended)

if nargin < 4, extended = false;
end
% 
% root_dir = load_root_dir('D:/240718/Data_change/H(re)', extended)
root_dir = load_root_dir('D:/SFS_BB_temp/Plt_A', extended);
writematrix(vt([4 5 6 1 2 3],:)', ...
            sprintf('%s/Axes/Plt_%s_Piece_%02d_Axis.xyz', ...
            root_dir, pot_id, frag_id), 'Delimiter', ' ', 'FileType', 'text'); 

end
