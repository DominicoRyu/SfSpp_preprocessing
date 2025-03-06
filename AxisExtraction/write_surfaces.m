function write_surfaces(C0, C1, pot_id, frag_id, extended)

if nargin < 5, extended = false;
end


% root_dir = load_root_dir('D:/240718/Data_change/H(re)', extended);
root_dir = load_root_dir('D:/SFS_BB_temp/Plt_A', extended);

writematrix(C0', sprintf('%s/Surfaces/Plt_%s_Piece_%02d_Surface_0.xyz', ...
                root_dir, pot_id, frag_id), 'FileType', 'text', 'Delimiter', ' ');
                  
if ~isempty(C1)
    writematrix(C1', sprintf('%s/Surfaces/Plt_%s_Piece_%02d_Surface_1.xyz', ...
                root_dir, pot_id, frag_id), 'FileType', 'text', 'Delimiter', ' ');
end

end
