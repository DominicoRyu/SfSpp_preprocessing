function visualize_fragment(pot_id, frag_id, extended, best_only)

if nargin < 3, extended = true;
end

if nargin < 4, best_only = false;
end
% 
% disp(sprintf('Z:/2020/Pottery Data/%s-%02d/Surfaces/%s-%02d-%02d_Surface_UniformSampled_0.xyz', ...
%                       namespace, pot_id, namespace, pot_id, frag_id));

[C0, C1] = read_surfaces(pot_id, frag_id, extended);
% C0 = dlmread(sprintf('Z:/2020/Pottery Data/%s-%02d/Surfaces/%s-%02d-%02d.obj_samples_Int.xyz', ...
%                       namespace, pot_id, namespace, pot_id, frag_id))';
% C1 = dlmread(sprintf('Z:/2020/Pottery Data/%s-%02d/Surfaces/%s-%02d-%02d.obj_samples_Ext.xyz', ...
%                       namespace, pot_id, namespace, pot_id, frag_id))'; 
C = [C0, C1];

vt = read_axis(pot_id, frag_id, extended);              

[breakline, isbase, isrim] = read_breakline(pot_id, frag_id, extended);
% [isrim2] = check_rim_and_base(pot_id, frag_id, extended, false);
% isrim = false(numel(breaklines), 1);

figure(frag_id);
clf;

numAxes = size(vt, 2);
if best_only, numAxes = 1;
end

if isempty(breakline), breakline = [];
else
    if sum(isrim) > 0
        breakline = {breakline{isrim}}; %#ok<CCAT1>
    end
end

for i = 1 : numAxes
    visualize_axis_of_symmetry_two(C0, C1, breakline, [vt(4:6,i); vt(1:3,i)]);
end

end

