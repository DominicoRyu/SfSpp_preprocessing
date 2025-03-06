function [surfaces_inverted, normals_inverted] = check_surface_inversion_v2(pot_id, frag_id, extended, save_files)

if nargin < 3, extended = false; end
if nargin < 4, save_files = false; end

% root_dir = load_root_dir([], extended);
%root_dir = load_root_dir('D:/trial4/Breaklines_rearr', extended);

vt = read_axis(pot_id, frag_id, extended);
[breakline0, isbase0, isrim0] = read_breakline(pot_id, frag_id, extended, false);
if isempty(breakline0)
    fprintf('No breakline data available. Check file or processing errors.\n');
    return;
end
C0 = horzcat(breakline0{:});
[breakline1, isbase1, isrim1] = read_breakline(pot_id, frag_id, extended, true);
if isempty(breakline1)
    fprintf('No breakline data available. Check file or processing errors.\n');
    return;
end
C1 = horzcat(breakline1{:});

%% 
if abs(vt(4:6,1)' * [0 0 1]' - 1) < 1e-9
    R = eye(3);
else
    R = [null(vt(4:6,1)'), vt(4:6,1)];
    if det(R) < 0.0, R(:,3) = -R(:,3);
    end
end

% Take the first one as the reference vector
R = R';
X_transformed_0 = R * bsxfun(@minus, C0(1:3,:), vt(1:3,1));
X_transformed_1 = R * bsxfun(@minus, C1(1:3,:), vt(1:3,1));

z0 = X_transformed_0(3,:);
z1 = X_transformed_1(3,:);

zmax = min(max(z0), max(z1));
zmin = max(min(z0), min(z1));

%% compute distances and determine whether we need to invert
d0 = compute_l2p_distance([vt(4:6,1); vt(1:3,1)], C0(1:3,z0 > zmin & z0 < zmax));
d1 = compute_l2p_distance([vt(4:6,1); vt(1:3,1)], C1(1:3,z1 > zmin & z1 < zmax));

% if d0 or d1 are empty, then there's no overlap between the interior and
% exterior surfaces. Take the one with greater (richer) breakline variation
% by checking the depth length.
if isempty(d0) || isempty(d1)
    zrange0 = max(z0) - min(z0);
    zrange1 = max(z1) - min(z1);
    surfaces_inverted = zrange0 < zrange1;
else
    surfaces_inverted = (mean(d0) > mean(d1));
end

normals_inverted = false;  % 법선 방향 뒤집기 삭제

if save_files && surfaces_inverted
    [C0, C1] = read_surfaces(pot_id, frag_id, extended);

    write_surfaces(C1, C0, pot_id, frag_id, extended);
    write_breakline(breakline1, isbase1, isrim1, pot_id, frag_id, extended, false);
    write_breakline(breakline0, isbase0, isrim0, pot_id, frag_id, extended, true);
end

fprintf('Pot %s frag. %02d extended %d surf. inverted: %d\n', pot_id, frag_id, extended, surfaces_inverted);

end
