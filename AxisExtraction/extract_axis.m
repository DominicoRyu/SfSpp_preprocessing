function vt = extract_axis(pot_id, frag_id, extended)

if nargin < 3, extended = false;
end

[C0, C1] = read_surfaces(pot_id, frag_id, extended);

vt = run_potsac(C0(:,1:1:end), C1(:,1:1:end));

end

