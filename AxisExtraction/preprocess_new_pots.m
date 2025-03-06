pot_ids = {'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J'};
num_fragments = [8, 9, 6, 32, 34, 7, 7, 11, 30, 19];
num_fragments = containers.Map(pot_ids, num_fragments);


tested_pot_ids = {'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J'};
% tested_pot_ids = {'18-0702-08', '19-0820-01', '19-0820-02', '19-0820-03', '20-0610-05', '20-0610-08'};
% tested_pot_ids = {'20-0610-08'};

extract_axes = true;
check_for_inv_surfaces = false;
assign_base_and_rim = false;
visualize_surfaces = false;


%% Extract axes
if extract_axes
    for pot_id = tested_pot_ids
        pot_id = pot_id{:};
        for frag_id = 1 : num_fragments(pot_id)
            vt = extract_axis(pot_id, frag_id, false);
            write_axis(vt, pot_id, frag_id, false);
            write_axis(vt, pot_id, frag_id, true);      
        end
    end
end

%% Check inversion for non-extended surfaces
if check_for_inv_surfaces
    for pot_id = tested_pot_ids
        pot_id = pot_id{:};
        for frag_id = 1 : num_fragments(pot_id)
            check_surface_inversion_v2(pot_id, frag_id, false, true);
            check_surface_inversion_v2(pot_id, frag_id, true, true);
        end
    end
end

%% Check inversion for non-extended surfaces
if assign_base_and_rim
    extended = false;
    overwrite = true;
    
    for pot_id = tested_pot_ids
        pot_id = pot_id{:};
        base_idx = nan(num_fragments(pot_id), 1);
        rim_idx = nan(num_fragments(pot_id), 1);
        for frag_id = 1 : num_fragments(pot_id)
            [breakline, isbase_prev, isrim_prev] = read_breakline(pot_id, frag_id, extended, false);
            [isbase, isrim] = check_base_and_rim(pot_id, frag_id, extended, false);
            base_idx(frag_id) = isbase;
            rim_idx(frag_id) = sum(isrim) > 0;

            if overwrite
                if isbase~=isbase_prev || ~isequal(isrim, isrim_prev)
                    % rewrite breaklines with modified attributes for base and                
                    write_breakline(breakline, isbase, isrim, pot_id, frag_id, extended, false);                
                end
            end
%             check_surface_inversion(pot_id, frag_id, true, true);
        end
        fprintf('---[Pot %s]---\n', pot_id);
        disp(find(base_idx)');
        disp(find(rim_idx)');

    end
end

%% Visualize fragments
if visualize_surfaces
    for pot_id = tested_pot_ids
        pot_id = pot_id{:};
        for frag_id = 1 : num_fragments(pot_id)
            visualize_fragment(pot_id, frag_id, true);
        end
    end
end