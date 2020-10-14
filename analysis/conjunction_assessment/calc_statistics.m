function [planar_time,...
          planar_distance,...
          planar_view_angle,...
          one_deg_time,...
          one_deg_distance,...
          conj_angle] = calc_statistics(ref_orbit, ref_t, conj_orbit, rad, T)
% calculates planar visibility statistics between to conjunctive orbits.  
    rel_pos = ref_orbit(2:end, 1:3) - conj_orbit(2:end, 1:3);

    rel_pos_unit = rel_pos ./ vecnorm(rel_pos, 2, 2);

    visible = check_intersections(...
        ref_orbit(2:end, 1:3),...
        conj_orbit(2:end, 1:3),...
        rel_pos_unit,...
        rad);

    dots = abs(dot(rel_pos_unit, rel_pos_unit(1,:).*ones(size(rel_pos_unit)), 2));
    visible_dots = (dots > cosd(1/60)) & visible;
    ind = find(visible_dots, 1, 'last');
    
    
    planar_time = T/4 - ref_t(ind+1);
    if isempty(ind)
        planar_time = -1;
    end
    planar_distance = norm(rel_pos(ind, :));
    planar_view_angle = 2*atand(0.5/(1000*planar_distance));

    rel_pos_mag = vecnorm(rel_pos, 2, 2);
    view_angles = 2*atand(0.5./(1000*rel_pos_mag));
    one_deg_ind = find(view_angles < 1/60, 1);
    one_deg_time = T/4 - ref_t(one_deg_ind+1);
    one_deg_distance = rel_pos_mag(one_deg_ind);
    
    ref_path_vec = ref_orbit(2, 1:3) - ref_orbit(1, 1:3);
    ref_path_unit = ref_path_vec / norm(ref_path_vec);
    conj_path_vec = conj_orbit(2, 1:3) - conj_orbit(1, 1:3);
    conj_path_unit = conj_path_vec / norm(conj_path_vec);
    conj_angle = acosd(dot(ref_path_unit, conj_path_unit));
end