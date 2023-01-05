function [trans] = get_j2000_coords(time_instant)
    obs_file = load('./data/j2000_measures.txt');
    
    x_sc = obs_file(:,2); % [km]
    y_sc = obs_file(:,3); % [km]
    z_sc = obs_file(:,4); % [km]
    v_x_sc = obs_file(:,5); % [km/s]
    v_y_sc = obs_file(:,6); % [km/s]
    v_z_sc = obs_file(:,7); % [km/s]
    
    i = time_instant;

    r_s=[x_sc(i); y_sc(i); z_sc(i)]
    v_s=[v_x_sc(i); v_y_sc(i); v_z_sc(i)];
    
    z_LVLH=-r_s/norm(r_s);
    y_LVLH=cross(z_LVLH,v_s)/norm(cross(z_LVLH,v_s));
    x_LVLH=cross(y_LVLH,z_LVLH)/norm(cross(y_LVLH,z_LVLH));
    
    trans=[x_LVLH' x_sc(i)*1000; y_LVLH' y_sc(i)*1000; z_LVLH' z_sc(i)*1000; 0 0 0 1]; % *1000 to get meters
end