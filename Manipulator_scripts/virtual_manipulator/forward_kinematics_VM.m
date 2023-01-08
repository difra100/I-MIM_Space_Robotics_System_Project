function [p_EE,p_tip] = forward_kinematics_VM(d, m, m_s,T_lvlh_b,T_lvlh_j2000,DHtable)

    [T_EE_lvlh, ~, p_EE_lvlh] = forward_kinematics(DHtable, T_lvlh_b);

    T_j1_lvlh = T_lvlh_b*DHtransf(DHtable(1,:));
    T_tip_lvlh = T_EE_lvlh*trvec2tform([1,0,0]);

    p_j1_lvlh = T_j1_lvlh(1:3,4);
    p_tip_lvlh = T_tip_lvlh(1:3,4);
    
    [c_2_j2000, R_, L_] = get_R_L_VM(T_lvlh_b, T_lvlh_j2000, d, p_j1_lvlh,p_EE_lvlh,p_tip_lvlh);
        
    vg = virtual_ground(c_2_j2000, R_, L_, m, m_s);
    
    [p_EE,p_tip, ~] = virtual_manipulator(vg, R_, L_, m, m_s); % Here DHtable is the virtual manipulator forward kinematics.

end