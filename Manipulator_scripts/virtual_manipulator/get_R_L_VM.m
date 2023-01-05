function [c_2_j2000, R_, L_, trans] = get_R_L_VM(DHTABLE, theta_v, T_lvlh_b, d1, d2, l1)
       % This function computes the R, L collections of vectors for the
       % virtual manipulator modelling.
       % INPUTs: DHTable is the Denavit-Hartenberg table, for the direct
       % kinematics, theta_v: spacecraft's attitude, T_lvlh_b: lvlh ---> base frame, d1/d2,l1: Robot dimensions.
       % OUTPUTs: center of mass c_2_j2000, R_, L_ expressed in the j2000
       % reference frame. trans is the homogeneous transformation of the
       % direct kinematic, it is requested for the optimization proble to
       % get the tip position.

       [trans, ~, p_EE] = forward_kinematics(DHTABLE, T_lvlh_b);
       time_instant = 1; % First instant to compute the virtual ground position 

       T_Lvlh_j2000 = get_attitude(time_instant); % Get Satellite's attitude, (from LVLH to j2000) 
        
       c_2_b = d2;  % Center of mass of the second joint (c_N, where N = 2)
        
        
       T_b_lvlh = T_lvlh_b*[rpy_rotation('xyz', theta_v')', [0;0;0]; 0 0 0 1];
     
        
       T_b_j2000 = T_Lvlh_j2000*T_b_lvlh;  % base to j2000
        
       R_0_j2000 = T_b_j2000*[0;0;0;1];  % manipulator base position, in the satellite reference frame
       R_0_j2000 = R_0_j2000(1:3);
        
       c_2_j2000 = T_b_j2000*[c_2_b;1];
       c_2_j2000 = c_2_j2000(1:3);
        
       p1_b = [0;0;l1];
        % 
       d1_j2000 = T_b_j2000*[d1;1];
       d1_j2000 = d1_j2000(1:3);
        
       d2_j2000 = T_b_j2000*[d2;1];
       d2_j2000 = d2_j2000(1:3);
        
       p1_j2000 = T_b_j2000*[p1_b;1];
       p1_j2000 = p1_j2000(1:3);
        
       p_EE_j2000 = T_b_j2000*[p_EE;1];
       p_EE_j2000 = p_EE_j2000(1:3);
        
       L_ = [d1_j2000'; (d2_j2000- p1_j2000)'; [0;0;0]'];
       R_ = [R_0_j2000';(p1_j2000 - d1_j2000)'; (p_EE_j2000-d2_j2000)'];
end
        