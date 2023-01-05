function [DHtable,T_lvlh_b, T_EE] = DH_generator_VM(l, q, theta_v, flagVM, m, m_s)
    
    DHtable = [pi/2 0 l(1)  q(1) ;
               0  l(2)  0 q(2)];
    dx=1;  % cube satellite diagonal.

    T_lvlh_b = rotm2tform(elem_rot_mat('y', pi/2)*elem_rot_mat('z', pi/4))*trvec2tform([0,0,dx/2]);
    if flagVM
        T_lvlh_b = T_lvlh_b*[rpy_rotation('xyz', theta_v'), [0;0;0]; 0 0 0 1];
        
        
        d1 = [0; 0; l(1)/2];
        d2 = [(l(2)/2)*cos(q(1))*cos(q(2)); (l(2)/2)*sin(q(1))*cos(q(2));l(1)+ (l(2)/2)*sin(q(1))]; 
        [c_2_j2000, R_, L_, T_EE] = get_R_L_VM(DHtable, theta_v, T_lvlh_b, d1, d2, l(1));
            
        vg = virtual_ground(c_2_j2000, R_, L_, m, m_s);
        [DHtable, ~] = virtual_manipulator(vg, R_, L_, m, m_s); % Here DHtable is the virtual manipulator forward kinematics.
      
    end

end