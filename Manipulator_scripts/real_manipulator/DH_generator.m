function [DHtable,T_lvlh_b] = DH_generator(l, q, L_s)
    
    DHtable = [pi/2 0 l(1)  q(1) ;
               0  l(2)  0 q(2)];

    T_lvlh_b = rotm2tform(elem_rot_mat('y', pi/2))*...
                rotm2tform(elem_rot_mat('z', pi))*...
                trvec2tform([0,0,L_s]);


%
%     %inverting
%     T_lvlh_b(1:3,1:3) = T_lvlh_b(1:3,1:3)';
%     T_lvlh_b(1:3,4) = - T_lvlh_b(1:3,1:3)*T_lvlh_b(1:3,4);
end