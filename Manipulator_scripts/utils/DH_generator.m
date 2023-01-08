function [DHtable,T_lvlh_b] = DH_generator(l, q)
    
    DHtable = [pi/2 0 l(1)  q(1) ;
               0  l(2)  0 q(2)];
    dx=1;  % cube satellite diagonal.

    T_lvlh_b = rotm2tform(elem_rot_mat('y', pi/2)*elem_rot_mat('z', pi/4))*trvec2tform([0,0,dx/2]);
end