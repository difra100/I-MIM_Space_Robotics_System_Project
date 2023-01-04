function [DHtable,T_lvlh_b] = DH_generator(l,q)% flagVM == false)
    
%     DHtable = [-pi/2 7 0  q(1) ;
%                0  5  0 q(2)];
    DHtable = [pi/2 0 l(1)  q(1) ;
               -pi/2  l(2)  0 q(2)+pi/2];
    
%     if flagVM 
%         
%         s_DHtable = [   ;
%                         ;
%                         ;]
%     
%     
%     
%     else
%     
% T_lvlh_b = transformation from the LVLH frame to the BASE of the manipulator
    dx=2;
    T_lvlh_b = rotm2tform(elem_rot_mat('x', -pi/2)*elem_rot_mat('z', pi/4))*trvec2tform([0,0,dx/2]);  % DHMatrix([-pi/2,dx,dx,0]);   
    %end

end