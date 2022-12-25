function [DHtable,T_i_b] = DH_generator(l,q)

DHtable = [-pi/2 7 0  q(1) ;
           0  5  0 q(2)];

% T_i_b = transformation from the INERTIA frame to the BASE of the manipulator
dx=2;
T_i_b= DHMatrix([pi/2,dx,dx,0]);   

end