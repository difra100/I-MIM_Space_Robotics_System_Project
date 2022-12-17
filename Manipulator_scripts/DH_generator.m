function [DHtable,T_i_b] = DH_generator(l,q)
% T = transformation from the inertia frame to the base of the manipulator
DHtable = [-pi/2 7 0  q(1) ;
           -pi/2  5  0 q(2)];

dx=2;
T_i_b = DHMatrix([pi/2,dx,dx,pi/2]);
end