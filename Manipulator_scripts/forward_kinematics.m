function [T_0N, R_0N, p] = forward_kinematics(DHTABLE,T_i_b)
% INPUTS:
% - DH table = [alpha_i a_i d_i theta_i]           

         
%% Build the general Denavit-Hartenberg trasformation matrix
syms alpha_i a_i d_i theta_i real
TDH = [ cos(theta_i) -sin(theta_i)*cos(alpha_i)  sin(theta_i)*sin(alpha_i) a_i*cos(theta_i);
        sin(theta_i)  cos(theta_i)*cos(alpha_i) -cos(theta_i)*sin(alpha_i) a_i*sin(theta_i);
          0             sin(alpha_i)             cos(alpha_i)            d_i;
          0               0                      0                   1];

%% Build transformation matrices for each link
[N,~] = size(DHTABLE);
A = cell(1,N);

% For every row in 'DHTABLE', substitute the right value from the general
% DH matrix
for i = 1:N
    alpha_i = DHTABLE(i,1);
    a_i = DHTABLE(i,2);
    d_i = DHTABLE(i,3);
    theta_i = DHTABLE(i,4);
    A_i = subs(TDH);
    A{i} = A_i;
end

%% Forward kinematics
T = eye(4); 
for i=1:N
    T = T*A{i};
    T = simplify(T);
end

T_0N = T_i_b*T;    % final Transformation matrix (roto-translation)
R_0N = T_0N(1:3,1:3);   % finalt Rotation matrix
p = T_0N(1:3,4);       % final cartesian Position

