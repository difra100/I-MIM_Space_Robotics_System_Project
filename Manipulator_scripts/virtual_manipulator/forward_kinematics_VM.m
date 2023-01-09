function [T_0N, R_0N, p,T_0N_2,T_0N_1] = forward_kinematics_VM(DHTABLE)
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
T = eye(4);

for i = 1:N
    line = DHTABLE(i, :);
    R = [cos(line(4)) -cos(line(1))*sin(line(4)) sin(line(1))*sin(line(4)) line(2)*cos(line(4));
         sin(line(4)) cos(line(1))*cos(line(4)) -sin(line(1))*cos(line(4)) line(2)*sin(line(4));
         0 sin(line(1)) cos(line(1)) line(3);     
         0 0 0 1;];
    A{i} = R;
    T = T * R;  
    
end

%% Forward kinematics
T = eye(4); 
for i=1:N-2
    T = T*A{i};
%     T = simplify(T)
end
T_0N_2 = T      % penultimo joint position(base of the manipulator real)
T_0N_1 = T*A{N-1}; % last joint position(J1)

T_0N = T*A{N};    % final Transformation matrix (roto-translation)
R_0N = T_0N(1:3,1:3);   % finalt Rotation matrix
p = T_0N(1:3,4);       % final cartesian Position
end