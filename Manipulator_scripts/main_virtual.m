%% Virtual Manipulator Section 
clear all
clc

%% INSTANTIATIONS
init;

verbosity = 2;
%Verbosity gives what is printed/shown:
% - 0: Nothing 
% - 1: kinematic/dynamics components (only)
% - 2: plots (only)
% - 3: (1) and (2)
fprintf('Verbosity level: %i\n\n',verbosity);

%% CREATING THE ROBOT

% Get the original DHtable and modifying the mapping from LVLH to the base 
% frame in order to include the new Dofs from the spaceCraft
[DHTABLE, T_lvlh_b_real] = DH_generator(l,q);
T_lvlh_b = T_lvlh_b_real*[rpy_rotation('xyz', theta_v'), [0;0;0]; 0 0 0 1];

 time_instant = 1; % First instant to compute the virtual ground position 
 T_lvlh_j2000 = get_attitude(time_instant); % Get Satellite's attitude, (from LVLH to j2000) 

%% KINEMATICS
[p_EE,p_tip] = forward_kinematics_VM(d, m, m_s,T_lvlh_b,T_lvlh_j2000,DHTABLE);


J_a = analitic_jacobian(p_EE,q');

[J_L,J_A] = geometric_jacobian(DHTABLE,q',['r','r']);
J_g = [J_L;J_A] ;


if (verbosity == 1 || verbosity == 3)
    fprintf('\n\n---------------------- Kinematics ---------------------')
    fprintf('\n - Forward kinematics:\n')
    fprintf('   %s\n',vpa(p_EE,2))
    fprintf('\n - Analitic Jacobian:\n')
    fprintf('   %s %s \n',vpa(J_a,2))
    fprintf('\n - Geometric Jacobian:\n')
    fprintf('   %s %s\n',vpa(J_g,2))
end

%% IMPORTANT TRANSFORMATIONS

q_i = [0,0,0,0,0]';


[T_EE_real,~,~] = forward_kinematics(DHTABLE,T_lvlh_b);

T_EE_real = T_lvlh_j2000*T_EE_real;
p_EE_j2000 = T_EE_real(1:3,4);
T_tip_real = T_lvlh_j2000*T_EE_real*trvec2tform([1,0,0]);
p_tip_j2000 = T_tip_real(1:3,4);

fprintf(' Virtual E-E position: %f \n ',vpa(norm(subs(p_tip, Q_augm, q_i)), 6))

fprintf(' Real E-E position: %f \n',vpa(norm(subs(p_tip_j2000, Q_augm, q_i)), 6))

mars_target = get_targets(orbit_ts, 1);
earth_target = get_targets(orbit_ts, 0);

% To cumulate points
q_ss = [];
dq_ss = [];
ddq_ss = [];
pointss = [];
tot_time = 0;


[M, V, B, C] = dynamic_model(q, dq, ddq, m, l, d, I1,I2);

disp(' Mars pointing trajectory ')
[q_f, q_ss, dq_ss, ddq_ss, pointss, tot_time] = get_trajectory_in_orbit_VM(q_i, q_ss, dq_ss, ddq_ss, pointss, tot_time, mars_target, M, ni, I_m, B_m, tau_max, p_EE, theta_bounds, l, m, m_s);

q_ss = q_ss(1:end-1, :)
dq_ss = dq_ss(1:end-1, :)
ddq_ss = ddq_ss(1:end-1, :)

disp(' Earth pointing trajectory ')
[q_f, q_ss, dq_ss, ddq_ss, pointss, tot_time] = get_trajectory_in_orbit_VM(q_f, q_ss, dq_ss, ddq_ss, pointss, tot_time, earth_target, M, ni, I_m, B_m, tau_max, p_EE, theta_bounds, l, m, m_s);











