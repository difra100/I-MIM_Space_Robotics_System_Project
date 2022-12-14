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


DHtable = DH_generator_VM(L_s/2,l, Q_augm);

time_instant = 1; % First instant to compute the virtual ground position 
T_lvlh_j2000 = get_attitude(time_instant); % Get Satellite's attitude, (from LVLH to j2000) 



%% KINEMATICS
DHtable_0 = subs(DHtable,theta_v,[0,0,0]');
[T_EE_lvlh, ~, p_EE_lvlh,T_lvlh_b,T_j1_lvlh] = forward_kinematics_VM(DHtable_0);
% T_j1_lvlh = T_lvlh_b*DHtransf(DHtable(1,:));
T_tip_lvlh = T_EE_lvlh*trvec2tform([1,0,0]);
p_j0_lvlh = T_lvlh_b(1:3,4);
p_j1_lvlh = T_j1_lvlh(1:3,4);
p_tip_lvlh = T_tip_lvlh(1:3,4);


[c_2_j2000, R_, L_] = get_R_L_VM(T_lvlh_b, T_lvlh_j2000, d, p_j0_lvlh,p_j1_lvlh,p_EE_lvlh,p_tip_lvlh);
    
vg = virtual_ground(c_2_j2000, R_, L_, m, m_s);

[~,r_,l_] = virtual_manipulator(vg, R_, L_, m, m_s); 

L0 = norm(r_(1));
L1 = norm(r_(2)+l_(2));
L2 = norm(r_(3)+l_(3));

% Now there you can see the final Denavit H table 
% of the virtual manipulator!
DHtable_VM = DH_generator_VM(L0,[L1,L2], Q_augm);

[T_VM, R_VM, p_VM,~,~] = forward_kinematics_VM(DHtable_VM);



%% IMPORTANT TRANSFORMATIONS

q_i = [0,0,0,0,0]';

% 
% [T_EE_real,~,~,~,~] = forward_kinematics(DHtable,T_lvlh_b);
% 
% T_EE_real = T_lvlh_j2000*T_EE_real;
% p_EE_j2000 = T_EE_real(1:3,4);
% T_tip_real = T_lvlh_j2000*T_EE_real*trvec2tform([1,0,0]);
% p_tip_j2000 = T_tip_real(1:3,4);
% 
% fprintf(' Virtual E-E position: %f \n ',vpa(norm(subs(p_VM, Q_augm, q_i)), 6))
% 
% fprintf(' Real E-E position: %f \n',vpa(norm(subs(p_EE_j2000, Q_augm, q_i)), 6))

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
[q_f, q_ss, dq_ss, ddq_ss, pointss, tot_time] = get_trajectory_in_orbit_VM( ...
    q_i, q_ss, dq_ss, ddq_ss, pointss, tot_time, ...
    mars_target, M, ni, I_m, B_m, tau_max, p_VM, ...
    theta_bounds, L0,[L1,L2]);

q_ss = q_ss(1:end-1, :);
dq_ss = dq_ss(1:end-1, :);
ddq_ss = ddq_ss(1:end-1, :);

disp(' Earth pointing trajectory ')
[q_f, q_ss, dq_ss, ddq_ss, pointss, tot_time] = get_trajectory_in_orbit_VM( ...
    q_i, q_ss, dq_ss, ddq_ss, pointss, tot_time, ...
    earth_target, M, ni, I_m, B_m, tau_max, p_VM, ...
    theta_bounds, L0,[L1,L2]);











