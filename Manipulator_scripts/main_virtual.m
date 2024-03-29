%% Virtual Manipulator Section 
clear all
clc

%% INSTANTIATIONS
init; % Where all the instantiation are made.

verbosity = 3;
%Verbosity gives what is printed/shown:
% - 0: Nothing 
% - 1: kinematic/dynamics components (only)
% - 2: plots (only)
% - 3: (1) and (2)
fprintf('Verbosity level: %i\n\n',verbosity);

%% CREATING THE ROBOT

% Get the original DHtable and modifying the mapping from LVLH to the base 
% frame in order to include the new Dofs from the spaceCraft

orbit_ts = minutes*6; % Conversion factor to get the number of rows (10 secs for each) from the planet positions dataset.
DHtable = DH_generator_VM(L_s/2,l, Q_augm); % Q_augm contains also the satellite's attitude angles. Note that they do not represent the satellite's attitude w.r.t. J2000 (LVLH ---> J2000), but it is the attitude w.r.t. LVLH. The assumption is that the robot change its orientation w.r.t. LVLH.

time_instant = 1; % First instant to compute the virtual ground position 
T_lvlh_j2000 = get_attitude(time_instant); % Get Satellite's attitude, (from LVLH to j2000) 



%% KINEMATICS
DHtable_0 = subs(DHtable,theta_v,[0,0,0]');
[T_EE_lvlh, ~, p_EE_lvlh,T_lvlh_b,T_j1_lvlh] = forward_kinematics_VM(DHtable_0);

T_tip_lvlh = T_EE_lvlh*trvec2tform([1,0,0]);
p_j0_lvlh = T_lvlh_b(1:3,4);
p_j1_lvlh = T_j1_lvlh(1:3,4);
p_tip_lvlh = T_tip_lvlh(1:3,4);


[c_2_j2000, R_, L_] = get_R_L_VM(T_lvlh_b, T_lvlh_j2000, d, p_j0_lvlh,p_j1_lvlh,p_EE_lvlh,p_tip_lvlh);
    
vg = virtual_ground(c_2_j2000, R_, L_, m, m_s);

[~,r_,l_] = virtual_manipulator(vg, R_, L_, m, m_s); 

% Re-define the Link length of the robot, in the virtaul manipulator case
% these are defined by the r, and l vectors.
L0 = norm(r_(1));
L1 = norm(r_(2)+l_(2));
L2 = norm(r_(3)+l_(3));

% Now there you can see the final Denavit H table 
% of the virtual manipulator!
DHtable_VM = DH_generator_VM(L0,[L1,L2], Q_augm);

[T_EE, R_EE, p_EE,~,~] = forward_kinematics_VM(DHtable_VM);
T_tip = T_EE*trvec2tform([0.01,0,0]);
p_tip = T_tip(1:3,4);


%% IMPORTANT TRANSFORMATIONS

q_i = [0,0,0,0,0]'; % Spherical joint (0,0,0) means that is the same of LVLH orientation.

robot = create_robot_VM(L0,[L1,L2],q_i);

vg_0 = double(subs(vg,Q_augm,q_i));


mars_target_cell = get_targets(1, orbit_ts, 1);
earth_target_cell = get_targets(orbit_ts+1, orbit_ts, 0);
mars_target = [];
earth_target = [];

R_of_nadir = rotm2tform(elem_rot_mat('y',deg2rad(15)));
for i=1:orbit_ts
    mars_target_i = R_of_nadir*T_lvlh_j2000*trvec2tform(vg_0')*[cell2mat(mars_target_cell(i));1];
    mars_target(:,end+1) = mars_target_i(1:3);
    earth_target_i = T_lvlh_j2000*trvec2tform(vg_0')*[cell2mat(earth_target_cell(i));1];
    earth_target(:,end+1) = earth_target_i(1:3);
end

% To cumulate points
q_ss = [];
dq_ss = [];
ddq_ss = [];
pointss = [];
tot_time = 0;



disp(' Mars pointing trajectory ')
[q_f, q_ss, dq_ss, ddq_ss, pointss, tot_time] = get_trajectory_in_orbit_VM( ...
    Q_augm, q_i, q_ss, dq_ss, ddq_ss, pointss, tot_time, ...
    mars_target, p_EE, ...
    theta_bounds, L0,[L1,L2], sampling_rate);

q_ss = q_ss(1:end-1, :);
dq_ss = dq_ss(1:end-1, :);
ddq_ss = ddq_ss(1:end-1, :);


disp(' Earth pointing trajectory ')
[q_f, q_ss, dq_ss, ddq_ss, pointss, tot_time] = get_trajectory_in_orbit_VM( ...
    Q_augm, q_f, q_ss, dq_ss, ddq_ss, pointss, tot_time, ...
    earth_target, p_EE, ...
    theta_bounds, L0,[L1,L2], sampling_rate);

if (verbosity == 2 || verbosity == 3)
    plot_robot_traj_VM(robot, q_ss, pointss, mars_target(:,end),Q_augm,p_EE,p_tip);
end

timesteps = (1:size(q_ss));

%% PLOT TRAJECTORY


if (verbosity == 2 || verbosity == 3) 
    figure()
    plot(timesteps,q_ss(:,1),'k',timesteps,q_ss(:,2),'c', ...
         timesteps,q_ss(:,3), 'm', timesteps,q_ss(:,4),'r', ...
         timesteps,q_ss(:,5), 'b')
    title('Trajectory (Q)')
    legend('Attitude 1','Attitude 2', 'Attitude 3', 'Joint 1', 'Joint 2')
    figure()
    plot(timesteps,dq_ss(:,1),'k',timesteps,dq_ss(:,2),'c', ...
         timesteps,dq_ss(:,3), 'm', timesteps,dq_ss(:,4),'r', ...
         timesteps,dq_ss(:,5), 'b')
    title('Trajectory (dQ) Velocity')
    legend('Attitude 1','Attitude 2', 'Attitude 3', 'Joint 1', 'Joint 2')
    figure()
    plot(timesteps,ddq_ss(:,1),'k',timesteps,ddq_ss(:,2),'c', ...
         timesteps,ddq_ss(:,3), 'm', timesteps,ddq_ss(:,4),'r', ...
         timesteps,ddq_ss(:,5), 'b')
    title('Trajectory (ddQ) Accelerations')
    legend('Attitude 1','Attitude 2', 'Attitude 3', 'Joint 1', 'Joint 2')
end










