% I-MIM project - manipulator main
clear all

%% INSTANTIATIONS
init;

verbosity = 3;
%Verbosity gives what is printed/shown:
% - 0: Nothing 
% - 1: kinematic/dynamics components (only)
% - 2: plots (only)
% - 3: (1) and (2)
fprintf('\nVerbosity level: %i\n',verbosity);

%% CREATING THE ROBOT

% DH table [alpha_i d_i a_i theta_i] (original method)

[DHTABLE,T_lvlh_b]= DH_generator(l,q, L_s);

q_i =[0;0];
robot = create_robot(l,q_i, L_s);


%% KINEMATICS
[T_EE,R_EE,p_EE] = forward_kinematics(DHTABLE,T_lvlh_b);
T_tip = T_EE*trvec2tform([1,0,0]);
p_tip = T_tip(1:3,4);

J_a = analitic_jacobian(p_EE,q');

[J_L,J_A] = geometric_jacobian(DHTABLE,q',['r','r']);
J_g = [J_L;J_A] ;


if (verbosity == 1 || verbosity == 3)
    fprintf('\n\n---------------------- Kinematics ---------------------\n')
    fprintf('\n - Forward kinematics:\n')
    fprintf('   %s\n',vpa(p_EE,2))
    fprintf('\n - Analitic Jacobian:\n')
    fprintf('   %s %s \n',vpa(J_a,2))
    fprintf('\n - Geometric Jacobian:\n')
    fprintf('   %s %s\n',vpa(J_g,2))
    fprintf('\n--------------------------------------------------------\n')

end



%% IMPORTANT TRANSFORMATIONS

% T_EE:  BASE frame -> EE frame
% T_lvlh_b : LVLH frame -> BASE frame

% dx=2;
% T_lvlh_b= DHMatrix([-pi/2,dx,dx,0]); % Inertia frame -> base frame

fprintf('\n----> To get to the Dynamic components press inv \n')
pause()
%% DYNAMIC COMPONENTS

[M, V, B, C] = dynamic_model(q, dq, ddq, m, l, d, I1,I2);
[M_true, V_true, B_true, C_true] = dynamic_model(q, dq, ddq, m_true, l_true, d_true, I1_true,I2_true);

if (verbosity == 1 || verbosity == 3)
    fprintf('\n\n------------------------ Dynamic componets ------------------------\n')
    fprintf('\n - Inertia matrix:\n')
    fprintf('   %s %s \n',vpa(M,3))
    fprintf('\n - Coreolis and Centrifugal term:\n')
    fprintf('   %s \n',vpa(V,3))
    fprintf('\n - Coreolis term:\n')    
    fprintf('   %s\n',vpa(B,3))
    fprintf('\n - Centrifugal term:\n')
    fprintf('   %s %s \n',vpa(C,3))
    fprintf('\n--------------------------------------------------------\n')
end

tau_c= [(1 + sign(dq(1)))/2 *tauC_plus + (1 - sign(dq(1)))/2 *tauC_minus;
        (1 + sign(dq(2)))/2 *tauC_plus + (1 - sign(dq(2)))/2 *tauC_minus];



%% DYNAMIC MODEL

% syms tau1 tau2 tau3 tau1_c tau2_c real
% 
% 
% 
% conservative_comp =  M*ddq + V;
% non_conservative_comp = tau_c + J_L'*atm_drag;
% 
% disp('------------------------ Final dynamic model -------------------\n')
% tau = conservative_comp - non_conservative_comp;
% disp('tau:')
% disp(vpa(tau,2))
fprintf('\n----> To get to the Trajectories press inv \n')
pause()
% TRAJECTORIES
orbit_ts = minutes*6; % Each timestep has 10 seconds of distance.





mars_target_cell = get_targets(0, orbit_ts, 1);% Extracting the first #orbit_ts points of mars position  (ts: time samples)
earth_target_cell = get_targets(orbit_ts+1, orbit_ts, 0);% Extracting the first #orbit_ts points of earth position (ts: time samples)
mars_target = [];
earth_target = [];

R_of_nadir = rotm2tform(elem_rot_mat('y',deg2rad(15)));
for i=1:orbit_ts
    mars_target_i = R_of_nadir*[cell2mat(mars_target_cell(i));1];
    mars_target(:,end+1) = mars_target_i(1:3);
    earth_target_i = [cell2mat(earth_target_cell(i));1];
    earth_target(:,end+1) = earth_target_i(1:3);
end


% To cumulate points
q_ss = q_i';
dq_ss = [0;0]';
ddq_ss = [0;0]';
pointss = [];
tot_time = 0;



% Get full trajectory
% input : q_i q_ss, dq_ss, ddq_ss, tot_time, target_poss
% output : q_f q_ss, dq_ss, ddq_ss, tot_time


fprintf('\n********** Mars pointing trajectory *************\n')
[q_f, q_ss, dq_ss, ddq_ss, pointss, tot_time] = get_trajectory_in_orbit(q,dq, t, ...
    q_i, q_ss, dq_ss, ddq_ss, pointss, tot_time, mars_target, M, p_EE, L_s, ...
    ni, I_m, B_m, tau_max,tau_c, theta_bounds, l, sampling_rate);

fprintf('\n********* Earth pointing trajectory *************\n')
[q_f, q_ss, dq_ss, ddq_ss, pointss, tot_time] = get_trajectory_in_orbit(q,dq, t, ...
    q_f, q_ss, dq_ss, ddq_ss, pointss, tot_time, earth_target, M, p_EE, L_s, ...
    ni, I_m, B_m, tau_max,tau_c, theta_bounds, l, sampling_rate);

if (verbosity == 2 || verbosity == 3)
    plot_robot_traj(robot, q_ss, pointss, earth_target(:,end),q,p_EE,p_tip);
end

fprintf('\n----> To get to the Control part press inv \n')
pause()
discr_interval = 1/sampling_rate;
timesteps = (0:discr_interval:tot_time)';

% if (verbosity == 2 || verbosity == 3)
%         figure
%         plot(timesteps,q_ss(:,1),'r',timesteps,q_ss(:,2),'b')
%         title('Configurations of trajectory',i)
%         
%         figure
%         plot(timesteps,dqss(:,1),'r',timesteps,dq_ss(:,2),'b')
%         title('Velocities of trajectory',i)
%         
%         figure
%         plot(timesteps,ddqss(:,1),'r',timesteps,dd_qss(:,2),'b')
%         title('Accelerations of trajectory',i)
% end




%% CONTROLLER

% NonLinear controller
% You receive a trajectory (traj_q, traj_dq, traj_ddq) in discrete time.
% For each timestep you get in the configuration: M, V, tau_c, jacobian.
% Then, you compute the input tau using the traj_ddq as command in acceleration.
% Continuing, you do the rest of the stuff by integrating the error bla bla
% bla.

space_craft_velocity_base = T_lvlh_b*[spacecraft_velocity;1];  % Spacecraft velocity expressed in the base frame 

atm_drag_fixed = (1/2) * drag_coeff * mars_density * Area_Antenna;        % without velocity = 1/2 * C_p * density * A




timesteps = (0:discr_interval:tot_time)'; 
count_steps = length(timesteps);

commads = zeros(count_steps,2);
taus = zeros(count_steps,2);

error = zeros(count_steps,4);
q_0 = zeros(count_steps,2);
dq_0 = zeros(count_steps,2);

% first conf/velocity is 'distrurbed'
q_0(1,:)=q_ss(1,:)-randn(1,2)*10*pi/180;
dq_0(1,:)=dq_ss(1,:)-randn(1,2)*pi/180;

q_d = q_ss;
dq_d = dq_ss;
ddq_d = ddq_ss;
trajectory = pointss;


options = odeset('RelTol', 1.0E-4, 'AbsTol', 1.0E-4);
dips('Integration steps:')
for i = 1:count_steps-1
    fprintf(' - Step %i \n',i)
    %Build the controller part in ored to compute the effective acc command...
    
    % Get the value of the errors from integration (as done in IDA_control):
    %           e = (qi-q), de = (dqi-dq) 
    % and then compute the command in acceleration
    %           command = ddqi + k_d*de + k_p*e

    % Finally, get the command in torque using the dynamic model
    % beta = V_i + tau_c;
    % tau = M_i*command + beta - J_i'*atm_drag;

    e=q_d(i,:)'- q_0(i,:)';
    de = dq_d(i,:)'- dq_0(i,:)';

    error(i,:) = [e;de];
    tspan = [timesteps(i) timesteps(i+1)];
    
    ee_vel = J_L*dq_d(i,:)';
    tot_vel = ee_vel + space_craft_velocity_base(1:3,:);

    atm_drag = atm_drag_fixed*(tot_vel.^2);

    atm_drag_true = atm_drag + [0;randn(1)*10^-2;0];

    [t_int,error_int] = ode113(@(time,error)controller(time, error, q_d, dq_d, ddq_d, ...
                            q,dq, timesteps, k_p, k_d, ...
                            M, V,tau_c,J_L,atm_drag, ...
                            M_true,V_true,atm_drag_true, i), ...
                            tspan, error(i,:)', options);
    


    row=length(error_int);

    q_0(i+1,:)=q_d(i+1,:)-error_int(row,1:2);
    dq_0(i+1,:)=dq_d(i+1,:)-error_int(row,3:4);

end


if (verbosity == 2 || verbosity == 3) 
    figure
    plot(timesteps,q_0(:,1),'r',timesteps,q_d(:,1),'r--', ...
         timesteps,q_0(:,2),'b',timesteps,q_d(:,2),'b--')
    title('Trajectory traking - Q')

    figure
    plot(timesteps,dq_0(:,1),'r',timesteps,dq_d(:,1),'r--', ...
         timesteps,dq_0(:,2),'b',timesteps,dq_d(:,2),'b--')
    title('Trajectory traking - dQ')
    
    figure
    plot(timesteps,error(:,1),'r',timesteps,error(:,2))
    title('Errors - Q')

    figure
    plot(timesteps,error(:,3),'r',timesteps,error(:,4))
    title('Errors - dQ')
end

fprintf('To plot the Robot Motion press inv \n')
pause()
if (verbosity == 2 || verbosity == 3)
    plot_robot_traj(robot, q_0, trajectory, earth_target(:,end),q,p_EE,p_tip)
end



