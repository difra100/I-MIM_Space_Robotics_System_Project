% I-MIM project - manipulator main


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

% DH table [alpha_i d_i a_i theta_i] (original method)

[DHTABLE,T_i_b]= DH_generator(l,q);

q_i =[0;0];
robot = create_robot(l,q_i);


%% KINEMATICS
[T_EE,R_EE,p_EE] = forward_kinematics(DHTABLE,T_i_b);
T_tip = T_EE*trvec2tform([1,0,0]);
p_tip = T_tip(1:3,4);

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

% T_EE:  BASE frame -> EE frame
% T_i_b : INERTIA frame -> BASE frame
% T_o_i:  LVLH frame -> INERTIA frame

R_o_i = [0 0 1;0 -1 0;1 0 0];
T_o_i = [[R_o_i,[0;0;0]];0 0 0 1]; % LVLH frame -> INERTIA frame

fprintf('To get to the Dynamic components press inv \n')
pause()
%% DYNAMIC COMPONENTS

[M, V, B, C] = dynamic_model(q, dq, ddq, m, l, d, I1,I2);
[M_true, V_true, B_true, C_true] = dynamic_model(q, dq, ddq, m_true, l_true, d_true, I1_true,I2_true);

if (verbosity == 1 || verbosity == 3)
    fprintf('\n\n------------------------ Dynamic componets ------------------------')
    fprintf('\n - Inertia matrix:\n')
    fprintf('   %s %s \n',vpa(M,3))
    fprintf('\n - Coreolis and Centrifugal term:\n')
    fprintf('   %s \n',vpa(V,3))
    fprintf('\n - Coreolis term:\n')    
    fprintf('   %s\n',vpa(B,3))
    fprintf('\n - Centrifugal term:\n')
    fprintf('   %s %s \n',vpa(C,3))
end

tau_c= [(1 + sign(dq(1)))/2 *tauC_plus + (1 - sign(dq(1)))/2 *tauC_minus;
        (1 + sign(dq(2)))/2 *tauC_plus + (1 - sign(dq(2)))/2 *tauC_minus];




%% DYNAMIC MODEL

% syms tau1 tau2 tau3 tau1_c tau2_c real
% 

% 
% % friction will depend on the angular velocity of each joint (pos o neg)
% if dq1 > 0
%     tau1_c = tau_c(1);
% else
%     tau1_c = tau_c(2);
% end
% if dq2 > 0
%     tau2_c = tau_c(1);
% else
%     tau2_c = tau_c(2);
% end
% 
% tau_c = [tau1_c, tau2_c]';
% 
% conservative_comp =  M*ddq + V;
% non_conservative_comp = tau_c + J_L'*atm_drag;
% 
% disp('------------------------ Final dynamic model -------------------')
% tau = conservative_comp - non_conservative_comp;
% disp('tau:')
% disp(vpa(tau,2))


fprintf('To get to the Trajectories press inv \n')
pause()
%% TRAJECTORIES
syms T real
syms s real % --> s = t/T in [0,1] (timing law)

target_poss = get_targets(orbit_ts, 'earth');

q_i = [pi;0];     

% To cumulate points
qss = [];
dqss = [];
ddqss = [];
pointss = [];
tot_time = 0;

if (verbosity == 1 || verbosity == 3)
    fprintf('\n\n ------------------------ Trajectories ---------------------------')
end

% Get full trajectory
for i = 1:size(target_poss,2)
    T = 1;     % TO tune when doing bang-cost-bang once in dynamics

    q_f = get_target_conf(q,p_EE,p_tip,cell2mat(target_poss(i)),theta_bounds, q_i);
    
    % Path planning
    [path_q_1, path_dq_1,path_ddq_1] = path_planning(q_i(1), q_f(1), 0, 0, 0, 0,s);
    [path_q_2, path_dq_2,path_ddq_2] = path_planning(q_i(2), q_f(2), 0, 0, 0, 0,s);
    path_q = [path_q_1;path_q_2];
    path_dq = [path_dq_1;path_dq_2];
    path_ddq = [path_ddq_1;path_ddq_2];

    % Timing law computation: 
    % Tune the T in oder to have the torque boundaries respected and then:
    %               t = s*T ....    traj = sub(path,s,t/T)
    tau_m = torque_motor([path_dq_1,path_dq_2],[path_ddq_1,path_ddq_2],ni,I_m,M,B_m);
    
    time_scaling = timing_law(q,s,path_q,tau_m,tau_max);

    if time_scaling>1
        T = T*time_scaling;
    end

    s_t = t/T;
    
    
    traj_q = subs(path_q,s,s_t);
    traj_dq = subs(path_dq,s,s_t);
    traj_ddq = subs(path_ddq,s,s_t);
%     tau_m_sub = subs(tau_m,s,s_t);

    if (verbosity == 1 || verbosity == 3)
        fprintf('\n - Trajectory %i\n',i)
        fprintf('   %s\n',vpa(traj_q,2));
    end

    % Discretization
    discr_interval = 1/sampling_rate;
    timesteps = (0:discr_interval:T)'; 
    [qs,dqs,ddqs, points] = get_trajectory_points(traj_q,traj_dq,traj_ddq, ...
                                                  t, timesteps, q, p_EE);
    
    % Plots
    if (verbosity == 2 || verbosity == 3)
        figure
        plot(timesteps,qs(:,1),'r',timesteps,qs(:,2),'b')
        title('Configurations of trajectory',i)
        
        figure
        plot(timesteps,dqs(:,1),'r',timesteps,dqs(:,2),'b')
        title('Velocities of trajectory',i)
        
        figure
        plot(timesteps,ddqs(:,1),'r',timesteps,ddqs(:,2),'b')
        title('Accelerations of trajectory',i)
    end

    % Concatenation
    q_i = q_f;
    qss = [qss;qs];
    dqss = [dqss;dqs];
    ddqss = [ddqss;dqs];
    pointss = [pointss; points];
    tot_time = tot_time +T;

end



fprintf('To get to the Control part press inv \n')
pause()
%% CONTROLLER

% NonLinear controller
% You receive a trajectory (traj_q, traj_dq, traj_ddq) in discrete time.
% For each timestep you get in the configuration: M, V, tau_c, jacobian.
% Then, you compute the input tau using the traj_ddq as command in acceleration.
% Continuing, you do the rest of the stuff by integrating the error bla bla
% bla.

atm_drag = [1,1,1]';        % TODO real one is needed
atm_drag_true = atm_drag + randn(3,1)/10;

timesteps = (0:discr_interval:tot_time)'; 
count_steps = length(timesteps);

commads = zeros(count_steps,2);
taus = zeros(count_steps,2);

error = zeros(count_steps,4);
q_0 = zeros(count_steps,2);
dq_0 = zeros(count_steps,2);

% first conf/velocity is 'distrurbed'
q_0(1,:)=qss(1,:)-randn(1,2)*pi/180;
dq_0(1,:)=dqss(1,:)-randn(1,2)*0.1*pi/180;

q_d = qss;
dq_d = dqss;
ddq_d = ddqs;
trajectory = pointss;

options = odeset('RelTol', 1.0E-4, 'AbsTol', 1.0E-4);

for i = 1:count_steps-1
    fprintf('Step %i \n',i)
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

    

    [t_int,error_int] = ode113(@(time,error)controller(time, error, q_d, dq_d, ddq_d, ...
                            q,dq, timesteps, k_p, k_d, ...
                            M, V,tau_c,J_L,atm_drag, ...
                            M_true,V_true,atm_drag_true), ...
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
    plot_robot_traj(robot, q_0, trajectory, cell2mat(target_poss(size(target_poss,2))),q,p_EE,p_tip)
end





















