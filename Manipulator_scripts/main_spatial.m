% I-MIM project - manipulator main


%% INSTANTIATIONS
init;
verbosity = 2;
%Verbosity gives what is printed/shown:
% - 0: Nothing 
% - 1: kinematic/dynamics components (only)
% - 2: plots (only)
% - 3: (1) and (2)



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


%% DYNAMIC COMPONENTS

[M, V, B, C] = dynamic_model(q, dq, ddq, m, l, d, I1,I2);

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





%% DYNAMIC MODEL

% syms tau1 tau2 tau3 tau1_c tau2_c real
% 
% atm_drag = [1,1,1]';
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
traj_time = 0;

if (verbosity == 1 || verbosity == 3)
    fprintf('\n\n ------------------------ Trajectories ---------------------------')
end

% Get full trajectory
for i = 1:size(target_poss,2)
    T = 1;     % TO tune when doing bang-cost-bang once in dynamics

    [q_f] = get_target_conf(q,p_EE,p_tip,cell2mat(target_poss(i)),theta_bounds, q_i);

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

    if (verbosity == 1 || verbosity == 3)
        fprintf('\n - Trajectory %i\n',i)
        fprintf('   %s\n',vpa(traj_q,2));
    end

    % Discretization
    discr_interval = 1/sampling_rate;
    time_steps = (0:discr_interval:T)'; 
    [qs,dqs,ddqs, points] = get_trajectory_points(traj_q,traj_dq,traj_ddq, ...
                                                  t, time_steps, q, p_EE);
    
    % Plots
    if (verbosity == 2 || verbosity == 3)
        figure
        plot(time_steps,qs(:,1),'r',time_steps,qs(:,2),'b')
        title('Configurations of trajectory',i)
        
        figure
        plot(time_steps,dqs(:,1),'r',time_steps,dqs(:,2),'b')
        title('Velocities of trajectory',i)
        
        figure
        plot(time_steps,ddqs(:,1),'r',time_steps,ddqs(:,2),'b')
        title('Accelerations of trajectory',i)
    end

    % Concatenation
    q_i = q_f;
    qss = [qss;qs];
    dqss = [dqss;dqs];
    ddqss = [ddqss;dqs];
    pointss = [pointss; points];
    traj_time = traj_time +T;

end

if (verbosity == 2 || verbosity == 3)
    plot_robot_traj(robot, qss, pointss, cell2mat(target_poss(size(target_poss,2))),q,p_EE,p_tip)
end



%% CONTROLLER

% You receive a trajectory (traj_q, traj_dq, traj_ddq) in discrete time.
% For each timestep you get in the configuration: M, V, tau_c, jacobian.
% Then, you compute the input tau using the traj_ddq as command in acceleration.
% Continuing, you do the rest ofthe stuff by integrating the error bla bla
% bla.

time_steps = (0:discr_interval:traj_time)'; 
count_steps = length(time_steps);

for i = 1:count_steps
    qi = qss(i,:)';
    dqi = dqss(i,:)';
    ddqi = ddqss(i,:)';
    
    % Compute all dynamic components in this timestep
    M_i = subs(M,q,qi);
    V_i = subs(V,q,qi);
    V_i = subs(V_i,dq,dqi);
    tau_c = [(1 + sign(dqi(1)))/2 *tauC_plus + (1 - sign(dqi(1)))/2 *tauC_minus;
             (1 + sign(dqi(2)))/2 *tauC_plus + (1 - sign(dqi(2)))/2 *tauC_minus];
    J_i = subs(J_L,q,qi);
   

    %TO DO 
    %Build the controller part in ored to compute the effective acc command...
    
    % Get the value of the errors from integration:
    %           e = (qi-q), de = (dqi-dq) 
    % and then compute the command in acceleration
    %           command = ddq_i + K_d*de + K_p*e

    % Finally, get the command in torque using the dynamic model
    % beta = V_i + tau_c;
    % tau = M_i*command + beta - J_i'*atm_drag;

    

end






















