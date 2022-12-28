% I-MIM project - manipulator main

init;

%% CREATING THE ROBOT

% DH table [alpha_i d_i a_i theta_i] (original method)

[DHTABLE,T_i_b]= DH_generator(l,q);

q_i =[0;0];
robot = create_robot(l,q_i);


%% KINEMATICS
[T_EE,R_EE,p_EE] = forward_kinematics(DHTABLE,T_i_b);
T_tip = T_EE*trvec2tform([1,0,0]);
p_tip = T_tip(1:3,4);

disp('---------------------- Forward kinematics ---------------------')
disp(p_EE)

% Jacobian (given p_EE forward kinematics)
disp('----------------------- Jacobian matrix -----------------------')
J_a = analitic_jacobian(p_EE,q');
disp('Analitic Jacobian:')
disp(vpa(J_a,2))
[J_L,J_A] = geometric_jacobian(DHTABLE,q',['r','r']);
J_g = [J_L;J_A] ;
disp('Geometric Jacobian:')
disp(J_g)


%% IMPORTANT TRANSFORMATIONS

% T_EE:  BASE frame -> EE frame
% T_i_b : INERTIA frame -> BASE frame
% T_o_i:  LVLH frame -> INERTIA frame

R_o_i = [0 0 1;0 -1 0;1 0 0];
T_o_i = [[R_o_i,[0;0;0]];0 0 0 1]; % LVLH frame -> INERTIA frame


%% DYNAMICS

[M, V, B, C] = dynamic_model(q, dq, ddq, m, l, d, I1,I2);
disp('------------------------ Inertia matrix ------------------------')
disp(vpa(M,3))
disp('-----------------Coriolis/centrifucgal terms -------------------')
disp(vpa(V,3))
disp('---------------- Splitting V in B and C ------------------------')
disp(vpa(B,3))
disp(vpa(C,3))

% % Accounting for friction
% tau_f = c_f*sign(dq) + v_f*dq;

% Motor torques after reduction (links side)
tau = M*ddq + V;%+ tau_f;
disp('------------------------ Final dynamic model -------------------')
disp(vpa(tau,3))


%% TRAJECTORIES
syms T real
syms s real % --> s = t/T in [0,1] (timing law)

target_poss = get_targets(orbit_ts, 'earth');

q_i = [pi;0];     

% To cumulate points
qss = [];
pointss = [];

% Get full trajectory
for i = 1:size(target_poss,2)
    T = 1;     % TO tune when doing bang-cost-bang once in dynamics

    [q_f] = get_target_conf(q,p_EE,p_tip,cell2mat(target_poss(i)),theta_bounds, q_i);

%     [traj_q_1,traj_dq_1,traj_ddq_1] = quintic_poly_traj(q_i(1), q_f(1),0,0,0,0,t,T);
%     [traj_q_2,traj_dq_2,traj_ddq_2] = quintic_poly_traj(q_i(2), q_f(2)',0,0,0,0,t,T)
%      
%     traj = [traj_q_1, traj_q_2];
%     disp(vpa(traj_q,2));
    
    % Path planning
    [path_q_1, path_dq_1,path_ddq_1] = path_planning(q_i(1), q_f(1), 0, 0, 0, 0,s);
    [path_q_2, path_dq_2,path_ddq_2] = path_planning(q_i(2), q_f(2), 0, 0, 0, 0,s);
    path = [path_q_1,path_q_2];
    % Timing law computation: 
    % Tune the T in oder to have the torque boundaries respected and then:
    %               t = s*T ....    traj = sub(path,s,t/T)
    tau_m = torque_motor([path_dq_1,path_dq_2],[path_ddq_1,path_ddq_2],ni,I_m,M,B_m);
    
    T = timing_law(q,s,t,path,tau_m,tau_max)
   
    s_t = t/T;
    
    disp('------------------------ Trajectory ---------------------------')
    traj = subs(path,s,s_t);
    disp(vpa(traj,2));
   
    [qs, points] = get_trajectory_points(traj, t, T, q, p_EE);
    % PLOTS
    
    q_i = q_f;

    qss = [qss;qs];
    pointss = [pointss; points];

end



plot_robot_traj(robot, qss, pointss, cell2mat(target_poss(size(target_poss,2))),q,p_EE,p_tip)



























