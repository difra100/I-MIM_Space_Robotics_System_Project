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

% disp('---------------------- Forward kinematics ---------------------')
% disp(p_EE)

% Jacobian (given p_EE forward kinematics)
J=jacobian(p_EE, q');
disp('----------------------- Jacobian matrix -----------------------')
disp(J)


%% DYNAMICS

[M, V, B, C] = dynamic_model(q, dq, ddq, m, l, d, I1,I2);
disp('------------------------ Inertia matrix ------------------------')
disp(vpa(M,3))
disp('-----------------Coriolis/centrifucgal terms -------------------')
disp(vpa(V,3))
disp('---------------- Splitting V in B and C ------------------------')
disp(vpa(B,3))
disp(vpa(C,3))

% Accounting for friction
tau_f = c_f*sign(dq) + v_f*dq;

% Motor torques after reduction (links side)
tau = M*ddq + V + tau_f;
disp('------------------------ Final dynamic model -------------------')
disp(vpa(tau,3))
% Motor torques before reduction (motor side)
temp1 = [M(1,1)/ni_1^2 , 0; 0, M(2,2)/ni_2^2]; % diag(M(ii)/ni_i^2)
temp2 = [0, M(1,2);M(2,1),0];        % M - M_ii
%ni_diag = [1/n_1,0;0,1/n_2];         % 
%ni_2 = [1/n_1^2,1/n_2^2];
%tau_m = (I_m + M_ii*)

%% TRAJECTORIES
target_pos = [10,30,20];
q_i = [0;0];     
T = 10;
q_f = get_target_conf(q,q_i,p_EE,p_tip,target_pos);


[traj_q_1,traj_dq_1,traj_ddq_1] = quintic_poly_traj(q_i(1), q_f(1),0,0,0,0,t,T);
[traj_q_2,traj_dq_2,traj_ddq_2] = quintic_poly_traj(q_i(2), q_f(2)',0,0,0,0,t,T);
traj_q = [traj_q_1;traj_q_2];
disp('------------------------ Trajectory ---------------------------')
disp(vpa(traj_q,2));
%% PLOTS
plot_robot_traj(robot,traj_q,target_pos,q,p_EE,p_tip,t,T);































