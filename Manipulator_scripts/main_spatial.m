% I-MIM project - manipulator main

init;

%% CREATING THE ROBOT

% DH table [alpha_i d_i a_i theta_i] (original method)

DHTABLE = [pi/2  l1  0 q1 ;
           -pi/2  l2  0 q2];

%% KINEMATICS
[T_EE,R_EE,p_EE] = forward_kinematics(DHTABLE);
%disp(T_EE)
disp('---------------------- Forward kinematics ---------------------')
disp(p_EE)

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
target = [pi/6 pi/3 pi];;   % target orientation of the EE
q_i = [0;0];     
T = 30;
[phi, theta, psi] = get_target_orientation(q,R_EE,target)
q_f = [theta; psi]

disp('------------------------ Trajectory ---------------------------')

[traj_q_1,traj_dq_1,traj_ddq_1] = quintic_poly_traj(q_i(1), q_f(1),0,0,0,0,t,T);
[traj_q_2,traj_dq_2,traj_ddq_2] = quintic_poly_traj(q_i(2), q_f(2)',0,0,0,0,t,T);
traj_q = [traj_q_1;traj_q_2]

DHparam = [pi/2  l1  0 q_i(1) ;
    -pi/2 l2  0 q_i(2)];
plot_robot_traj(DHparam,traj_q,q,p_EE,t,T)

% [traj_q, traj_dq, traj_ddq] = optimal_traj(q,q_i,theta_low_bounds',theta_high_bounds',target,t,T);
% disp(vpa(traj_q,2));
% disp(vpa(traj_dq,2))
% disp(vpa(traj_ddq,2))
% 
% DHparam = [0  l1  0 q_i(1) ;
%            0  l2  0 q_i(2)];
% 
% plot_robot_traj(DHparam,traj_q,q,p_EE,t,T)





























