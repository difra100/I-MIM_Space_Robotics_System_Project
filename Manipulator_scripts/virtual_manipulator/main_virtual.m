%% Virtual Manipulator Section 
init;
% [DHTABLE, T_lvlh_b, ] = DH_generator_VM(l,q, theta_v, true, m, m_s)

q_i = [0,0,0,0,0];


% disp(' Virtual E-E position:  ')
% vpa(norm(subs(virtual_M, Q_augm, q_i)), 6)
% disp(' Real E-E position:  ')
% vpa(norm(subs(p_EE_j2000, Q_augm, q_i)), 6)

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











