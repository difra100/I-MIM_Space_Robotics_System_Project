function [q_f, q_ss, dq_ss, ddq_ss, pointss, tot_time] = get_trajectory_in_orbit_VM( ...
    q_i, q_ss, dq_ss, ddq_ss, pointss, tot_time, ...
    target_poss, M, ni, I_m, B_m, tau_max, ...
    position_EE, theta_bounds, l_0,l)
    % This function computes the optimal joint configuration in order to
    % point at a certain planet.
    % INPUTs : q_i : Starting configuration, q_ss: time series of the
        % previous configuration trajectory, dq_ss: time series of the
        % previous velocity trajectory, ddq_ss: time series of the
        % previous acceleration trajectory. tot_time: total time of the
        % trajectory before the new maneuvre, target_poss: Planet position,
        % M, V, B, C are the dynamical parameters, position_EE: This is the E-E
        % position, in the robot base frame.
    % OUTPUTs : q_f : Final pointing configuration, the other are the same
        % as before, but now they are computed after the new maneuvre.

    syms s real  % --> s = t/T in [0,1] (timing law)
    for i = 1:size(target_poss,2)
        T = 1;     
    
        q_f = get_target_conf_VM(cell2mat(target_poss(i)),theta_bounds, q_i, l_0,l);
        
        % Path planning
        [path_theta_1, path_dtheta_1,path_ddtheta_1] = path_planning(q_i(1), q_f(1), 0, 0, 0, 0,s); % theta1
        [path_theta_2, path_dtheta_2,path_ddtheta_2] = path_planning(q_i(2), q_f(2), 0, 0, 0, 0,s); % theta2
        [path_theta_3, path_dtheta_3,path_ddtheta_3] = path_planning(q_i(3), q_f(3), 0, 0, 0, 0,s); % theta3
        [path_q_1, path_dq_1,path_ddq_1] = path_planning(q_i(4), q_f(4), 0, 0, 0, 0,s); % q1
        [path_q_2, path_dq_2,path_ddq_2] = path_planning(q_i(5), q_f(5), 0, 0, 0, 0,s); % q2
        path_Q = [path_theta_1; path_theta_2; path_theta_3; path_q_1;path_q_2];
        path_dQ = [path_dtheta_1; path_dtheta_2; path_dtheta_3; path_dq_1;path_dq_2];
        path_ddQ = [path_ddtheta_1;path_ddtheta_2;path_ddtheta_3;path_ddq_1;path_ddq_2];
    
        % Timing law computation: 
        % Tune the T in oder to have the torque boundaries respected and then:
        %               t = s*T ....    traj = sub(path,s,t/T)
    
        s_t = t/T;
        
        
        traj_Q = subs(path_Q,s,s_t);
        traj_dQ = subs(path_dQ,s,s_t);
        traj_ddQ = subs(path_ddQ,s,s_t);
    %     tau_m_sub = subs(tau_m,s,s_t);
   
    
        % Discretization
        discr_interval = 1/sampling_rate;
        timesteps = (0:discr_interval:T)'; 
        [qs,dqs,ddqs, points] = get_trajectory_points(traj_Q,traj_dQ,traj_ddQ, ...
                                                      t, timesteps, q, position_EE);
        
        % Plots
        
        % Concatenation
        q_i = q_f;
        q_ss = [q_ss;qs];
        dq_ss = [dq_ss;dqs];
        ddq_ss = [ddq_ss;ddqs];
        pointss = [pointss; points];
        tot_time = tot_time +T;
    
    end
end