function [q_f, q_ss, dq_ss, ddq_ss, pointss, tot_time] = get_trajectory_in_orbit(q,dq, t, ...
    q_i, q_ss, dq_ss, ddq_ss, pointss, tot_time, target_poss, M, position_EE, L_s, ...
    ni, I_m, B_m, tau_max,tau_c_sym, theta_bounds, l, sampling_rate)
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
        T = 1;     % TO tune when doing bang-cost-bang once in dynamics


        q_f = get_target_conf(target_poss(:,i),theta_bounds, q_i, l, L_s);
        % Path planning
        [path_q_1, path_dq_1,path_ddq_1] = path_planning(q_i(1), q_f(1), 0, 0, 0, 0,s);
        [path_q_2, path_dq_2,path_ddq_2] = path_planning(q_i(2), q_f(2), 0, 0, 0, 0,s);
        path_q = [path_q_1;path_q_2];
        path_dq = [path_dq_1;path_dq_2];
        path_ddq = [path_ddq_1;path_ddq_2];
    
        % Timing law computation: 
        % Tune the T in oder to have the torque boundaries respected and then:
        %               t = s*T ....    traj = sub(path,s,t/T)
        tau_c = subs(tau_c_sym,dq,path_dq);

        tau_m = torque_motor([path_dq_1,path_dq_2],[path_ddq_1,path_ddq_2],ni,I_m,M,B_m,tau_c);
        
        time_scaling = timing_law(q,s,path_q,tau_m,tau_max);
        
        if time_scaling>1
            T = T*time_scaling;
        end
    
        s_t = t/T;
        
        
        traj_q = subs(path_q,s,s_t);
        traj_dq = subs(path_dq,s,s_t);
        traj_ddq = subs(path_ddq,s,s_t);
    %     tau_m_sub = subs(tau_m,s,s_t);
   
    
        % Discretization
        discr_interval = 1/sampling_rate;
        timesteps = (0:discr_interval:T)'; 
        [qs,dqs,ddqs, points] = get_trajectory_points(traj_q,traj_dq,traj_ddq, ...
                                                      t, timesteps, q, position_EE);
        
        % Plots

%         remaining_time = 10 - T;
%         
%         sampling_time = 1/sampling_rate;
%         num_of_confs = remaining_time/sampling_time;
% 
%         remaining_qs = qs(end,:).*ones(num_of_confs, size(qs(end, :),2));
%         remaining_dqs = dqs(end,:).*ones(num_of_confs, size(dqs(end, :),2));
%         remaining_ddqs = ddqs(end,:).*ones(num_of_confs, size(ddqs(end, :),2));
%         qs = [qs;remaining_qs];
%         dqs = [dqs; remaining_dqs];
%         ddqs = [ddqs; remaining_ddqs];

        % Concatenation
        q_i = q_f;
        q_ss = [q_ss;qs(2:end, :)];
        dq_ss = [dq_ss;dqs(2:end, :)];
        ddq_ss = [ddq_ss;ddqs(2:end, :)];
        pointss = [pointss; points];
        tot_time = tot_time +T;
    
    end
end