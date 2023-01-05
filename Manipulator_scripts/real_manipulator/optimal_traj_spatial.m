function [traj_q, traj_dq, traj_ddq] = optimal_traj_spatial(q,q_i,q_low_bounds,q_high_bounds,target,t,T)

% INPUTS:
%   - q = [q1,q2]: joint vars
%   - q_i = [q1_i,q2_i]: initial configuration 
%   - target = q_1+q_2: target orientation (float)
%   - t: time symbolic variable
%   - T: total time (int)

%% OPTIMAL INVERSE SOLUTION

[traj_q1, traj_dq1, traj_ddq1] = quintic_poly_traj(q_i(1), q_f.q1,  0, 0, 0, 0, t,T);
[traj_q2, traj_dq2, traj_ddq2] = quintic_poly_traj(q_i(2), q_f.q2,  0, 0, 0, 0, t,T);

traj_q = [traj_q1;traj_q2];
traj_dq = [traj_dq1;traj_dq2];
traj_ddq = [traj_ddq1;traj_ddq2];
                                                                                                                                                                                                                                                                          
end