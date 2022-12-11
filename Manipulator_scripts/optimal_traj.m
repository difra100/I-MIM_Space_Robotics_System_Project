function [traj_q, traj_dq, traj_ddq] = optimal_traj(q,q_i,q_low_bounds,q_high_bounds,target,t,T)

% INPUTS:
%   - q = [q1,q2]: joint vars
%   - q_i = [q1_i,q2_i]: initial configuration 
%   - target = q_1+q_2: target orientation (float)
%   - t: time symbolic variable
%   - T: total time (int)

%% OPTIMAL INVERSE SOLUTION

% find the q_f = [q1_f,q2_f] which minimize the delta 
q1 = optimvar('q1');
q2 = optimvar('q2');
prob = optimproblem;
prob.Objective = (q1-q_i(1))^2 + (q2-q_i(2))^2;

%joint boundaries
prob.Constraints.cons1 = -q1 <= -q_low_bounds(1);
prob.Constraints.cons2 = q1 <= q_high_bounds(1);
prob.Constraints.cons3 = -q2 <= -q_low_bounds(2);
prob.Constraints.cons4 = q2 <= q_high_bounds(2);

% target conf constarint
prob.Constraints.cons5 = q1+q2 <= target;
prob.Constraints.cons6 = -(q1+q2) <= -target;

q_f = solve(prob);  % Since we have redundancy we use the remaining dof to compute a solution that is not too far from our target.


[traj_q1, traj_dq1, traj_ddq1] = quintic_poly_traj(q_i(1), q_f.q1,  0, 0, 0, 0, t,T);
[traj_q2, traj_dq2, traj_ddq2] = quintic_poly_traj(q_i(2), q_f.q2,  0, 0, 0, 0, t,T);

traj_q = [traj_q1;traj_q2];
traj_dq = [traj_dq1;traj_dq2];
traj_ddq = [traj_ddq1;traj_ddq2];
                                                                                                                                                                                                                                                                          
end